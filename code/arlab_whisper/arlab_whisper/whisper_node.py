#!/usr/bin/env python3

"""
ARLAB Whisper

Maintainer: tobias.neher@uni-a.de

This module implements a ROS 2 speech-to-text (STT) pipeline based on the Faster-Whisper library
with integrated wakeword detection.

The node continuously processes microphone input and uses two separate Whisper
models for wakeword detection and speech transcription.

The processing pipeline consists of the following steps:
- Audio input is continuously received and stored in a rolling buffer.
- An threshold is used to detect audio activity.
- A lightweight Whisper model processes the audio to detect configured wakewords.
- Once a wakeword is detected, the current audio buffer is saved and additional
  audio is recorded for a configurable duration.
- A second Whisper model transcribes the recorded audio into text.
- The resulting transcript is published via a ROS 2 topic.

Wakewords, Whisper models, audio settings and transcription
parameters can be configured via ROS 2 parameters.

The node publishes recognized speech commands as WhisperTranscript messages.
"""


import os
import tempfile
import time
import wave
from collections import deque
from enum import Enum

import numpy as np
import re
import rclpy
import sounddevice as sd

from faster_whisper import WhisperModel

from rclpy.node import Node
from std_msgs.msg import Bool

from arlab_common_interfaces.msg import WhisperTranscript


class PipelineState(Enum):
    """
    Pipeline execution states for the speech processing pipeline.
    """

    IDLE = 1
    CHECK_SPEECH = 2
    CHECK_WAKEWORD = 3
    RECORD_COMMAND = 4
    TRANSCRIBE_COMMAND = 5


class WhisperNode(Node):
    """
    ROS 2 node for wakeword detection and speech transcription.
    """

    def __init__(self):
        """
        Loads parameters, initializes the wakeword and whisper models,
        configures the audio stream, creates ROS publishers and subscription
        and starts the pipeline.

        """
        super().__init__("whisper")

        # Load all parameters from a YAML file.
        self.load_parameters()

        # Check whether the Zirbi wake words should be added to the wake word detection.
        self.checkedWakewords = self.wakewords.copy()
        if self.use_zirbi_wakewords:
            self.checkedWakewords += self.zirbi_wakewords

        # Set the initial pipeline state.
        self.state = PipelineState.CHECK_SPEECH
        self.listening_enabled = True
        self.last_detection_time = 0.0
        self.command_audio = []
        self.recording_command = False
        self.record_buffer = []

        # Load the Whisper models.
        self.get_logger().info("Loading wakeword model")
        self.wakeword_model = WhisperModel(
            self.wakeword_model_name,
            device=self.device,
            compute_type=self.compute_type,
        )

        self.get_logger().info("Loading command model")
        self.command_model = WhisperModel(
            self.command_model_name,
            device=self.device,
            compute_type=self.compute_type,
        )

        # Initialize the audio input buffer.
        self.audio_buffer = deque(maxlen=int(self.sample_rate * self.rolling_buffer_seconds))

        # Initialize ROS subscriptions and publishers.
        self.create_subscription(
            Bool,
            "/whisper/enabled",
            self.enabled_callback,
            10,
        )
        self.transcript_publisher = self.create_publisher(
            WhisperTranscript,
            "/whisper/transcript",
            10,
        )

        # Initialize the audio input stream.
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype=np.int16,
            callback=self.audio_callback,
        )
        # Start listening.
        self.stream.start()

        # Initialize the timer to start the pipeline process.
        self.timer = self.create_timer(self.check_interval, self.process_pipeline)

        self.get_logger().info("Whisper node ready")

    def load_parameters(self):
        """
        Declares all configurable parameters with default values and loads the
        parameter values provided by the launch file via a YAML file.
        """

        # Declare all parameters with their default values.

        self.declare_parameter("wakewords", ["robot"])
        self.declare_parameter("use_zirbi_wakewords", True)
        self.declare_parameter("zirbi_wakewords", ["zirbi"])
        self.declare_parameter("wakeword_model", "tiny.en")
        self.declare_parameter("command_model", "large-v3-turbo")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("compute_type", "int8")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("rolling_buffer_seconds", 3.0)
        self.declare_parameter("command_record_seconds", 4.0)
        self.declare_parameter("speech_threshold", 150)
        self.declare_parameter("check_interval", 0.25)
        self.declare_parameter("language", "en")
        self.declare_parameter("beam_size", 5)

        # Load the parameter values provided by the launch file.

        self.wakewords = [word.lower() for word in self.get_parameter("wakewords").value]

        self.use_zirbi_wakewords = self.get_parameter("use_zirbi_wakewords").value

        self.zirbi_wakewords = [word.lower() for word in self.get_parameter("zirbi_wakewords").value]

        self.wakeword_model_name = self.get_parameter("wakeword_model").value
        self.command_model_name = self.get_parameter("command_model").value
        self.device = self.get_parameter("device").value
        self.compute_type = self.get_parameter("compute_type").value
        self.sample_rate = self.get_parameter("sample_rate").value
        self.channels = self.get_parameter("channels").value
        self.rolling_buffer_seconds = self.get_parameter("rolling_buffer_seconds").value
        self.command_record_seconds = self.get_parameter("command_record_seconds").value
        self.speech_threshold = self.get_parameter("speech_threshold").value
        self.check_interval = self.get_parameter("check_interval").value
        self.language = self.get_parameter("language").value
        self.beam_size = self.get_parameter("beam_size").value

    def enabled_callback(self, msg: Bool):
        """
        Enables or disables wakeword detection and the pipeline through a received ROS 2 message.

        args: msg (Bool): ROS 2 Bool message
        """

        self.listening_enabled = msg.data
        self.get_logger().info(f"Listening ON/OFF: {self.listening_enabled}")

    def audio_callback(self, indata, frames, time_info, status):
        """
        Receives audio chunks from the audio input stream, stores them in the
        rolling audio buffer.

        args:
            indata (numpy.ndarray): Incoming audio data
            frames (int): Number of audio frames
            time_info (CData): Timing information
            status: Status information
        """

        # Handle audio stream status warnings.
        if status:
            self.get_logger().warning(str(status))

        samples = indata[:, 0]

        # Fill the rolling buffer.
        self.audio_buffer.extend(samples)

        # Append samples to the recording buffer if recording is enabled.
        if self.recording_command:
            self.record_buffer.extend(samples)

    def process_pipeline(self):
        """
        Main pipeline structure

        Checks the current pipeline state and executes the corresponding processing step.
        Includes speech detection, wakeword detection, command recording and command transcription.
        """

        # Returns if the pipeline is disabled.
        if not self.listening_enabled:
            self.state = PipelineState.IDLE
            return

        # Starts the pipeline if it was disabled.
        if self.state == PipelineState.IDLE:
            self.state = PipelineState.CHECK_SPEECH

        # Waits until the audio buffer is full, checks if speech is detected.
        # If this is the case, switches the state to check for a wakeword.
        if self.state == PipelineState.CHECK_SPEECH:
            if len(self.audio_buffer) < self.sample_rate:
                return
            if self.detect_speech():
                self.state = PipelineState.CHECK_WAKEWORD
            return

        # Tries to detect a wakeword, if this was the case and no other wakeword was detected for some time.
        # Switch the state and save the input buffer.
        if self.state == PipelineState.CHECK_WAKEWORD:
            audio = np.array(self.audio_buffer, dtype=np.int16)
            if self.detect_wakeword(audio):
                now = time.time()
                if now - self.last_detection_time > self.rolling_buffer_seconds:
                    self.last_detection_time = now
                    self.get_logger().info("Wakeword detected")
                    # Save the current audio buffer.
                    self.command_audio = list(self.audio_buffer)
                    self.state = PipelineState.RECORD_COMMAND
            else:
                self.state = PipelineState.CHECK_SPEECH
            return

        # Records a command if triggered and switches to transcribe.
        if self.state == PipelineState.RECORD_COMMAND:
            self.command_audio = self.record_command()
            self.state = PipelineState.TRANSCRIBE_COMMAND
            return

        # Transcribes and sends the command. Then resets the pipeline.
        if self.state == PipelineState.TRANSCRIBE_COMMAND:
            transcript = self.transcribe_command(self.command_audio)
            if transcript:
                self.send_transcript(transcript)
            self.command_audio = []
            self.state = PipelineState.CHECK_SPEECH
            return

    def detect_speech(self):
        """
        Calculates the "energy" of the rolling audio buffer
        and compares it against the configured speech threshold.

        return: bool: True if the detected audio exceeds the threshold, otherwise False.
        """

        audio = np.array(self.audio_buffer, dtype=np.float32)
        rms = np.sqrt(np.mean(audio**2))
        if rms < self.speech_threshold:
            return False
        return True

    def detect_wakeword(self, audio):
        """
        Detects whether a configured wakeword is part of the audio using a Whisper model
        args:
            audio (numpy.ndarray): Audio sample

        returns:
            bool: True if a configured wakeword was detected, otherwise False.
        """

        # Save the audio temporarily and run the whisper model.
        wav_path = self.save_temp_wav(audio)
        try:
            segments, info = self.wakeword_model.transcribe(
                wav_path,
                language=self.language,
                beam_size=self.beam_size,

                # An initial_prompt="zirbi" is possible at this point.
            )

            # Convert the transcript to lowercase.
            text = " ".join(segment.text for segment in segments).lower()

            # Extract all words from the transcript.
            words = re.findall(r"[a-zäöüß]+", text.lower())

            # Check if the refactored words fit the beginning of a wakeword.
            for wakeword_i in self.checkedWakewords:
                wakeword = wakeword_i.lower()
                for word in words:
                    if word == wakeword:
                        return True
                    if word.startswith(wakeword):
                        return True
            return False

        except Exception as e:
            self.get_logger().error(f"wakeword detect error: {e}")
            return False

        finally:
            try:
                os.remove(wav_path)
            except Exception:
                pass

    def record_command(self):
        """
        Initializes the command recording buffer with the audio captured before the
        wakeword.
        Continues recording incoming audio for some time.

        return:
            numpy.ndarray: Recorded command audio samples.
        """

        self.record_buffer = list(self.command_audio)
        self.recording_command = True
        time.sleep(self.command_record_seconds)
        self.recording_command = False

        audio = np.array(self.record_buffer, dtype=np.int16)
        return audio

    def save_temp_wav(self, audio):
        """
        Creates a temporary WAV file from the provided audio data.
        args: audio (numpy.ndarray): Audio samples

        return: str: Path to the created temporary file.
        """
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
        filename = temp_file.name
        temp_file.close()
        with wave.open(filename, "wb") as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio.tobytes())
        return filename

    def transcribe_command(self, audio):
        """
        Converts the provided audio data into a temporary WAV file and runs the
        Whisper model to generate a transcription.

        args: audio (numpy.ndarray): Recorded audio
        return: str: Transcribed text, empty if the transcript fails.
        """

        # Run the transcription model.
        wav_path = self.save_temp_wav(audio)
        try:
            segments, info = self.command_model.transcribe(
                wav_path, language=self.language, beam_size=self.beam_size, initial_prompt="Your name is Zirbi."
            )
            # Combine the transcription segments into a single string.
            text = " ".join(segment.text for segment in segments)
            text = text.strip()
            return text

        except Exception as e:
            self.get_logger().error(f"Command transcription error: {e}")
            return ""

        finally:
            try:
                os.remove(wav_path)
            except Exception:
                pass

    def send_transcript(self, transcript):
        """
        Creates a WhisperTranscript message from the provided text and publishes it.

        args: transcript (str): Transcribed text to publish.
        """
        if not transcript:
            return
        msg = WhisperTranscript()
        msg.transcript = transcript
        self.transcript_publisher.publish(msg)
        self.get_logger().info(f"Published transcript: {transcript}")

    def destroy_node(self):
        """
        Cleans up resources before shutting down the node.
        """
        try:
            self.stream.stop()
            self.stream.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    node = WhisperNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
