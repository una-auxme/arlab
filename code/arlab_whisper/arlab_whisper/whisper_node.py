"""
This package allows speech recognition using Faster-Whisper with wakeword detection.
It continuously monitors microphone input, detects configurable wakewords,
transcribes them and publishes the result
"""

#!/usr/bin/env python3

import os
import tempfile
import time
import wave
from collections import deque
from enum import Enum, auto

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
    Pipeline execution states for the speech processing pipeline
    """

    IDLE = 1
    CHECK_SPEECH = 2
    CHECK_WAKEWORD = 3
    RECORD_COMMAND = 4
    TRANSCRIBE_COMMAND = 5


class WhisperNode(Node):
    """
    ros2 node for wakeword detection and speech transcription
    """

    def __init__(self):
        """
        loads parameters, initializes the wakeword and whisper models,
        configures the audio stream creates ros publishers and subscription
        and starts the pipeline

        """
        super().__init__("whisper")

        # load all parameters from a yaml file
        self.load_parameters()

        # check if the zirbi wakewords should be added to the wakeword detection
        self.checkedWakewords = self.wakewords
        if self.use_zirbi_wakewords:
            self.checkedWakewords += self.zirbi_wakewords

        # set the pipeline start
        self.state = PipelineState.CHECK_SPEECH
        self.listening_enabled = True
        self.last_detection_time = 0.0
        self.command_audio = []
        self.recording_command = False
        self.record_buffer = []

        # load the whisper models
        self.get_logger().info("Loading wakeword model")
        self.wakeword_model = WhisperModel(
            self.wakeword_model_name,
            device=self.device,
            compute_type=self.compute_type,
        )

        self.get_logger().info("loading command model")
        self.command_model = WhisperModel(
            self.command_model_name,
            device=self.device,
            compute_type=self.compute_type,
        )

        # init the audio input buffer
        self.audio_buffer = deque(maxlen=int(self.sample_rate * self.rolling_buffer_seconds))

        # ros subscriptions/publisher
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

        # init the audio input stream
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype=np.int16,
            callback=self.audio_callback,
        )
        # start listening
        self.stream.start()

        # init the timer to start the pipline process
        self.timer = self.create_timer(self.check_interval, self.process_pipeline)

        self.get_logger().info("Whisper node ready")

    def load_parameters(self):
        """
        declares all configurable parameters with default values and loads the
        parameter values provided by the launch file via a yaml file
        """

        # declare all default parameters

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

        # load the yaml parameters loaded in the launch file

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
        en- or disables wakeword detection and the pipeline through a recieved ros2 message

        args: msg (Bool): ros bool msg

        """

        self.listening_enabled = msg.data
        self.get_logger().info(f"Listening ON/OFF: {self.listening_enabled}")

    def audio_callback(self, indata, frames, time_info, status):
        """
        receives audio chunks from the audio input stream, stores them in the
        rolling audio buffer

        Args:
            indata (numpy.ndarray): incoming audio data
            frames (int): number of audio frames
            time_info (CData): timing information
            status: status information
        """

        # retrun issues
        if status:
            self.get_logger().warning(str(status))

        samples = indata[:, 0]

        # fill the buffer
        self.audio_buffer.extend(samples)

        # if record is enabled
        if self.recording_command:
            self.record_buffer.extend(samples)

    def process_pipeline(self):
        """
        Main pipeline structure

        checks the current pipeline state and executes the corresponding processingstep
        includs speech detection, wakeword detection command recording and
        command transcription
        """

        # returns if the pipeline is disables
        if not self.listening_enabled:
            self.state = PipelineState.IDLE
            return

        # starts the pipeline if it was disabled
        if self.state == PipelineState.IDLE:
            self.state = PipelineState.CHECK_SPEECH

        # waits till the audio buffer is full, checks if speech is detected
        # if this is the case switches the state to check for a wakeword
        if self.state == PipelineState.CHECK_SPEECH:
            if len(self.audio_buffer) < self.sample_rate:
                return
            if self.detect_speech():
                self.state = PipelineState.CHECK_WAKEWORD
            return

        # tries to detect a wakeword, if this was the case and no other wakeword was detected for some time
        # switch the state and save the input buffer
        if self.state == PipelineState.CHECK_WAKEWORD:
            audio = np.array(self.audio_buffer, dtype=np.int16)
            if self.detect_wakeword(audio):
                now = time.time()
                if now - self.last_detection_time > self.rolling_buffer_seconds:
                    self.last_detection_time = now
                    self.get_logger().info("Wakeword detected")
                    # save old buffer
                    self.command_audio = list(self.audio_buffer)
                    self.state = PipelineState.RECORD_COMMAND
            else:
                self.state = PipelineState.CHECK_SPEECH
            return

        # records a command if triggerd and switch to transcribe
        if self.state == PipelineState.RECORD_COMMAND:
            self.command_audio = self.record_command()
            self.state = PipelineState.TRANSCRIBE_COMMAND
            return

        # transcibes and sends the command resets the pipeline
        if self.state == PipelineState.TRANSCRIBE_COMMAND:
            transcript = self.transcribe_command(self.command_audio)
            if transcript:
                self.send_transcript(transcript)
            self.command_audio = []
            self.state = PipelineState.CHECK_SPEECH
            return

    def detect_speech(self):
        """ "
        calculates the "energy" of the rolling audio buffer
        and compares it vs the configured speech threshold

        return: bool: true if the detected audio exeeds the threshold, oetherwise False.
        """

        audio = np.array(self.audio_buffer, dtype=np.float32)
        rms = np.sqrt(np.mean(audio**2))
        if rms < self.speech_threshold:
            return False
        return True

    def detect_wakeword(self, audio):
        """detects whether a configured wakeword is part of the audio using a whisper model
        Args:
            audio (numpy.ndarray): audio sample

        Returns:
            bool: true if a configured wakeword was detected, otherweise false
        """

        # save the audio temporarly and run the whisper model
        wav_path = self.save_temp_wav(audio)
        try:
            segments, info = self.wakeword_model.transcribe(
                wav_path,
                language=self.language,
                beam_size=self.beam_size,
                # initial_prompt="zirbi"
            )
            """
            #debug output of the transcript
            for segment in segments:
                self.get_logger().info(
                    f"debug: {segment.text}"
                )
            """
            # refactor the text to all lowercase
            text = " ".join(segment.text for segment in segments).lower()

            # self.get_logger().debug(f"Wakeword model: {text}")

            # extract all used words from the text
            words = re.findall(r"[a-zäöüß]+", text.lower())

            # check if the refactored words fit a wakeword word beginning
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
        initializes the command recording buffer with the audio captured before the
        wakeword
        continues recording incoming audio for some time

        return:
            numpy.ndarray: recorded command audio samples
        """

        self.record_buffer = list(self.command_audio)
        self.recording_command = True
        time.sleep(self.command_record_seconds)
        self.recording_command = False

        audio = np.array(self.record_buffer, dtype=np.int16)
        return audio

    def save_temp_wav(self, audio):
        """
        creates a temporary WAV file from the provided audio data
        Args: audio (numpy.ndarray): audio sample

        Return: str: path to the created tfile
        """
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
        filename = temp_file.name
        temp_file.close()
        with wave.open(filename, "wb") as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(2)
            wf.setframerate(elf.sample_rate)
            wf.writeframes(audio.tobytes())
        return filename

    def transcribe_command(self, audio):
        """
        converts the provided audio data into a temporary -wav fileand runs the
        whisper model to generate a transcription

        args: audio (numpy.ndarray): recorded audio
        return: str: transcribed text, empty if the transcript fails
        """

        # run model
        wav_path = self.save_temp_wav(audio)
        try:
            segments, info = self.command_model.transcribe(
                wav_path, language=self.language, beam_size=self.beam_size, initial_prompt="Your name is Zirbi."
            )
            # combines the output into one clean string
            text = " ".join(segment.text for segment in segments)
            text = text.strip()
            return text

        except Exception as e:
            self.get_logger().error(f"command transcriptio error: {e}")
            return ""

        finally:
            try:
                os.remove(wav_path)
            except Exception:
                pass

    def send_transcript(self, transcript):
        """
        creates a WhisperTranscript message from the provided text and publishes it

        args: transcript (str): transcribed text to publish

        """
        if not transcript:
            return
        msg = WhisperTranscript()
        msg.transcript = transcript
        self.transcript_publisher.publish(msg)
        self.get_logger().info(f"Published transcript: {transcript}")

    def destroy_node(self):
        """
        cleans up resources before shutting down node.
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
