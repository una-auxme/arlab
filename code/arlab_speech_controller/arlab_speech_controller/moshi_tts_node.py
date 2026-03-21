"""Moshi TTS node

Node functionality:
    Text-to-speech (TTS) inference using the Moshi model.
    Reads parameters from ROS2 parameter server for model configuration.
    Subscribes to text topics and publishes audio output via sounddevice.
    Handles sentence splitting/merging and streaming audio generation.

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

import queue
from collections import deque
from typing import List

import rclpy
import sounddevice as sd
import torch
from moshi.models.loaders import CheckpointInfo
from moshi.models.tts import (
    # DEFAULT_DSM_TTS_REPO,
    DEFAULT_DSM_TTS_VOICE_REPO,
    TTSModel,
)
from moshi.utils.compile import no_compile
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String

from .moshi_tts_gen import TTSGen


def split_into_sentences(text: str) -> List[str]:
    """Split text into sentences based on punctuation delimiters.

    Iteratively splits the text by each delimiter in order (., !, ?, :, ;, ,),
    keeping the delimiter attached to preceding text segments. Returns a list
    of sentence fragments where each fragment ends with its delimiter (if any).

    Args:
        text (str): Input text string to split into sentences.

    Returns:
        List[str]: List of sentence fragments, each ending with its delimiter.
    """
    delimiters = [".", "!", "?", ":", ";", ","]
    sentences = [text]
    for delimiter in delimiters:
        splits = []
        for sentence in sentences:
            new_splits = sentence.split(delimiter)
            for i in range(len(new_splits) - 1):
                new_splits[i] += delimiter
            splits += new_splits

        sentences = splits
    return sentences


class MoshiTTS(Node):
    """ROS2 node for text-to-speech inference using the Moshi model.

    This node loads a pretrained TTS model from Hugging Face, subscribes to
    text topics, and generates speech audio output using sounddevice. It handles
    sentence splitting/merging based on configurable length parameters and
    manages streaming audio generation with state restoration for long outputs.

    Parameters:
        hf_repo (str): Hugging Face repository containing the pretrained TTS model.
            Defaults to "kyutai/tts-0.75b-en-public".
        voice_repo (str): Hugging Face repository containing pre-computed voice embeddings.
        voice (str): Voice file path relative to the voice repo root, or a local path.
            See DEFAULT_DSM_TTS_VOICE_REPO for available options.
        device (str): Device to run inference on ("cuda", "cpu", etc.). Defaults to "cuda".
        max_offset (int): Maximum frames the model can produce without state restore.
            Higher values support longer continuous text but may degrade quality.
            Defaults to 500.
        max_sentence_length (int): Maximum characters per sentence. Sentences exceeding
            this length are split, while those below half this length are merged.
            Defaults to 200.
    """

    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        self.hf_repo = (
            self.declare_parameter(
                "hf_repo",
                "kyutai/tts-0.75b-en-public",
                # DEFAULT_DSM_TTS_REPO,
                descriptor=ParameterDescriptor(description="HF repo in which to look for the pretrained models."),
            )
            .get_parameter_value()
            .string_value
        )
        self.voice_repo = (
            self.declare_parameter(
                "voice_repo",
                DEFAULT_DSM_TTS_VOICE_REPO,
                descriptor=ParameterDescriptor(description="HF repo in which to look for pre-computed voice embeddings."),
            )
            .get_parameter_value()
            .string_value
        )
        self.voice = (
            self.declare_parameter(
                "voice",
                "unmute-prod-website/degaulle-2.wav",
                descriptor=ParameterDescriptor(
                    description=f"The voice to use, relative to the voice repo root. See {DEFAULT_DSM_TTS_VOICE_REPO}"
                ),
            )
            .get_parameter_value()
            .string_value
        )

        self.device = (
            self.declare_parameter(
                "device",
                "cuda",
                descriptor=ParameterDescriptor(description=f"Device on which to run, defaults to 'cuda'.See {DEFAULT_DSM_TTS_VOICE_REPO}"),
            )
            .get_parameter_value()
            .string_value
        )

        self.max_offset = (
            self.declare_parameter(
                "max_offset",
                500,
                descriptor=ParameterDescriptor(
                    description="Max amount of frames (offset) the model "
                    "is allowed to produce in one go without a state restore. "
                    "More supports longer continuous text output, "
                    "but the model deteriorates quickly at some point"
                ),
            )
            .get_parameter_value()
            .integer_value
        )

        self.max_sentence_length = (
            self.declare_parameter(
                "max_sentence_length",
                200,
                descriptor=ParameterDescriptor(
                    description="Max number of characters in one sentence. "
                    "Bigger sentences will be split. "
                    "Sentences smaller than 0.5*max_sentence_length will be merged."
                ),
            )
            .get_parameter_value()
            .integer_value
        )

        self.delay_steps: int = 0
        self.silent_steps: int = 0
        self.state_clean: bool = True

        self.pcms_audio_queue = queue.Queue()
        self.audio_enabled: bool = False

        self.sentence_buffer: deque[str] = deque()

        self._setup_tts_model()
        self._setup_audio_output()

        self._start_audio_output()
        self._start_tts_model()

        self.create_subscription(
            String,
            topic="tts_output",
            callback=self._tts_output_sub_callback,
            qos_profile=10,
        )

        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _setup_tts_model(self):
        """Initialize the TTS model and audio generation components.

        Loads the pretrained TTS model from Hugging Face, configures voice embeddings,
        sets up the TTSGen for streaming audio output, and prepares the audio callback.
        """
        self.get_logger().info("Loading model...")
        checkpoint_info = CheckpointInfo.from_hf_repo(self.hf_repo)
        # This initialization is required for the bigger 1.6b model
        # self.tts_model = TTSModel.from_checkpoint_info(
        #     checkpoint_info, n_q=32, temp=0.6, device=self.device
        # )
        self.tts_model = TTSModel.from_checkpoint_info(checkpoint_info, n_q=16, temp=0.6, cfg_coef=3, device=self.device)

        if self.voice.endswith(".safetensors"):
            voice_path = self.voice
        elif self.voice.startswith("local:"):
            voice_path = self.voice.removeprefix("local:")
        else:
            voice_path = self.tts_model.get_voice_path(self.voice)
        self.get_logger().info(f"Using voice path: {voice_path}")
        self.prefixes = [self.tts_model.get_prefix(voice_path)]
        # This initialization is required for the bigger 1.6b model
        # CFG coef goes here because the model was trained with CFG distillation,
        # so it's not _actually_ doing CFG at inference time.
        # Also, if you are generating a dialog, you should have two voices in the list.
        # condition_attributes = self.tts_model.make_condition_attributes(
        #     [voice_path], cfg_coef=2.0
        # )

        def _on_frame(audio_samples):
            if self.audio_enabled:
                self.pcms_audio_queue.put_nowait(audio_samples)

        self.tts_gen = TTSGen(self.tts_model, [], on_frame=_on_frame, prefixes=self.prefixes)

    def _step_timer_callback(self):
        """Timer callback for streaming TTS generation.

        Checks audio queue size, manages model state restoration when offset exceeds
        max_offset, processes sentences from the buffer, and advances the TTS generation
        step. Handles silent periods to detect completion of audio output.
        """
        if self.pcms_audio_queue.qsize() > 5:
            return

        # Make sure we don't generate too much in one go
        if self.tts_gen.offset >= self.max_offset:
            self._restore_model_state()

        # Queue tokens from the current sentence (First entry in the sentence buffer)
        if len(self.sentence_buffer) > 0 and len(self.sentence_buffer[0].strip()) > 0:
            self.get_logger().info(f"Added text to model: {self.sentence_buffer[0]}")
            self.tts_gen.append_text(self.sentence_buffer[0])
            self.sentence_buffer[0] = ""

        num_entries = len(self.tts_gen.state.entries)
        # Case 1: Active generation - there are entries in the generation state
        # or we're still within the end_step of a previous entry.
        # Continue generating audio and set up delay steps for audio processing.
        if num_entries > 0 or (self.tts_gen.state.end_step is not None and self.tts_gen.offset < self.tts_gen.state.end_step):
            self.state_clean = False
            self.silent_steps = 0
            self.tts_gen.step()
            self.delay_steps = (
                # The delay steps are usually not sufficient to
                # make sure all audio gets processed.
                # First we tired adding arbitrary numbers (16 worked pretty well)
                # Now we just wait until the output is silent
                self.tts_model.delay_steps + max(self.tts_model.lm.delays)
            )
        # Case 2: Waiting for audio to drain - we have pending delay steps
        # to wait out before processing the next generation step.
        elif self.delay_steps > 0:
            self.state_clean = False
            self.silent_steps = 0
            self.tts_gen.step()
            self.delay_steps -= 1
        # Case 3: Checking for silence - no active entries and no pending delays,
        # so we step to check if audio output has become silent. Increment
        # silent_steps counter if silence is detected, reset if audio plays.
        elif self.silent_steps < self.tts_gen.audio_silent_steps:
            samples = self.tts_gen.step()
            if samples is not None:
                if self.tts_gen.is_audio_silent(samples):
                    self.silent_steps += 1
                else:
                    self.silent_steps = 0
        # Case 4: Generation complete - no active entries, no pending delays,
        # and silence threshold exceeded. Clean up state and prepare for
        # the next sentence from the buffer.
        else:
            if not self.state_clean:
                self.state_clean = True
                self._restore_model_state()

            # Start the next sentence
            if len(self.sentence_buffer) > 0 and len(self.sentence_buffer[0].strip()) == 0:
                self.get_logger().info("Switched to new sentence.")
                self.sentence_buffer.popleft()

    def _restore_model_state(self):
        """Restore the TTS model state after exceeding max_offset.

        Logs a message with the current offset and calls restore_state() on the
        TTSGen instance to reset the generation state for continued output.
        """
        self.get_logger().info(f"Restored state at offset: {self.tts_gen.offset}")
        self.tts_gen.restore_state()

    def _start_tts_model(self):
        """Initialize and start the TTS model streaming.

        Calls init_streaming() on the TTSGen, logs the initial offset, enables
        audio output, and creates a 50ms timer for the step callback.
        """
        self.tts_gen.init_streaming()
        self.get_logger().info(f"Initial offset: {self.tts_gen.offset}")
        self.audio_enabled = True
        self.create_timer(0.05, self._step_timer_callback)

    def _setup_audio_output(self):
        """Set up the audio output stream using sounddevice.

        Creates an OutputStream with a callback that reads PCM audio data from
        the queue and writes it to the output. If the queue is empty, outputs silence.
        Uses the TTS model's sample rate and 1920 block size for mono audio.
        """

        def audio_callback(outdata, _a, _b, _c):
            # self.get_logger().info("Audio callback received.")
            try:
                pcm_data = self.pcms_audio_queue.get(block=False)
                outdata[:, 0] = pcm_data
            except queue.Empty:
                outdata[:] = 0

        self.audio_output_stream = sd.OutputStream(
            samplerate=self.tts_model.mimi.sample_rate,
            blocksize=1920,
            channels=1,
            callback=audio_callback,
        )

    def _start_audio_output(self):
        """Start the audio output stream.

        Starts the audio_output_stream to begin audio playback.
        """
        self.audio_output_stream.start()

    def _tts_output_sub_callback(self, msg: String):
        """Callback for text-to-speech output topic subscription.

        Receives text messages on the "tts_output" topic, splits them into sentences,
        merges/splits based on max_sentence_length parameters, and adds to the sentence
        buffer for TTS generation. Logs the received text message.

        Args:
            msg (String): ROS2 String message containing text to convert to speech.
        """
        data = msg.data
        self.get_logger().info(f"TTS: {data}")
        sentences = deque(split_into_sentences(data))
        if len(sentences) > 0 and len(self.sentence_buffer) > 0:
            self.sentence_buffer[len(self.sentence_buffer) - 1] += sentences.popleft()
        self.sentence_buffer += sentences

        # Split/merge sentences based on min/max length
        max_length = self.max_sentence_length
        min_length = self.max_sentence_length // 2
        idx = 0
        while idx < len(self.sentence_buffer):
            sentence = self.sentence_buffer[idx]
            s_len = len(sentence)
            if s_len > max_length:
                self.sentence_buffer[idx] = sentence[s_len // 2 :]
                self.sentence_buffer.insert(idx, sentence[: s_len // 2])
            elif s_len < min_length and idx + 1 < len(self.sentence_buffer):
                self.sentence_buffer[idx] += self.sentence_buffer[idx + 1]
                del self.sentence_buffer[idx + 1]
            else:
                idx += 1

    def shutdown(self):
        """Shutdown the TTS node and release resources.

        Resets the Mimi streaming state, stops the audio output stream, and closes
        the sounddevice stream to free system resources.
        """
        self.tts_model.mimi.reset_streaming()
        self.audio_output_stream.stop()
        self.audio_output_stream.close()


@torch.no_grad()
@no_compile()
def main(args=None):
    """Initialize ROS2, create the MoshiTTS node, and spin it.

    This function initializes the ROS2 context, creates a MoshiTTS node instance,
    and spins the node to handle callbacks. Gracefully handles keyboard interrupts
    by exiting cleanly.

    The @no_compile() decorator is required because Jetson devices do not support
    torch.compile (via Triton), which would cause runtime errors on these platforms.

    Args:
        args (list, optional): Command line arguments passed to rclpy.init().
    """
    # from arlab_common.debugging import start_debugger

    # start_debugger(wait_for_client=True)

    rclpy.init(args=args)
    try:
        node = MoshiTTS()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
