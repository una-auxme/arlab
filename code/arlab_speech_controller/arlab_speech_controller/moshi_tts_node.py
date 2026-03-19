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
        elif self.delay_steps > 0:
            self.state_clean = False
            self.silent_steps = 0
            self.tts_gen.step()
            self.delay_steps -= 1
        elif self.silent_steps < self.tts_gen.audio_silent_steps:
            samples = self.tts_gen.step()
            if samples is not None:
                if self.tts_gen.is_audio_silent(samples):
                    self.silent_steps += 1
                else:
                    self.silent_steps = 0
        else:
            if not self.state_clean:
                self.state_clean = True
                self._restore_model_state()

            # Start the next sentence
            if len(self.sentence_buffer) > 0 and len(self.sentence_buffer[0].strip()) == 0:
                self.get_logger().info("Switched to new sentence.")
                self.sentence_buffer.popleft()

    def _restore_model_state(self):
        self.get_logger().info(f"Restored state at offset: {self.tts_gen.offset}")
        self.tts_gen.restore_state()
        # self.tts_gen.reset_state()

    def _start_tts_model(self):
        self.tts_gen.init_streaming()
        self.get_logger().info(f"Initial offset: {self.tts_gen.offset}")
        self.audio_enabled = True
        self.create_timer(0.05, self._step_timer_callback)

    def _setup_audio_output(self):
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
        self.audio_output_stream.start()

    def _tts_output_sub_callback(self, msg: String):
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
        self.tts_model.mimi.reset_streaming()
        self.audio_output_stream.stop()
        self.audio_output_stream.close()


@torch.no_grad()
@no_compile()
def main(args=None):
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
