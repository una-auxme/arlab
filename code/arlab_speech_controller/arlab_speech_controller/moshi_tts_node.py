import queue

import numpy as np
import rclpy
import sounddevice as sd
import torch
from moshi.models.loaders import CheckpointInfo
from moshi.models.tts import (
    DEFAULT_DSM_TTS_REPO,
    DEFAULT_DSM_TTS_VOICE_REPO,
    Entry,
    TTSModel,
    script_to_entries,
)
from moshi.utils.compile import no_compile, no_cuda_graph
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String

from .moshi_tts_gen import TTSGen


class MoshiTTS(Node):
    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        self.hf_repo = (
            self.declare_parameter(
                "hf_repo",
                "kyutai/tts-0.75b-en-public",
                # DEFAULT_DSM_TTS_REPO,
                descriptor=ParameterDescriptor(
                    description="HF repo in which to look for the pretrained models."
                ),
            )
            .get_parameter_value()
            .string_value
        )
        self.voice_repo = (
            self.declare_parameter(
                "voice_repo",
                DEFAULT_DSM_TTS_VOICE_REPO,
                descriptor=ParameterDescriptor(
                    description="HF repo in which to look for "
                    "pre-computed voice embeddings."
                ),
            )
            .get_parameter_value()
            .string_value
        )
        self.voice = (
            self.declare_parameter(
                "voice",
                "unmute-prod-website/degaulle-2.wav",
                descriptor=ParameterDescriptor(
                    description="The voice to use, relative to the voice repo root. "
                    f"See {DEFAULT_DSM_TTS_VOICE_REPO}"
                ),
            )
            .get_parameter_value()
            .string_value
        )

        self.device = (
            self.declare_parameter(
                "device",
                "cuda",
                descriptor=ParameterDescriptor(
                    description="Device on which to run, defaults to 'cuda'."
                    f"See {DEFAULT_DSM_TTS_VOICE_REPO}"
                ),
            )
            .get_parameter_value()
            .string_value
        )

        self.tts_first_turn = True
        self.delay_steps: int = 0
        self.pcms_audio_queue = queue.Queue()
        self.audio_enabled: bool = False

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
        self.tts_model = TTSModel.from_checkpoint_info(
            checkpoint_info, n_q=16, temp=0.6, cfg_coef=3, device=self.device
        )

        if self.voice.endswith(".safetensors"):
            voice_path = self.voice
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

        self.tts_gen = TTSGen(
            self.tts_model, [], on_frame=_on_frame, prefixes=self.prefixes
        )

    def _step_timer_callback(self):
        if self.pcms_audio_queue.qsize() > 5:
            return

        num_entries = len(self.tts_gen.state.entries)
        if num_entries > 0 or (
            self.tts_gen.state.end_step is not None
            and self.tts_gen.offset < self.tts_gen.state.end_step
        ):
            self.tts_gen.step()
            self.delay_steps = (
                # Adding 16 delay_steps to make sure all outputs are processed. This number seems arbitrary,
                # but the original library uses an value of 8 which was not enough in our case.
                self.tts_model.delay_steps + max(self.tts_model.lm.delays) + 16
            )
        elif self.delay_steps > 0:
            self.tts_gen.step()
            self.delay_steps -= 1
        else:
            self.tts_gen.restore_start_state()
            # self.tts_gen.reset_state()

        self.get_logger().info(
            f"offset: {self.tts_gen.offset}, skip: {self.tts_gen.prefix_skip}"
        )

    def _start_tts_model(self):
        self.tts_gen.init_streaming()
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
        self.tts_gen.append_text(data)

    def shutdown(self):
        self.tts_model.mimi.reset_streaming()
        self.audio_output_stream.stop()
        self.audio_output_stream.close()


@torch.no_grad()
@no_compile()
@no_cuda_graph()
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
