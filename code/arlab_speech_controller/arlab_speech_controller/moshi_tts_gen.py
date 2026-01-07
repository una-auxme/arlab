# Based on https://github.com/kyutai-labs/delayed-streams-modeling/blob/8fb54b2b07a34a852e148ad11b820e7526c48b59/scripts/tts_pytorch_streaming.py
# The TTSGen is based heavily on moshi's TTSModel class.

import typing as tp
from collections import deque
from dataclasses import dataclass

import torch
from moshi.conditioners import dropout_all_conditions
from moshi.models.lm import LMGen
from moshi.models.tts import ConditionAttributes, TTSModel, _delayed


def _make_null(
    all_attributes: tp.Sequence[ConditionAttributes],
) -> list[ConditionAttributes]:
    # When using CFG, returns the null conditions.
    return dropout_all_conditions(all_attributes)


@dataclass
class TTSGen:
    tts_model: TTSModel
    attributes: tp.Sequence[ConditionAttributes]
    prefixes: list[torch.Tensor] | None = None
    cfg_is_no_prefix: bool = True
    cfg_is_no_text: bool = True
    on_frame: tp.Optional[tp.Callable[[torch.Tensor], None]] = None

    def __post_init__(self):
        tts_model = self.tts_model
        attributes = self.attributes
        self.offset = 0
        self.on_frame_output_active: bool = False

        # Taken from https://huggingface.co/kyutai/tts-0.75b-en-public
        if self.prefixes is not None:
            # self.prefix_skip = int(
            #     (tts_model.mimi.sample_rate * self.prefixes[0].shape[-1])
            #     / tts_model.mimi.frame_rate
            # )
            self.prefix_skip = int(self.prefixes[0].shape[-1])
            print(self.prefix_skip)
        else:
            self.prefix_skip = 0

        self.state = self.tts_model.machine.new_state([])

        if tts_model.cfg_coef != 1.0:
            if tts_model.valid_cfg_conditionings:
                raise ValueError(
                    "This model does not support direct CFG, but was trained with "
                    "CFG distillation. Pass instead `cfg_coef` to `make_condition_attributes`."
                )
            nulled = _make_null(attributes)
            attributes = list(attributes) + nulled

        assert tts_model.lm.condition_provider is not None
        prepared = tts_model.lm.condition_provider.prepare(attributes)
        condition_tensors = tts_model.lm.condition_provider(prepared)

        cfg_is_masked_until = None
        text_prefixes = None
        audio_prefixes = None
        device = tts_model.lm.device
        if self.prefixes is not None:
            if self.cfg_is_no_prefix:
                cfg_is_masked_until = []
            text_prefixes = []
            audio_prefixes = []
            for prefix in self.prefixes:
                if cfg_is_masked_until is not None:
                    cfg_is_masked_until.append(prefix.shape[-1] + tts_model.delay_steps)
                K, _ = prefix.shape
                assert K == tts_model.lm.num_codebooks
                text_prefixes.append(deque(prefix[0].cpu().tolist()))
                delays = [
                    d + tts_model.delay_steps
                    for d in tts_model.lm.delays[tts_model.lm.audio_offset :]
                ]
                delayed = _delayed(
                    prefix[tts_model.lm.audio_offset :],
                    delays,
                    tts_model.machine.token_ids.ungenerated,
                )
                delayed = delayed.to(device)
                audio_prefixes.append(deque(delayed.t()))

        def _on_text_logits_hook(text_logits):
            if tts_model.padding_bonus:
                text_logits[..., tts_model.machine.token_ids.pad] += (
                    tts_model.padding_bonus
                )
            return text_logits

        def _on_audio_hook(audio_tokens):
            audio_offset = tts_model.lm.audio_offset
            delays = tts_model.lm.delays
            ungenerated = tts_model.machine.token_ids.ungenerated
            for q in range(audio_tokens.shape[1]):
                delay = delays[q + audio_offset]
                if self.offset < delay + tts_model.delay_steps:
                    audio_tokens[:, q] = tts_model.machine.token_ids.zero
            if audio_prefixes is not None:
                for b, audio_prefix in enumerate(audio_prefixes):
                    if audio_prefix:
                        audio_codes = audio_prefix.popleft()
                        mask = audio_codes != ungenerated
                        audio_tokens[b] = torch.where(
                            mask, audio_codes, audio_tokens[b]
                        )

        def _on_text_hook(text_tokens):
            tokens = text_tokens.tolist()
            out_tokens = []
            for b, (token, state) in enumerate(zip(tokens, [self.state])):
                if text_prefixes is not None and text_prefixes[b]:
                    out_token = text_prefixes[b].popleft()
                else:
                    out_token, _ = tts_model.machine.process(self.offset, state, token)
                out_tokens.append(out_token)
            text_tokens[:] = torch.tensor(
                out_tokens, dtype=torch.long, device=text_tokens.device
            )

        tts_model.lm.dep_q = tts_model.n_q
        self.lm_gen = LMGen(
            tts_model.lm,
            temp=tts_model.temp,
            temp_text=tts_model.temp,
            cfg_coef=tts_model.cfg_coef,
            condition_tensors=condition_tensors,
            on_text_logits_hook=_on_text_logits_hook,
            on_text_hook=_on_text_hook,
            on_audio_hook=_on_audio_hook,
            cfg_is_masked_until=cfg_is_masked_until,
            cfg_is_no_text=True,
        )
        self.lm_gen.streaming_forever(1)

        for _ in range(self.prefix_skip):
            self.step()
        self.on_frame_output_active = True

    def reset_state(self):
        self.state = self.tts_model.machine.new_state([])
        self.lm_gen.reset_streaming()
        self.offset = 0

        # if self.current_text_prefixes is not None:
        #     self.current_text_prefixes.clear()
        #     self.current_text_prefixes.append(self.default_text_prefixes)
        # if self.current_audio_prefixes is not None:
        #     self.current_text_prefixes.clear()
        #     self.current_text_prefixes.append(self.default_text_prefixes)

    def process(self):
        while len(self.state.entries) > self.tts_model.machine.second_stream_ahead:
            self.step()

    def step(self):
        missing = self.tts_model.lm.n_q - self.tts_model.lm.dep_q
        input_tokens = torch.full(
            (1, missing, 1),
            self.tts_model.machine.token_ids.zero,
            dtype=torch.long,
            device=self.tts_model.lm.device,
        )
        frame = self.lm_gen.step(input_tokens)
        self.offset += 1
        if frame is not None and self.on_frame_output_active:
            if self.on_frame is not None:
                self.on_frame(frame)

    def append_entry(self, entry):
        self.state.entries.append(entry)
