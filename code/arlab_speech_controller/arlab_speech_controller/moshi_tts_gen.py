# Based on https://github.com/kyutai-labs/delayed-streams-modeling/blob/8fb54b2b07a34a852e148ad11b820e7526c48b59/scripts/tts_pytorch_streaming.py
# The TTSGen is based heavily on moshi's TTSModel class.

import typing as tp
from collections import deque
from copy import deepcopy
from dataclasses import dataclass

import numpy as np
import torch
from moshi.conditioners import dropout_all_conditions
from moshi.models.lm import LMGen
from moshi.models.tts import (
    ConditionAttributes,
    Entry,
    TTSModel,
    _delayed,
    script_to_entries,
)


def prepare_script(model: TTSModel, script: str, first_turn: bool) -> list[Entry]:
    multi_speaker = first_turn and model.multi_speaker
    return script_to_entries(
        model.tokenizer,
        model.machine.token_ids,
        model.mimi.frame_rate,
        [script],
        multi_speaker=multi_speaker,
        padding_between=1,
    )


def _make_null(
    all_attributes: tp.Sequence[ConditionAttributes],
) -> list[ConditionAttributes]:
    # When using CFG, returns the null conditions.
    return dropout_all_conditions(all_attributes)


def _backup_attrs(object: tp.Any, attr_names: tp.List[str]) -> tp.Dict[str, tp.Any]:
    result = {}
    for attr_name in attr_names:
        copied_attr = deepcopy(getattr(object, attr_name))
        result[attr_name] = copied_attr
    return result


def _restore_attrs(object: tp.Any, attr_dict: tp.Dict[str, tp.Any]):
    for key, value in attr_dict.items():
        if hasattr(object, key):
            setattr(object, key, value)


@dataclass
class TTSGen:
    tts_model: TTSModel
    attributes: tp.Sequence[ConditionAttributes]
    prefixes: list[torch.Tensor] | None = None
    cfg_is_no_prefix: bool = True
    cfg_is_no_text: bool = True
    on_frame: tp.Optional[tp.Callable[[np.ndarray], None]] = None
    audio_silent_threshold: float = 0.01
    audio_silent_steps: int = 5

    def __post_init__(self):
        tts_model = self.tts_model
        attributes = self.attributes
        self.offset = 0

        self.state = self.tts_model.machine.new_state([])

        if tts_model.cfg_coef != 1.0:
            if tts_model.valid_cfg_conditionings:
                raise ValueError(
                    "This model does not support direct CFG, but was trained with "
                    "CFG distillation. "
                    "Pass instead `cfg_coef` to `make_condition_attributes`."
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

    def init_streaming(self):
        self.lm_gen.streaming_forever(1)
        self.tts_model.mimi.streaming_forever(1)

        self.append_text("Hello.")

        silent_frame_counter = 0

        # Wait until the audio is silent after the initial preconditioning
        while silent_frame_counter < self.audio_silent_steps:
            samples = self.step()
            if samples is not None:
                if self.is_audio_silent(samples):
                    silent_frame_counter += 1
                else:
                    silent_frame_counter = 0

        # Save the state after the preconditioning
        self.backup_state()

    def backup_state(self):
        if self.lm_gen._streaming_state is not None:
            self.lm_gen_state_backup = _backup_attrs(
                self.lm_gen._streaming_state,
                [
                    "offsets",
                    "offset_cpu",
                ],
            )
        if self.lm_gen.lm_model._streaming_state is not None:
            self.lm_model_state_backup = deepcopy(
                self.lm_gen.lm_model.get_streaming_state()
            )
        self.state_backup = deepcopy(self.state)
        self.offset_backup = self.offset

    def restore_state(self):
        self.state = deepcopy(self.state_backup)
        self.offset = self.offset_backup
        _restore_attrs(self.lm_gen._streaming_state, self.lm_gen_state_backup)
        self.lm_gen.lm_model.set_streaming_state(deepcopy(self.lm_model_state_backup))

    def reset_state(self):
        self.state = self.tts_model.machine.new_state([])
        self.lm_gen.reset_streaming()
        self.offset = 0

    def process(self):
        while len(self.state.entries) > self.tts_model.machine.second_stream_ahead:
            self.step()

    def step(self) -> tp.Optional[np.ndarray]:
        missing = self.tts_model.lm.n_q - self.tts_model.lm.dep_q
        input_tokens = torch.full(
            (1, missing, 1),
            self.tts_model.machine.token_ids.zero,
            dtype=torch.long,
            device=self.tts_model.lm.device,
        )
        frame = self.lm_gen.step(input_tokens)
        self.offset += 1
        if frame is not None and (frame != -1).all():
            # Converts frame into audio samples
            pcm = self.tts_model.mimi.decode(frame[:, 1:, :]).cpu().numpy()
            result = np.clip(pcm[0, 0], -1, 1)
            if self.on_frame is not None:
                self.on_frame(result)
            return result
        return None

    def append_entry(self, entry):
        self.state.entries.append(entry)

    def append_text(self, msg: str):
        entries = prepare_script(self.tts_model, msg.strip(), first_turn=False)
        self.tts_first_turn = False
        for entry in entries:
            self.append_entry(entry)
            self.process()

    def is_audio_silent(self, audio_samples: np.ndarray):
        max = np.max(np.abs(audio_samples))
        return max <= self.audio_silent_threshold
