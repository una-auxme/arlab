"""Moshi TTS generation node

Node functionality:
    Generates streaming TTS audio using the Moshi model.
    Supports continuous streaming output, restoring model state and silence detection.

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

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
from moshi.utils.compile import CUDAGraphed


def prepare_script(model: TTSModel, script: str, first_turn: bool) -> list[Entry]:
    """Prepare a text script for TTS generation.

    Converts a text script into a list of Entry objects that can be processed
    by the TTS model's state machine. Handles multi-speaker configuration and
    adds padding between entries.

    Parameters:
        model (TTSModel): The TTS model containing tokenizer and machine configurations.
        script (str): The text script to convert into entries.
        first_turn (bool): Whether this is the first turn of speech generation.
            Used to determine multi-speaker configuration.

    Returns:
        list[Entry]: A list of Entry objects representing the prepared script.
    """
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
    """Create null condition attributes for CFG (classifier-free guidance).

    When using classifier-free guidance, null conditions are needed for the
    CFG branch. This function returns null condition attributes by dropping
    all conditions from the input sequence.

    Parameters:
        all_attributes (Sequence[ConditionAttributes]): The full sequence of
            condition attributes to create null versions of.

    Returns:
        list[ConditionAttributes]: A list of null condition attributes.
    """
    # When using CFG, returns the null conditions.
    return dropout_all_conditions(all_attributes)


def _backup_attrs(object: tp.Any, attr_names: tp.List[str]) -> tp.Dict[str, tp.Any]:
    """Backup specified attributes from an object with deep copying.

    Creates deep copies of the specified attributes and stores them in a
    dictionary. This is used for saving model state before operations that
    might modify it.

    Parameters:
        object (Any): The object containing attributes to backup.
        attr_names (List[str]): List of attribute names to backup.

    Returns:
        Dict[str, Any]: A dictionary mapping attribute names to their deep-copied values.
    """
    result = {}
    for attr_name in attr_names:
        copied_attr = deepcopy(getattr(object, attr_name))
        result[attr_name] = copied_attr
    return result


def _restore_attrs(object: tp.Any, attr_dict: tp.Dict[str, tp.Any]):
    """Restore attributes from a dictionary to an object.

    Restores previously backed-up attributes from a dictionary back to the
    original object. Only restores attributes that exist on the target object.

    Parameters:
        object (Any): The object to restore attributes onto.
        attr_dict (Dict[str, Any]): Dictionary mapping attribute names to their values.
    """
    for key, value in attr_dict.items():
        if hasattr(object, key):
            setattr(object, key, value)


@dataclass
class TTSGen:
    """TTS generator for streaming audio generation with Moshi model.

    This class wraps a TTSModel and manages the streaming generation process,
    including state management and silence detection.
    It supports classifier-free guidance (CFG), text/audio prefixes
    for zero-shot transfer, and continuous streaming output.

    Attributes:
        tts_model (TTSModel): The underlying TTS model for audio generation.
        attributes (Sequence[ConditionAttributes]): Condition attributes for
            speaker and other conditioning parameters.
        prefixes (Optional[List[Tensor]]): Optional text/audio prefixes for
            zero-shot transfer from reference audio.
        cfg_is_no_prefix (bool): Whether CFG uses null prefix instead of prefix.
        cfg_is_no_text (bool): Whether CFG uses null text condition.
        on_frame (Optional[Callable]): Callback invoked for each generated audio frame.
        audio_silent_threshold (float): Threshold for detecting silent audio frames.
        audio_silent_steps (int): Number of consecutive silent frames to wait.
    """

    tts_model: TTSModel
    attributes: tp.Sequence[ConditionAttributes]
    prefixes: list[torch.Tensor] | None = None
    cfg_is_no_prefix: bool = True
    cfg_is_no_text: bool = True
    on_frame: tp.Optional[tp.Callable[[np.ndarray], None]] = None
    audio_silent_threshold: float = 0.01
    audio_silent_steps: int = 5

    def __post_init__(self):
        """Initialize the TTS generator after attribute validation.

        Sets up the streaming state, handles CFG condition setup if needed,
        prepares condition tensors, and configures hooks for text/audio processing.
        Validates that the model supports the requested configuration.
        """
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
                delays = [d + tts_model.delay_steps for d in tts_model.lm.delays[tts_model.lm.audio_offset :]]
                delayed = _delayed(
                    prefix[tts_model.lm.audio_offset :],
                    delays,
                    tts_model.machine.token_ids.ungenerated,
                )
                delayed = delayed.to(device)
                audio_prefixes.append(deque(delayed.t()))

        def _on_text_logits_hook(text_logits):
            if tts_model.padding_bonus:
                text_logits[..., tts_model.machine.token_ids.pad] += tts_model.padding_bonus
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
                        audio_tokens[b] = torch.where(mask, audio_codes, audio_tokens[b])

        def _on_text_hook(text_tokens):
            tokens = text_tokens.tolist()
            out_tokens = []
            for b, (token, state) in enumerate(zip(tokens, [self.state])):
                if text_prefixes is not None and text_prefixes[b]:
                    out_token = text_prefixes[b].popleft()
                else:
                    out_token, _ = tts_model.machine.process(self.offset, state, token)
                out_tokens.append(out_token)
            text_tokens[:] = torch.tensor(out_tokens, dtype=torch.long, device=text_tokens.device)

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
        """Initialize streaming mode and wait for initial silence.

        Enables continuous streaming generation, waits for the audio to become
        silent after initial preconditioning, and saves the state for restoration.

        The method:
            1. Enables streaming forever for both LMGen and MIMI models.
            2. Waits until consecutive silent frames match the threshold.
            3. Saves the current state for backup.
        """
        self.lm_gen.streaming_forever(1)
        self.tts_model.mimi.streaming_forever(1)

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
        """Backup the current streaming state for potential restoration.

        Saves copies of all internal model states including:
            - LMGen streaming state (cache, offsets, offset_cpu)
            - LM model streaming state
            - TTS generator state and offset

        This allows reverting to a previous state after processing entries,
        which is essential since the audio quality degrades quickly when the context fills up.
        """
        if self.lm_gen._streaming_state is not None:
            self.lm_gen_state_backup = _backup_attrs(
                self.lm_gen._streaming_state,
                [
                    "cache",
                    "offsets",
                    "offset_cpu",
                ],
            )
        if self.lm_gen.lm_model._streaming_state is not None:
            self.lm_model_state_backup = deepcopy(self.lm_gen.lm_model.get_streaming_state())
        self.state_backup = deepcopy(self.state)
        self.offset_backup = self.offset

    def restore_state(self):
        """Restore the previously backed-up streaming state.

        Restores all saved model states including:
            - LMGen streaming state and its graphed components
            - LM model streaming state (with CUDA graph re-initialization)
            - TTS generator state and offset

        Also re-configures CUDA graphs for the restored model, which is necessary
        because the graphs don't automatically detect state reload.
        """
        self.state = deepcopy(self.state_backup)
        self.offset = self.offset_backup
        _restore_attrs(self.lm_gen._streaming_state, self.lm_gen_state_backup)
        self.lm_gen.lm_model.set_streaming_state(deepcopy(self.lm_model_state_backup))

        # Fix cuda graphs
        # (because they seem to not detect our reload of the lm_model streaming state)
        # Code taken from LMGen._init_streaming_state
        disable = self.lm_gen.lm_model.device.type != "cuda"
        graphed_main = CUDAGraphed(self.lm_gen.lm_model.forward_text, disable=disable)
        if self.lm_gen.lm_model.depformer is not None:
            graphed_depth = CUDAGraphed(self.lm_gen.depformer_step, disable=disable)
        else:
            graphed_depth = None
        self.lm_gen._streaming_state.graphed_main = graphed_main
        self.lm_gen._streaming_state.graphed_depth = graphed_depth

    def reset_state(self):
        """Reset the generator state to initial conditions.

        Clears all accumulated state including:
            - Resets entries list in the state machine
            - Resets LMGen to fresh streaming mode
            - Resets offset counter to zero

        Use this method to start fresh generation, for example after detecting
        a new conversation turn or when switching speakers.
        """
        self.state = self.tts_model.machine.new_state([])
        self.lm_gen.reset_streaming()
        self.offset = 0

    def process(self):
        """Process accumulated entries and generate audio until ready.

        Advances the state machine by processing entries until only one entry
        remains ahead of the stream buffer. This ensures the generator is ready
        to accept new text input without backlog. Should be called after adding
        new entries before appending more text.
        """
        while len(self.state.entries) > self.tts_model.machine.second_stream_ahead:
            self.step()

    def step(self) -> tp.Optional[np.ndarray]:
        """Generate a single audio frame and return it if valid.

        Produces one frame of audio output by:
            1. Creating input tokens for the LM model
            2. Stepping the LM generator forward
            3. Incrementing the offset counter
            4. Decoding frames to audio samples when ready

        Returns:
            Optional[np.ndarray]: The decoded audio sample array if the frame
                contains valid data (not silence/padding), otherwise None.
        """
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
        """Append a single entry to the state machine's entry list.

        Adds an Entry object to the current entries list. This is typically
        used internally when processing multiple entries from a script. The
        caller should usually use append_text() instead for convenience.

        Parameters:
            entry (Entry): The entry object to append to the state.
        """
        self.state.entries.append(entry)

    def append_text(self, msg: str):
        """Append text to generate and process streaming output.

        Converts the input text into entries and adds them to the state machine.
        Processes any accumulated entries until the generator is ready for new
        input. Handles multi-turn conversations by detecting silence between
        turns.

        Parameters:
            msg (str): The text message to convert into TTS entries and generate.
        """
        stripped = msg.strip()
        if stripped:
            entries = prepare_script(self.tts_model, msg.strip(), first_turn=False)
            self.tts_first_turn = False
            for entry in entries:
                self.append_entry(entry)
                self.process()

    def is_audio_silent(self, audio_samples: np.ndarray) -> bool:
        """Check if the audio samples are considered silent.

        Determines whether an audio frame is silent by comparing the maximum
        absolute amplitude against a threshold. Used for detecting pauses
        between conversation turns.

        Parameters:
            audio_samples (np.ndarray): The audio sample array to check.

        Returns:
            bool: True if the audio is considered silent, False otherwise.
        """
        max = np.max(np.abs(audio_samples))
        return max <= self.audio_silent_threshold
