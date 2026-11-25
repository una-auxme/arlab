#!/usr/bin/env python3

"""
Audio Test

Maintainer: tobias.neher@uni-a.de

This module provides a simple audio recording script for testing microphone
input using the sounddevice library.

The script continuously captures audio from the microphone in configurable
blocks and saves each block as a WAV file.

Each new audio block overwrites the previously saved file, so only the most
recent recording is retained.

The recording continues until the script is interrupted (Ctrl+C).
The output directory is created automatically if it does not already exist.
"""

import os
import wave

import numpy as np
import sounddevice as sd


SAMPLE_RATE = 16000
CHANNELS = 1
DTYPE = np.int16
OUTPUT_FOLDER = "recordings"
RECORD_SECONDS = 5
BLOCKSIZE = int(SAMPLE_RATE * RECORD_SECONDS)


def save_wav(filename, audio):
    """
    Saves the provided audio samples to a WAV file.

    args:
        filename (String): Path to the stored file location.
        audio (numpy.ndarray): Audio samples as an array.
    """
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)  # int16 = 2 bytes
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(audio.tobytes())


def main():
    """Continuously records audio and saves it as a WAV file.

    Creates the output directory and opens an audio input stream.
    Reads audio blocks of a configurable duration and saves each block
    to the same WAV file, overwriting the previous recording.
    Continues recording until interrupted.
    """
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)
    try:
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype=DTYPE,
            blocksize=BLOCKSIZE,
        ) as stream:
            while True:
                audio, overflowed = stream.read(BLOCKSIZE)

                filename = os.path.join(
                    OUTPUT_FOLDER,
                    "testaudio.wav",
                )
                save_wav(filename, audio)
                print(f"Saved: {filename}")

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
