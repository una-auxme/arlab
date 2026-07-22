#!/usr/bin/env python3
import os
import wave
from datetime import datetime

import numpy as np
import sounddevice as sd


SAMPLE_RATE = 16000
CHANNELS = 1
DTYPE = np.int16
OUTPUT_FOLDER = "recordings"
RECORD_SECONDS = 5
BLOCKSIZE = int(SAMPLE_RATE * RECORD_SECONDS)


def save_wav(filename, audio):
    """saves audio samples to a WAV file

    Args:
        filename (String): path to the stored file location
        audio (numpy.ndarray): Audio samples as a array of the
            dtype datatype
    """
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)  # int16 = 2 bytes
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(audio.tobytes())


def main():
    """continuously record audio and save stores it to a wav file

    creates the output subdirectory, opens an audio stream, records audio and saves
    each block to the path, records until interrupted
    """
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)
    # print(sd.query_devices())
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
                    f"testaudio.wav",
                )
                save_wav(filename, audio)
                print(f"Saved: {filename}")

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
