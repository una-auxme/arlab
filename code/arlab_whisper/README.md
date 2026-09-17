# ARLAB whisper

This package provides a speech-to-text pipeline including a wakeword-detection-based faster-whisper model to generate a transcript from an audio input stream. 

## Package structure

```txt
arlab_whisper/
├── arlab_whisper/
│   └── whisper_node.py       # STT whisper node
├── audio_test/               # Python test files to record audio data
├── config/                   # Config for the whisper settings
├── launch/                   # Launch files
└── requirements.txt          # Python dependencies
```

## Key features

 - Generates a transcript of audio input via a faster-whisper model
 - Only activated via a small faster-whisper model, which is used as a wakeword detection
 - Config files are provided

