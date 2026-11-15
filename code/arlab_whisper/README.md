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

## Usage

The node can be started in two different ways:

The node can be started directly using `ros2 run`. In this case, the default parameter values defined in the script are used.

```bash
ros2 run arlab_whisper whisper_node
```

Alternatively, the node can be started using a ROS 2 launch file using `ros2 launch`. The launch file loads the parameters from a YAML configuration file and passes them to the node.

```bash
ros2 launch arlab_whisper whisper.launch.py
```
