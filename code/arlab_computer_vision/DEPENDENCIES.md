# Dependencies Overview

## System Packages (ROS2)

- `ros-jazzy-desktop` (oder minimal: `ros-jazzy-ros-base`)
- `ros-jazzy-cv-bridge` - OpenCV bridge
- `ros-jazzy-tf2-ros` - TF2 transformations
- `ros-jazzy-vision-msgs` - Vision message types
- `ros-jazzy-sensor-msgs` - Sensor message types
- `python3-colcon-common-extensions` - Build tools

## Python Packages

### Core (requirements.txt)

- `ultralytics` - YOLO models
- `numpy` - Numerical operations
- `opencv-python` - Image processing
- `scikit-learn` - Clustering (DBSCAN)

### PyTorch (choose one)

- **CUDA** (`requirements.cuda.txt`): PyTorch with CUDA support
- **CPU** (`requirements.cpu.txt`): PyTorch CPU-only
- **ROCm** (`requirements.rocm.txt`): PyTorch with AMD GPU support

## ROS2 Workspace Dependencies

- `arlab_common_interfaces` - PredictSnapshot action interface
- `arlab_knowledge_interfaces` - Entity messages
- `arlab_asyncio_executor` - Async executor

## Optional

- CUDA Toolkit (für GPU-Inferenz)
- NVIDIA Jetson SDK (für Jetson)
