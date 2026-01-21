# Installation Guide

Kompakte Installationsanleitung für `arlab_computer_vision`.

## Voraussetzungen

- ROS2 Jazzy installiert
- Python 3.12+
- CUDA (für GPU-Inferenz) oder CPU-only

## 1. System Dependencies

```bash
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-vision-msgs \
    ros-jazzy-sensor-msgs
```

## 2. Python Dependencies

### GPU (CUDA) - Desktop/Jetson
```bash
pip3 install -r requirements.cuda.txt
pip3 install -r requirements.txt
```

**Jetson-spezifisch:** PyTorch für Jetson installieren:
```bash
# Option 1: NVIDIA Jetson PyTorch (empfohlen)
# Siehe: https://forums.developer.nvidia.com/t/pytorch-for-jetson/
# Download von: https://developer.nvidia.com/embedded/jetpack

# Option 2: PyPI mit CUDA 11.8 (falls verfügbar)
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

### CPU-only
```bash
pip3 install -r requirements.cpu.txt
pip3 install -r requirements.txt
```

### ROCm (AMD GPU)
```bash
pip3 install -r requirements.rocm.txt
pip3 install -r requirements.txt
```

## 3. Workspace Setup

```bash
# In deinem ROS2 Workspace
cd /path/to/arlab

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## 4. YOLO Model

Das YOLO-Modell (`yolo11n-seg.pt`) wird automatisch beim ersten Start heruntergeladen, falls nicht vorhanden.

**Manuell:** Modell in `yolo_weights/` ablegen.

## 5. Testen

```bash
# Node starten
ros2 launch arlab_computer_vision object_detection_launch.py

# In anderem Terminal: Test-Client
source install/setup.bash
python3 code/arlab_computer_vision/scripts/test_predict_snapshot_action_client.py
```

## Troubleshooting

- **CUDA nicht gefunden:** Prüfe `nvidia-smi` und CUDA-Installation
- **Import-Fehler:** Stelle sicher, dass Workspace gebaut wurde (`colcon build`)
- **Action nicht gefunden:** `arlab_common_interfaces` muss gebaut sein
- **Memory-Fehler (Jetson):** Reduziere `point_cloud_max_points` oder `max_image_width` Parameter
