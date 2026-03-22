# Autonomous Robotics Lab - Team Zirbi

- [Autonomous Robotics Lab - Team Zirbi](#autonomous-robotics-lab---team-zirbi)
  - [Documentation](#documentation)
  - [Getting Started](#getting-started)
    - [Setup](#setup)
    - [Simulation startup](#simulation-startup)
  - [Docker](#docker)
  - [Architecture Overview](#architecture-overview)
    - [Bringup \& Integration](#bringup--integration)
    - [Core Infrastructure](#core-infrastructure)
    - [Perception \& Vision](#perception--vision)
    - [Knowledge \& State Management](#knowledge--state-management)
    - [Decision Making \& Behavior](#decision-making--behavior)
    - [Manipulation](#manipulation)
    - [Movement \& Navigation](#movement--navigation)
    - [Speech \& Audio](#speech--audio)
    - [Safety](#safety)
    - [Visualization \& UI](#visualization--ui)
  - [Contributing](#contributing)
  - [Authors and acknowledgment](#authors-and-acknowledgment)

This is the code repository for the home-assistant robot of team Zirbi developed in Autonomous Robotics Lab

This Project aims to develop an fully automatic @Home League robot for the [RoboCup](https://athome.robocup.org/).

Goals of this Robot are:

- be a social and cooperative robot for everyday use
- accomplish easy tasks in home environment
- learn new tasks from visibel demonstration or verbal instructions
- bring joy to users and developers

## Documentation

The [`doc`](./doc/) folder contains the [general project documentation](./doc/00.%20Documentation%20Overview.md).
All code documentation can be found in the respective source code files.

## Getting Started

### Setup

This project needs to be cloned as part of the [arlab_docker](https://git.rz.uni-augsburg.de/imech-m/autonomous_robotics_lab/arlab_docker) project to work.

```bash
# Do not clone arlab manually, just clone arlab_docker
git clone https://git.rz.uni-augsburg.de/imech-m/autonomous_robotics_lab/arlab_docker
cd arlab_docker

# Run the automated setup script (Clones this repo and sets up the environment)
./user_setup.sh
```

For further information about the setup, dependencies and the containers in general, look at the [arlab_docker README](https://git.rz.uni-augsburg.de/imech-m/autonomous_robotics_lab/arlab_docker/-/blob/main/README.md).

### Simulation startup

To start the simulation environment, first ensure the Docker container is running:

```bash
# Navigate to the arlab_docker root
cd /path/to/arlab_docker
# Build and start the Docker container (choose your GPU type)
docker compose -f docker-compose.dev.<your-gpu-type>.yml up --build
```

**Available GPU types / platforms:**

- `cuda` - NVIDIA GPU support (Linux/WSL Lab PCs)
- `intel` - Intel GPU (Vulkan)
- `rocm` - AMD GPU (ROCm)
- `wsl` - Windows WSL2 with WSLG GPU support

Once the container is running, enter it and **launch the simulation as described in the [arlab_bringup documentation](./code/arlab_bringup/README.md)**.

```bash
# Find container name
docker ps
# Enter the container (Or just attach Vs Code with the Container Tools extension)
docker exec -it <container-name> bash

# Start basic simulation
ros2 launch manipulator_description manipulator.full.sim.launch.py
```

## Docker

The [arlab_docker](https://git.rz.uni-augsburg.de/imech-m/autonomous_robotics_lab/arlab_docker) repository contains the docker configuration to execute this project.

## Architecture Overview

The following tables provide an overview of the ROS 2 packages in the [`code`](./code/) directory, organized by functional category.

### Bringup & Integration

| Package | Description | Documentation |
| ----------- | --------------- | ---------------- |
| [`arlab_bringup`](./code/arlab_bringup/) | Main launch package for the robot system | [Read more](./code/arlab_bringup/README.md) |

### Core Infrastructure

| Package | Description |
| ----------- | --------------- |
| [`arlab_asyncio_executor`](./code/arlab_asyncio_executor/) | Async executor for running Python nodes in ROS 2 |
| [`arlab_common`](./code/arlab_common/) | Common utilities and shared code |
| [`arlab_common_interfaces`](./code/arlab_common_interfaces/) | Common ROS 2 action and message interfaces |
| [`arlab_template_interfaces`](./code/arlab_template_interfaces/) | Template interfaces for custom message types |
| [`arlab_templates`](./code/arlab_templates/) | ROS 2 package templates for new components |

### Perception & Vision

| Package | Description | Documentation |
| ----------- | --------------- | ---------------- |
| [`arlab_computer_vision`](./code/arlab_computer_vision/) | YOLO-based object detection/segmentation integrated with the knowledge base | [Read more](./code/arlab_computer_vision/README.md) |
| [`arlab_perception`](./code/arlab_perception/) | Perception helpers (lidar relay, camera interface) | [Read more](./code/arlab_perception/README.md) |

### Knowledge & State Management

| Package | Description | Documentation |
| ----------- | --------------- | ---------------- |
| [`arlab_knowledge`](./code/arlab_knowledge/) | Database for handling robot surroundings, events and maps | [Read more](./code/arlab_knowledge/README.md) |
| [`arlab_knowledge_interfaces`](./code/arlab_knowledge_interfaces/) | Knowledge base ROS 2 message and service interfaces | *read above* |

### Decision Making & Behavior

| Package | Description | Documentation |
| ----------- | --------------- | ---------------- |
| [`arlab_decision_making`](./code/arlab_decision_making/) | Global decision maker using py_trees-based behavior trees | [Read more](./code/arlab_decision_making/README.md) |
| [`py_trees_error_selector`](./code/py_trees_error_selector/) | Error selector behavior tree node for robust error handling | *read above* |

### Manipulation

| Package | Description | Documentation |
| ----------- | --------------- | ---------------- |
| [`arlab_manipulation`](./code/arlab_manipulation/) | Orchestrates manipulation commands (move, pick, place, home, open, close) | [Read more](./code/arlab_manipulation/README.md) |
| [`arlab_manipulation_cpp`](./code/arlab_manipulation_cpp/) | C++ interface for manipulation execution | *read above* |
| [`manipulator_description`](./code/manipulator_description/) | URDF/xacro robot model, launch files, controller configuration | [Read more](./code/manipulator_description/README.md) |
| [`manipulator_robot_driver`](./code/manipulator_robot_driver/) | Actual driver for UR robots (modified for ARLab) | [Read more](./code/manipulator_robot_driver/README.md) |
| [`manipulator_ur_moveit_config`](./code/manipulator_ur_moveit_config/) | Customized MoveIt 2 configuration for the manipulator | [Read more](./code/manipulator_ur_moveit_config/README.md) |

### Movement & Navigation

| Package | Description |
| ----------- | --------------- |
| [`arlab_movement`](./code/arlab_movement/) | Movement behaviors and navigation |

### Speech & Audio

| Package | Description | Documentation |
| ----------- | --------------- | ---------------- |
| [`arlab_speech_controller`](./code/arlab_speech_controller/) | Speech synthesis and audio processing (MOSHI TTS) | [Read more](./code/arlab_speech_controller/README.md) |

### Safety

| Package | Description |
| ----------- | --------------- |
| [`arlab_safety`](./code/arlab_safety/) | Safety monitoring and emergency handling |

### Visualization & UI

| Package | Description | Documentation |
| ----------- | --------------- | ---------------- |
| [`arlab_ui`](./code/arlab_ui/) | Robot UI and visualization components | [Read more](./code/arlab_ui/README.md) |
| [`arlab_vizbox`](./code/arlab_vizbox/) | Visualization box for robot state display *(deprecated)* | |

## Contributing

Everybody can contribute. Get in touch with us for questions and open tasks. We are happy to have you on board.
We follow the [![Contributor Covenant](https://img.shields.io/badge/Contributor%20Covenant-2.1-4baaaa.svg)](./CODE_OF_CONDUCT.md)

For easy start we created a [CONTRIBUTING](./CONTRIBUTING.md) Guideline.

## Authors and acknowledgment

- Lukas Asam
- Daniel Gabler
- Aleksander Michalak
- Sofia Öttl
- Jonas Platzer
- Leonie Schmidt
- Peter Viechter
- Meruna Yugarajah
- Nils Mandischer
- Robert Jeutter
