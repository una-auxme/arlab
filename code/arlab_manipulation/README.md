# arlab_manipulation

ROS2 Python package for orchestrating commands from desicion maker for manipulator processing with use of knowledgebase and gripping parameter service. Some functions are still experimental.
This project was built specifically for the `ARLab` at the University of Augsburg and developed in the context of the `Zirbi` robot.

### Quickstart
Start the manipulation stack (gripping parameter service, orchestrator, C++ subscriber):
```bash
ros2 launch arlab_manipulation launch.py
