# arlab_manipulation

ROS2 Python package for orchestrating pick-and-place manipulation with octomap support and gripping parameter computation.  
Built for the **ARLab** at the University of Augsburg for robotic manipulation research. **Experimental / not fully tested.**

### Local launch
Start the manipulation stack (gripping parameter service, orchestrator, C++ subscriber):
```bash
ros2 launch arlab_manipulation launch.py
