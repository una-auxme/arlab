# Lidar Sensor Merger

Merges the output of topic /scan_lidar_front and scan_lidar_back onto /scan. works by sampling points of the first topic and mapping them to a new "simulated" lidar sensor. Necessary because SLAM Toolbox does not work with multiple sensors.

**Maintainer:** Julian Schropp &lt;<julian-schropp@web.de>&gt;
**License:** Apache2.0  

## LAUNCH

ros2 run lidar_sensor_merger lidar_sensor_merger
