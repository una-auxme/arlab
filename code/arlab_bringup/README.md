# ARLAB bringup

This package provides launch configurations for the distributed system of the robot.

## Package structure

```txt
arlab_bringup/
├── arlab_bringup/
│   ├── __init__.py              # Package initialization
└── launch/
    ├── arlab-nuc.launch.py      # NUC hardware launch file
    ├── jetson-arlab1.launch.py  # Jetson ARLAB1 launch file
    ├── jetson-arlab2.launch.py  # Jetson ARLAB2 launch file
    └── jetson-arlab3.launch.py  # Jetson ARLAB3 launch file
```

## Key features

- Hardware-specific launch configurations
- Support for NUC and Jetson platforms
- Easy robot system startup with pre-configured launch files
