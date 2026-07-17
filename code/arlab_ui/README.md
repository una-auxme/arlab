# ARLab UI

This package contains UI components for the ARLab robot Zirbi.

## HRI Display Prototype

The current HRI display prototype is implemented with PyQt6 and can be started as a ROS 2 Python package.

The prototype focuses on the robot-facing display for the RoboCup@Home tasks Pick and Place and Laundry. It provides:

- a home screen for selecting task modes,
- challenge-specific control screens,
- a runtime view for jury-facing output,
- a developer view for debugging runtime state and ROS integration,
- a placeholder camera view,
- synchronized display of speech output received from ROS.

## Run

After building and sourcing the workspace:

    devbuild
    devsource

Start the display with:

    ros2 run arlab_ui zirbi_display

or with:

    ros2 launch arlab_ui zirbi_display.launch.py

## ROS Integration

The prototype currently subscribes to:

    /tts_output

Message type:

    std_msgs/msg/String

Incoming messages are shown in the Speech Output card of the runtime view.

Example test command:

    ros2 topic pub --once /tts_output std_msgs/msg/String "{data: 'This message comes from ROS and is shown in the PyQt display.'}"

## Dependencies

The PyQt-based display requires:

    PyQt6

This dependency is listed in:

    arlab_ui/requirements.txt

## Current Limitations

This is still a prototype and not the final HRI output architecture.

Currently implemented:

- standalone PyQt6 display prototype,
- ROS package structure,
- start via ros2 run,
- start via ros2 launch,
- optional /tts_output subscription.

Still open:

- final HRI Output Interface from Decision Making,
- UI action feedback topic for confirm, repeat and cancel,
- live camera topic integration,
- final launch and configuration structure,
- review and cleanup before merge.
