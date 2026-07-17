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
- synchronized display of speech output received from ROS,
- prototype feedback from UI buttons back into ROS.

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

The prototype also publishes basic UI button actions to:

    /ui_action

Message type:

    std_msgs/msg/String

Current prototype actions:

- confirm
- repeat
- cancel

Example test command:

    ros2 topic echo /ui_action std_msgs/msg/String

This is a prototype feedback topic and not the final HRI interface. The final interface should be aligned with the Decision Making structure.

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
- optional /tts_output subscription,
- prototype /ui_action publisher for Confirm, Repeat and Cancel.

Still open:

- final HRI Output Interface from Decision Making,
- structured UI action feedback interface,
- live camera topic integration,
- final launch and configuration structure,
- review and cleanup before merge.

## Development Notes

This HRI display prototype was implemented by Jessica Herrmann as part of the HRI/UI work for the Autonomous Robotics Lab project.

ChatGPT 5.5 was used as a development assistant during the prototyping process, mainly for code structuring, ROS package setup, debugging and documentation support. The code was tested manually in the ROS development container.
