# ARLab UI

This package contains UI components for the Autonomous Robotics Lab robot Zirbi.

The current focus of this package is the HRI display prototype for the robot-facing user interface. The prototype is implemented with PyQt6 and integrated as a ROS 2 Python package.

## HRI Display Prototype

The HRI display prototype supports the interaction flow for the RoboCup@Home tasks Pick and Place and Laundry. It is designed as a first ROS-integrated display layer for showing task state, speech output, interaction options and camera feedback.

The prototype currently provides:

- a home screen for selecting task modes,
- challenge-specific control screens,
- a runtime view for jury-facing task feedback,
- a developer view for debugging runtime state and ROS integration,
- live display of speech output received from ROS,
- prototype feedback from UI buttons back into ROS,
- optional live camera display from a ROS image topic,
- fallback to a local placeholder image if no camera frames are received.

The implementation is still a prototype. It is intended to support the design review and further HRI integration work, not to define the final HRI architecture.

## Package Structure

Relevant files:

    arlab_ui/
    |-- arlab_ui/
    |   |-- __init__.py
    |   |-- zirbi_display.py
    |   `-- camera_placeholder.png
    |-- launch/
    |   `-- zirbi_display.launch.py
    |-- config/
    |   `-- buttons.json
    |-- resource/
    |   `-- arlab_ui
    |-- package.xml
    |-- requirements.txt
    |-- setup.cfg
    `-- setup.py

The main display implementation is located in:

    arlab_ui/arlab_ui/zirbi_display.py

## Run

After building and sourcing the workspace:

    devbuild
    devsource

Start the display directly with:

    ros2 run arlab_ui zirbi_display

or with the launch file:

    ros2 launch arlab_ui zirbi_display.launch.py

## ROS Integration

### Speech Output

The display subscribes to:

    /tts_output

Expected message type:

    std_msgs/msg/String

Incoming messages are shown in the Speech Output card of the runtime view.

Example test command:

    ros2 topic pub --once /tts_output std_msgs/msg/String "{data: 'This message comes from ROS and is shown in the PyQt display.'}"

This topic is currently used to connect the display prototype to the existing speech output flow.

### UI Action Feedback

The display publishes basic UI button actions to:

    /ui_action

Message type:

    std_msgs/msg/String

Current prototype actions:

    confirm
    repeat
    cancel

Example test command:

    ros2 topic echo /ui_action std_msgs/msg/String

The /ui_action topic is a temporary prototype interface. It is useful for testing button feedback from the display, but it is not intended to be the final structured HRI feedback interface.

The final feedback interface should be aligned with the Decision Making and HRI Output structure.

### Camera Image Input

The runtime camera panel subscribes to:

    /camera_gripper/color/image_raw

Expected message type:

    sensor_msgs/msg/Image

Incoming image messages are converted with:

    cv_bridge -> QImage -> QPixmap

and rendered in the camera area of the runtime view.

If no image messages are received, the display falls back to the local placeholder image:

    camera_placeholder.png

The camera integration was tested in the development container with a laptop webcam passed through from the host system. The webcam frames were read with OpenCV, published as sensor_msgs/msg/Image on /camera_gripper/color/image_raw, and displayed live in the PyQt GUI.

Final validation with the real robot camera during task execution is still open.

## Dependencies

The PyQt-based display requires:

    PyQt6

This dependency is listed in:

    arlab_ui/requirements.txt

ROS-related runtime dependencies are declared in:

    arlab_ui/package.xml

Current relevant ROS dependencies:

    rclpy
    std_msgs
    sensor_msgs
    cv_bridge

## Current Implementation Status

Currently implemented:

- PyQt6 display prototype,
- ROS 2 Python package structure,
- start via ros2 run,
- start via ros2 launch,
- /tts_output subscriber for speech output,
- /ui_action publisher for prototype button feedback,
- camera image subscriber for /camera_gripper/color/image_raw,
- placeholder fallback if no camera frames are received,
- runtime view with jury-facing information,
- developer view with runtime state and ROS integration information.

Still open:

- final HRI Output Interface from Decision Making,
- structured UI action feedback interface beyond the temporary /ui_action topic,
- final validation with the real robot camera during task execution,
- final launch and configuration structure,
- review and cleanup before merge.

## Notes for Design Review

This prototype demonstrates that the display is no longer a standalone mockup only. It is packaged as a ROS 2 Python package and can exchange basic runtime information with ROS.

The current prototype separates:

- challenge selection,
- task control,
- runtime feedback,
- developer/debug information.

The runtime view is driven by an internal runtime state model. This model represents the kind of information that a future HRI Output Interface should provide in a structured way, for example:

- current challenge,
- current subtask,
- display text,
- speech text,
- next expected action,
- progress state,
- available user interaction buttons,
- camera feedback.

## Development Notes

This HRI display prototype was implemented by Jessica Herrmann as part of the HRI/UI work for the Autonomous Robotics Lab project.

ChatGPT 5.5 was used as a development assistant during the prototyping process, mainly for code structuring, ROS package setup, debugging and documentation support.

The code was tested manually in the ROS development container. Tested integrations include:

- starting the display through ROS 2,
- receiving /tts_output messages,
- publishing /ui_action messages,
- displaying a live camera image published on /camera_gripper/color/image_raw.
