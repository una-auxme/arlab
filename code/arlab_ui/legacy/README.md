# Legacy UI Files

This folder contains legacy UI and debugging files that existed before the current PyQt-based HRI display prototype.

These files are kept for reference only and are not part of the current Zirbi HRI display implementation.

## Files

- main.py
  - legacy Tkinter-based UI demo
  - originally created for the Tag der Informatik 12.03.2026
  - reads button definitions from legacy/config/buttons.json
  - uses image resources from legacy/assets/
  - not used by the current PyQt HRI display prototype

- error_throwing_node.py
  - legacy ROS 2 debug node
  - emits test error log messages
  - not used by the current PyQt HRI display prototype

## Legacy Resources

- assets/
  - legacy Zirbi image and icon files used by the old Tkinter UI

- config/buttons.json
  - legacy button configuration used by the old Tkinter UI

## Current UI Entry Point

The current HRI display prototype is implemented in:

    arlab_ui/arlab_ui/zirbi_display.py

It is started with:

    ros2 run arlab_ui zirbi_display

or:

    ros2 launch arlab_ui zirbi_display.launch.py
