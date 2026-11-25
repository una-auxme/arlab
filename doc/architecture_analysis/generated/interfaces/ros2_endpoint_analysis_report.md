# ROS 2 Endpoint Analysis

## Summary

- Unmatched declarations: 94
- Launch-configured candidates: 4
- Namespace candidates: 1
- Unresolved declarations: 2
- No counterpart detected: 87

## Interpretation

Results are based on static Python/C++ analysis.
No counterpart detected does not prove an endpoint is unused.
External nodes, runtime namespaces, remapping and dynamic configuration may affect actual communication.

## Endpoint details

| Package | Role | Interface | Status |
|---|---|---|---|
| manipulator_robot_driver | client | `dashboard_client/play` | launch_configured_candidate |
| manipulator_robot_driver | client | `dashboard_client/restart_safety` | launch_configured_candidate |
| manipulator_robot_driver | server | `~/play` | launch_configured_candidate |
| manipulator_robot_driver | server | `~/restart_safety` | launch_configured_candidate |
| arlab_speech_controller | subscriber | `tts_output` | namespace_candidate |
| arlab_decision_making | client | `/mia_hand/grasps/cylindrical/action` | no_counterpart_detected |
| arlab_manipulation_cpp | client | `/mia_hand/grasps/cylindrical/action` | no_counterpart_detected |
| arlab_manipulation_cpp | client | `/mia_hand/grasps/lateral/action` | no_counterpart_detected |
| arlab_manipulation_cpp | client | `/mia_hand/grasps/pinch/action` | no_counterpart_detected |
| arlab_manipulation_cpp | client | `/mia_hand/grasps/point_down/action` | no_counterpart_detected |
| arlab_manipulation_cpp | client | `/mia_hand/grasps/point_up/action` | no_counterpart_detected |
| arlab_manipulation_cpp | client | `/mia_hand/grasps/spherical/action` | no_counterpart_detected |
| arlab_manipulation_cpp | client | `/mia_hand/grasps/tridigital/action` | no_counterpart_detected |
| arlab_manipulation_cpp | client | `move_action` | no_counterpart_detected |
| arlab_movement | server | `/movement/action` | no_counterpart_detected |
| manipulator_robot_driver | client | `/tool_contact_controller/detect_tool_contact` | no_counterpart_detected |
| manipulator_robot_driver | client | `/trajectory_until_node/execute` | no_counterpart_detected |
| manipulator_robot_driver | client | `scaled_joint_trajectory_controller/follow_joint_trajectory` | no_counterpart_detected |
| manipulator_robot_driver | client | `scaled_joint_trajectory_controller/follow_joint_trajectory` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/execute` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/set_mode` | no_counterpart_detected |
| mia_hand_simulation_controller | client | `/move_action` | no_counterpart_detected |
| mia_hand_simulation_controller | client | `/move_action` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/add_status_event` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/cupboard_get_shelf` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/furniture_get_pickable` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/furniture_update_pickable` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/get_description` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/get_map` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/get_pose` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/get_status_events` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/pickable_get_furniture` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/shelf_get_cupboard` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/upd_entity` | no_counterpart_detected |
| arlab_knowledge | server | `/arlab/knowledge/upd_pose` | no_counterpart_detected |
| arlab_manipulation_cpp | client | `/mia_hand/data_streams/fingers/forces/switch` | no_counterpart_detected |
| arlab_movement | server | `/arlab/entity_placer/place` | no_counterpart_detected |
| arlab_templates | server | `/add_three_ints` | no_counterpart_detected |
| arlab_templates | client | `add_three_ints` | no_counterpart_detected |
| manipulator_robot_driver | client | `controller_manager/list_controllers` | no_counterpart_detected |
| manipulator_robot_driver | client | `controller_manager/switch_controller` | no_counterpart_detected |
| manipulator_robot_driver | client | `io_and_status_controller/resend_robot_program` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/add_to_log` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/brake_release` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/clear_operational_mode` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/close_popup` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/close_safety_popup` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/connect` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/get_loaded_program` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/get_robot_mode` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/get_safety_mode` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/is_in_remote_control` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/load_installation` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/load_program` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/pause` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/popup` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/power_off` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/power_on` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/program_running` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/program_saved` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/program_state` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/quit` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/raw_request` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/resume` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/shutdown` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/stop` | no_counterpart_detected |
| manipulator_robot_driver | server | `~/unlock_protective_stop` | no_counterpart_detected |
| arlab_computer_vision | publisher | `/vision/debug_pc` | no_counterpart_detected |
| arlab_computer_vision | publisher | `/vision/segmented_image` | no_counterpart_detected |
| arlab_computer_vision | subscriber | `camera_color_image` | no_counterpart_detected |
| arlab_computer_vision | subscriber | `camera_info` | no_counterpart_detected |
| arlab_knowledge | publisher | `/arlab/knowledge/visualization` | no_counterpart_detected |
| arlab_manipulation | subscriber | `/mia_hand/data_streams/fingers/forces/data` | no_counterpart_detected |
| arlab_manipulation | subscriber | `/monitored_planning_scene` | no_counterpart_detected |
| arlab_manipulation | publisher | `/object_placed` | no_counterpart_detected |
| arlab_movement | subscriber | `/arlab/entity_pose` | no_counterpart_detected |
| arlab_movement | publisher | `/arlab/movement/annotating` | no_counterpart_detected |
| arlab_movement | subscriber | `/map` | no_counterpart_detected |
| arlab_movement | subscriber | `/odom` | no_counterpart_detected |
| arlab_perception | publisher | `/relay_scan` | no_counterpart_detected |
| arlab_perception | subscriber | `/scan` | no_counterpart_detected |
| arlab_templates | subscriber | `/local_module_heartbeat` | no_counterpart_detected |
| arlab_ui | subscriber | `/camera_gripper/color/image_raw` | no_counterpart_detected |
| arlab_ui | publisher | `/ui_action` | no_counterpart_detected |
| arlab_whisper | subscriber | `/whisper/enabled` | no_counterpart_detected |
| arlab_whisper | publisher | `/whisper/transcript` | no_counterpart_detected |
| manipulator_robot_driver | publisher | `direction` | no_counterpart_detected |
| manipulator_robot_driver | subscriber | `io_and_status_controller/robot_mode` | no_counterpart_detected |
| manipulator_robot_driver | subscriber | `io_and_status_controller/robot_program_running` | no_counterpart_detected |
| manipulator_robot_driver | subscriber | `io_and_status_controller/robot_program_running` | no_counterpart_detected |
| manipulator_robot_driver | subscriber | `io_and_status_controller/safety_mode` | no_counterpart_detected |
| manipulator_robot_driver | subscriber | `~/script_command` | no_counterpart_detected |
| manipulator_robot_driver | client | `action_name` | unresolved |
| manipulator_robot_driver | client | `srv_name` | unresolved |
