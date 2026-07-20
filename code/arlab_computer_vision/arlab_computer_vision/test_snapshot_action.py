"""Manual test script for the ObjectDetection `/vision/snapshot` action.

Sends a single snapshot goal that exercises both the pinned general model
(open_img.pt, MODEL_GENERAL_OBJECTS_AND_PEOPLE - always runs) and the
lazy-loaded person model (person.pt, MODEL_DISHWASHER - requested via
extra_models). This forces the ModelRegistry to load person.pt on demand
alongside open_img.pt so both weights run against the same live camera frame.

Prerequisites (run these first, in separate terminals):
    ros2 launch realsense2_camera rs_launch.py enable_pointcloud:=true \
        camera_name:=camera_gripper
    ros2 launch arlab_computer_vision object_detection_launch.py

Usage:
    python3 test_snapshot_action.py [--clear-database] [--mask-hand] [--out FILE]

The node does not require the knowledge base to be up: KB calls are skipped
with a warning if the KB services are unavailable, so this script also works
standalone against just the camera + object_detection node.

Maintainers:
    Simeon Wagner <simeon.wagner@uni-a.de>
    Lars Britz <lars.britz@uni-a.de>
"""

import argparse
import sys

import rclpy
from arlab_common_interfaces.action import VisionSnapshotAction
from arlab_common_interfaces.msg import VisionSnapshotCommand, VisionSnapshotResponse
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image

_RESULT_NAMES = {
    VisionSnapshotResponse.SUCCESS: "SUCCESS",
    VisionSnapshotResponse.ERROR_UNKNOWN: "ERROR_UNKNOWN",
    VisionSnapshotResponse.ERROR_NO_IMAGE_DATA: "ERROR_NO_IMAGE_DATA",
}


class SnapshotActionTester(Node):
    """Requests one vision snapshot and grabs the resulting visualization frame."""

    def __init__(self, clear_database: bool, mask_hand: bool, out_path: str) -> None:
        super().__init__("test_snapshot_action_client")
        self._out_path = out_path
        self._bridge = CvBridge()
        self._got_frame = False

        self._client = ActionClient(self, VisionSnapshotAction, "/vision/snapshot")

        # Grab whatever frame the node publishes for this snapshot so we can
        # visually confirm both models produced detections.
        self._image_sub = self.create_subscription(
            Image,
            "/vision/segmented_image",
            self._on_segmented_image,
            qos_profile=1,
        )

        command = VisionSnapshotCommand()
        command.clear_database = clear_database
        command.mask_hand = mask_hand
        # open_img.pt (MODEL_GENERAL_OBJECTS_AND_PEOPLE) is pinned and always runs.
        # Requesting MODEL_DISHWASHER forces the registry to lazy-load person.pt too.
        command.extra_models = [VisionSnapshotCommand.MODEL_DISHWASHER, VisionSnapshotCommand.MODEL_GENERAL_OBJECTS_AND_PEOPLE]

        goal = VisionSnapshotAction.Goal()
        goal.command = command

        self.get_logger().info("Waiting for /vision/snapshot action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available. Is object_detection running?")
            sys.exit(1)

        self.get_logger().info(
            f"Sending snapshot goal (extra_models=[MODEL_DISHWASHER=person.pt], clear_database={clear_database}, mask_hand={mask_hand})"
        )
        send_goal_future = self._client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            sys.exit(1)
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        response = future.result().result.response
        result_name = _RESULT_NAMES.get(response.result, str(response.result))
        self.get_logger().info(f"Snapshot result: {result_name}")
        if response.error_msg:
            self.get_logger().warn(f"error_msg: {response.error_msg}")

    def _on_segmented_image(self, msg: Image) -> None:
        if self._got_frame:
            return
        self._got_frame = True
        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        import cv2

        cv2.imwrite(self._out_path, cv_image)
        self.get_logger().info(f"Saved annotated frame to {self._out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clear-database", action="store_true", help="Clear KB entities before the snapshot")
    parser.add_argument("--mask-hand", action="store_true", help="Mask out the lower part of the frame")
    parser.add_argument("--out", default="/tmp/snapshot_result.png", help="Where to save the annotated frame")
    args = parser.parse_args()

    rclpy.init()
    node = SnapshotActionTester(
        clear_database=args.clear_database,
        mask_hand=args.mask_hand,
        out_path=args.out,
    )
    try:
        # Give the result callback and image subscription a few seconds to fire.
        rclpy.spin_once(node, timeout_sec=0.0)
        end_time = node.get_clock().now().nanoseconds + int(15e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
