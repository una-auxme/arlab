"""Manual test client for the ObjectDetection `/vision/snapshot` action.

Sends one snapshot goal to a running ObjectDetection node, logs the result code
it reports back, and saves the first annotated frame the node publishes on
`/vision/segmented_image` so the detections can be checked visually.

The goal's `extra_models` list selects which models to run in addition to the
ones config/models.yaml marks as pinned. This
tests both loading paths of the ModelRegistry against a single camera
frame. Edit that list in `SnapshotActionTester.__init__` to change what is
used for inference, All available MODEL_* constants are defined in
VisionSnapshotCommand.msg.

`--clear-database` and `--mask-hand` are passed straight through into the
VisionSnapshotCommand. After sending the goal the script spins for a fixed 15
seconds to give the result and image callbacks time to arrive, then shuts down.
It exits with status 1 if the action server does not appear within 10 seconds
or if the goal is rejected.

This is a manual script, not an automated test: it needs a real camera and a
running node, so it is neither collected by pytest nor installed as an entry
point.

Prerequisites (run these first, in separate terminals):
    ros2 launch realsense2_camera rs_launch.py enable_pointcloud:=true \
        camera_name:=camera_gripper
    ros2 launch arlab_computer_vision object_detection_launch.py

Usage:
    python3 test_snapshot_action.py [--clear-database] [--mask-hand] [--out FILE]

The object deteciotn node does not require the knowledge base to be up: KB calls are skipped
with a warning if the KB services are unavailable, so starting the KB is not necessary for the scripts successful execution


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

# VisionSnapshotResponse result codes are plain ints on the wire; this maps them
# back to their constant names so the logged outcome is readable.
_RESULT_NAMES = {
    VisionSnapshotResponse.SUCCESS: "SUCCESS",
    VisionSnapshotResponse.ERROR_UNKNOWN: "ERROR_UNKNOWN",
    VisionSnapshotResponse.ERROR_NO_IMAGE_DATA: "ERROR_NO_IMAGE_DATA",
}


class SnapshotActionTester(Node):
    """Requests one vision snapshot and grabs the resulting visualization frame."""

    def __init__(self, clear_database: bool, mask_hand: bool, out_path: str) -> None:
        """Build the snapshot goal and send it once the action server is up.

        The goal is sent from the constructor rather than from a separate call,
        so constructing the node is the whole test; `main` only has to spin it
        long enough for the callbacks to arrive.

        Args:
            clear_database: Ask the node to delete the stored pickables before
                inserting this snapshot's detections.
            mask_hand: Ask the node to blank the lower part of the frame, which
                suppresses detections of the robot's own gripper.
            out_path: File the annotated frame is written to.

        Side Effects:
            Terminates the process with status 1 if the action server does not
            appear within 10 seconds.
        """
        super().__init__("test_snapshot_action_client")
        self._out_path = out_path
        self._bridge = CvBridge()
        self._got_frame = False

        self._client = ActionClient(self, VisionSnapshotAction, "/vision/snapshot")

        # Grab whatever frame the node publishes for this snapshot so the
        # detections can be confirmed visually.
        self._image_sub = self.create_subscription(
            Image,
            "/vision/segmented_image",
            self._on_segmented_image,
            qos_profile=1,
        )

        command = VisionSnapshotCommand()
        command.clear_database = clear_database
        command.mask_hand = mask_hand

        # Insert the models you want to run in this array. If models are pinned they'll always run
        # - MODEL_DISHWASHER
        # - MODEL_PEOPLE
        # - MODEL_GENERAL_OBJECTS
        command.extra_models = [VisionSnapshotCommand.MODEL_GENERAL_OBJECTS]

        goal = VisionSnapshotAction.Goal()
        goal.command = command

        self.get_logger().info("Waiting for /vision/snapshot action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available. Is object_detection running?")
            sys.exit(1)

        self.get_logger().info(
            f"Sending snapshot goal (extra_models={command.extra_models}, clear_database={clear_database}, mask_hand={mask_hand})"
        )
        send_goal_future = self._client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        """Chain the result request onto an accepted goal.

        Args:
            future: Completed future from `send_goal_async`, holding the goal
                handle.

        Side Effects:
            Terminates the process with status 1 if the goal was rejected.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            sys.exit(1)
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        """Log the outcome the node reported for the snapshot.

        The node succeeds the goal even when processing failed, so the actual
        outcome has to be read from the response code rather than from the
        goal status.

        Args:
            future: Completed future from `get_result_async`.
        """
        response = future.result().result.response
        result_name = _RESULT_NAMES.get(response.result, str(response.result))
        self.get_logger().info(f"Snapshot result: {result_name}")
        if response.error_msg:
            self.get_logger().warn(f"error_msg: {response.error_msg}")

    def _on_segmented_image(self, msg: Image) -> None:
        """Save the first annotated frame the node publishes, then ignore the rest.

        Only the first frame is kept: the node keeps publishing while it runs,
        and later frames would overwrite the one belonging to this snapshot.

        Args:
            msg: Annotated `sensor_msgs/Image` from `/vision/segmented_image`.
        """
        if self._got_frame:
            return
        self._got_frame = True
        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        import cv2

        cv2.imwrite(self._out_path, cv_image)
        self.get_logger().info(f"Saved annotated frame to {self._out_path}")


def main() -> None:
    """Parse the command line, send one snapshot goal and wait for the replies.

    Spins for a fixed 15 seconds rather than until the callbacks have fired so that the run is bound if no reply ever arrive.
    """
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
