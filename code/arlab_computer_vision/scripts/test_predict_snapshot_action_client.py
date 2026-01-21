#!/usr/bin/env python3
"""Test client for predict_snapshot action.

This script tests the predict_snapshot action server by:
1. Capturing RGB and depth images from topics
2. Sending them as a goal to the action server
3. Displaying feedback and results

Usage:
    # Terminal 1: Start the object detection node
    ros2 launch arlab_computer_vision object_detection_launch.py

    # Terminal 2: Run this test client
    python3 test_predict_snapshot_action_client.py
"""

import asyncio
import signal
import sys
import threading
import time

import rclpy
from arlab_common_interfaces.action import PredictSnapshot
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class PredictSnapshotActionClient(Node):
    """Test client for predict_snapshot action."""

    def __init__(self):
        """Initialize action client node."""
        super().__init__("predict_snapshot_action_client")
        self.rgb_image = None
        self.depth_image = None
        self.rgb_received = False
        self.depth_received = False

        # Create action client
        self._action_client = ActionClient(self, PredictSnapshot, "predict_snapshot")
        self.get_logger().info("Action client created, waiting for server...")

        # Subscribe to camera_info (for logging)
        self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.camera_info_callback,
            10,
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Log camera info reception."""
        self.get_logger().debug(f"Received camera_info: {msg.width}x{msg.height}")

    def rgb_callback(self, msg: Image):
        """Capture first RGB image."""
        if not self.rgb_received:
            self.rgb_image = msg
            self.rgb_received = True
            self.get_logger().info(
                f"Captured RGB: {msg.width}x{msg.height}, "
                f"frame_id={msg.header.frame_id}"
            )

    def depth_callback(self, msg: Image):
        """Capture first depth image."""
        if self.rgb_received and not self.depth_received:
            self.depth_image = msg
            self.depth_received = True
            self.get_logger().info(
                f"Captured Depth: {msg.width}x{msg.height}, "
                f"frame_id={msg.header.frame_id}"
            )

    async def wait_for_server(self, timeout_sec: float = 10.0):
        """Wait for action server to be available.

        Args:
            timeout_sec: Maximum time to wait in seconds.

        Returns:
            True if server is available, False on timeout.
        """
        self.get_logger().info("Waiting for action server...")
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self._action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("Action server is available!")
                return True
            await asyncio.sleep(0.1)
        self.get_logger().error("Action server not available after timeout")
        return False

    async def send_goal(self, rgb_image: Image, depth_image: Image | None = None):
        """Send goal to action server.

        Args:
            rgb_image: RGB image to process.
            depth_image: Optional depth image.

        Returns:
            Action result or None on error.
        """
        # Create goal
        goal_msg = PredictSnapshot.Goal()
        goal_msg.rgb_image = rgb_image
        if depth_image is not None:
            goal_msg.depth_image = depth_image

        self.get_logger().info("Sending goal to action server...")
        self.get_logger().info(
            f"  RGB: {rgb_image.width}x{rgb_image.height}, "
            f"frame_id={rgb_image.header.frame_id}"
        )
        if depth_image is not None:
            self.get_logger().info(
                f"  Depth: {depth_image.width}x{depth_image.height}, "
                f"frame_id={depth_image.header.frame_id}"
            )

        # Send goal and wait for result
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        try:
            goal_handle = await send_goal_future
            if not goal_handle.accepted:
                self.get_logger().error("Goal was rejected by server")
                return None

            self.get_logger().info("Goal accepted, waiting for result...")

            # Wait for result
            result_future = goal_handle.get_result_async()
            result = await result_future

            return result.result

        except Exception as e:
            self.get_logger().error(f"Error sending goal: {e}", exc_info=True)
            return None

    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server.

        Args:
            feedback_msg: Feedback message from action server.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: {feedback.status} "
            f"(progress: {feedback.progress:.1%}, "
            f"objects: {feedback.objects_detected})"
        )

    async def run_test(self):
        """Run the test."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("PredictSnapshot Action Client Test")
        self.get_logger().info("=" * 60)

        # Wait for action server
        if not await self.wait_for_server():
            return

        # Subscribe to images
        self.create_subscription(
            Image, "/camera/color/image_raw", self.rgb_callback, 10
        )
        self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self.depth_callback, 10
        )

        # Wait for RGB image
        self.get_logger().info("Waiting for RGB image...")
        for _ in range(100):  # 10 seconds max
            if self.rgb_received:
                break
            await asyncio.sleep(0.1)
        else:
            self.get_logger().error("Timeout: RGB image not received")
            return

        # Wait a bit for depth
        await asyncio.sleep(0.5)

        # Send goal
        self.get_logger().info("Sending goal to action server...")
        result = await self.send_goal(self.rgb_image, self.depth_image)

        # Display results
        self.get_logger().info("=" * 60)
        if result is not None:
            if result.success:
                self.get_logger().info(f"Action succeeded!")
                self.get_logger().info(f"Detected {len(result.entities)} entities:")
                for i, entity in enumerate(result.entities):
                    self.get_logger().info(f"  {i+1}. {entity.description}")
                    if entity.pose:
                        pos = entity.pose.position
                        self.get_logger().info(
                            f"     Position: x={pos.x:.3f}, "
                            f"y={pos.y:.3f}, z={pos.z:.3f}"
                        )
                        self.get_logger().info(
                            f"     Frame: {entity.pose_reference_frame}"
                        )
            else:
                self.get_logger().error(f"Action failed: {result.error_message}")
        else:
            self.get_logger().error("No result received from action server")
        self.get_logger().info("=" * 60)


def main():
    """Main function - runs executor in main thread."""
    rclpy.init()

    client = PredictSnapshotActionClient()

    # Setup async event loop for test
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # Run test in background thread
    def run_test_thread():
        """Run test in separate thread with async event loop."""
        async def test_wrapper():
            """Wrapper to run test."""
            # Wait a bit for nodes to initialize
            await asyncio.sleep(2.0)
            await client.run_test()
            # Signal executor to shutdown
            import rclpy.executors

            # We need to shutdown the executor
            # Since we're in a thread, we'll use a flag
            client._test_complete = True

        loop.run_until_complete(test_wrapper())

    test_thread = threading.Thread(target=run_test_thread, daemon=True)
    test_thread.start()

    # Setup signal handler
    def signal_handler(sig, frame):
        """Handle shutdown signal."""
        client.get_logger().info("Shutting down...")
        client.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Create executor for this node
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(client)

    try:
        # Spin executor in main thread (required for signal handlers)
        # Check if test is complete periodically
        while not hasattr(client, "_test_complete") or not client._test_complete:
            executor.spin_once(timeout_sec=0.1)
            if not test_thread.is_alive():
                # Test thread finished, wait a bit then shutdown
                time.sleep(0.5)
                break
    except KeyboardInterrupt:
        client.get_logger().info("Interrupted")
    finally:
        executor.shutdown()
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
