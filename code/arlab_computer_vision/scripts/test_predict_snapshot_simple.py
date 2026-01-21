#!/usr/bin/env python3
"""Simple test script for predict_snapshot functionality.

This script starts the ObjectDetection node and tests predict_snapshot.

Usage:
    python3 test_predict_snapshot_simple.py
"""

import asyncio
import signal
import sys
import threading
import time

import rclpy
from arlab_asyncio_executor.executors import AsyncIOExecutor
from arlab_computer_vision.object_detection import ObjectDetection
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class SimpleSnapshotTester(Node):
    """Simple test node that captures one image and calls predict_snapshot."""

    def __init__(self, object_detection_node: ObjectDetection):
        """Initialize test node."""
        super().__init__("simple_snapshot_tester")
        self.object_detection_node = object_detection_node
        self.rgb_image = None
        self.depth_image = None
        self.rgb_received = False
        self.depth_received = False
        self.test_complete = False
        
        # Subscribe to camera_info with correct topic name
        # (ObjectDetection node subscribes to "camera_info" which needs remapping)
        self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.camera_info_callback,
            10
        )
        self.get_logger().info("Subscribed to /camera/color/camera_info for test")
    
    def camera_info_callback(self, msg):
        """Forward camera_info to ObjectDetection node."""
        # Call the ObjectDetection node's camera_info callback directly
        self.object_detection_node.camera_info_callback(msg)

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

    async def run_test(self):
        """Run the test."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Simple predict_snapshot Test")
        self.get_logger().info("=" * 60)

        # Wait for camera intrinsics
        self.get_logger().info("Waiting for camera_info...")
        for _ in range(300):  # 30 seconds max
            if self.object_detection_node._camera_intrinsics_set:
                break
            await asyncio.sleep(0.1)
        else:
            self.get_logger().error("Timeout: camera_info not received")
            self.test_complete = True
            return

        self.get_logger().info("Camera intrinsics OK")

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
            self.test_complete = True
            return

        # Wait a bit for depth
        await asyncio.sleep(0.5)

        # Call predict_snapshot
        self.get_logger().info("Calling predict_snapshot...")
        try:
            entities = await self.object_detection_node.predict_snapshot(
                self.rgb_image, self.depth_image
            )

            # Show results
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"Result: {len(entities)} entities detected")
            self.get_logger().info("=" * 60)

            if len(entities) > 0:
                for i, entity in enumerate(entities):
                    name = (
                        entity.get("name").data
                        if hasattr(entity.get("name"), "data")
                        else "unknown"
                    )
                    self.get_logger().info(f"  {i+1}. {name}")
                    if entity.get("pose"):
                        pose = entity["pose"]
                        if hasattr(pose, "position"):
                            pos = pose.position
                            self.get_logger().info(
                                f"     Position: x={pos.x:.3f}, "
                                f"y={pos.y:.3f}, z={pos.z:.3f}"
                            )
            else:
                self.get_logger().info("No objects detected")

            self.get_logger().info("=" * 60)
            self.get_logger().info("Test completed successfully!")

        except Exception as e:
            self.get_logger().error(f"Error: {e}", exc_info=True)
        finally:
            self.test_complete = True


def main():
    """Main function - runs executor in main thread."""
    rclpy.init()

    # Create nodes
    object_detection_node = ObjectDetection()
    tester = SimpleSnapshotTester(object_detection_node)

    # Setup executor
    executor = AsyncIOExecutor(async_init=object_detection_node.async_init())
    executor.add_node(object_detection_node)
    executor.add_node(tester)

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
            await tester.run_test()
            # Signal executor to shutdown
            executor.shutdown()

        loop.run_until_complete(test_wrapper())

    test_thread = threading.Thread(target=run_test_thread, daemon=True)
    test_thread.start()

    # Setup signal handler
    def signal_handler(sig, frame):
        """Handle shutdown signal."""
        tester.get_logger().info("Shutting down...")
        executor.shutdown()
        tester.destroy_node()
        object_detection_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Spin executor in main thread (required for signal handlers)
        executor.spin()
    except KeyboardInterrupt:
        tester.get_logger().info("Interrupted")
    finally:
        executor.shutdown()
        tester.destroy_node()
        object_detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
