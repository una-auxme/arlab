"""ROS2 YOLO-based object detection node with KB integration.

This module defines a ROS2 node that performs real-time object detection
using Ultralytics YOLO segmentation, converts results into
semantic/geometric entities, and communicates with a knowledge base (KB)
via ROS services.

TODO:
    - Implement IoU-based tracking of entities and update existing
      entries in the knowledge base instead of re-adding them each frame.

Maintainers:
    Aleksander Michalak <aleksander1.michalak@uni-a.de>
"""

import asyncio
import os
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from arlab_asyncio_executor.executors import AsyncIOExecutor
from arlab_knowledge_interfaces.msg import Entity, EntityType
from arlab_knowledge_interfaces.srv import AddEntity, DelEntities, GetEntities
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String  # <-- needed for 'name' field in entities
from ultralytics import YOLO
from vision_msgs.msg import Point2D


class ObjectDetection(Node):
    """ROS2 node for real-time object detection and KB synchronization.

    The node:
        1) Subscribes to RGB images and camera intrinsics.
        2) Runs YOLO segmentation on incoming frames.
        3) Converts detections to semantic entities (label, bbox, point cloud,
           pose).
        4) Interacts with a knowledge base through ROS services to insert or
           update entities.

    Attributes:
        visualize (bool): Whether to display a live visualization window.
        bridge (CvBridge): ROS–OpenCV conversion bridge.
        model (YOLO): Ultralytics YOLO segmentation model.
        camera_intrinsics (dict[str, float] | None): Focal lengths and principal
            point.
        service_client_group (MutuallyExclusiveCallbackGroup): Callback group for
            services.
        prefix (str): Namespace prefix for KB services.
        client_get_entities: Service client for fetching entities.
        client_del_entities: Service client for deleting entities.
        client_add_entities: Service client for adding an entity.
    """

    def __init__(self) -> None:
        """Initialize the node, parameters, subscriptions, and service clients."""
        super().__init__(type(self).__name__)

        package_share_dir = get_package_share_directory("arlab_computer_vision")
        default_yolo_weights = os.path.join(
            package_share_dir,
            "yolo_weights",
            "yolo11n-seg.pt",  # 'yolo11n-trained.pt' for detection model
        )

        # Declare configurable parameters.
        self.declare_parameter("yolo_weights", default_yolo_weights)
        self.declare_parameter("visualize", True)
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("use_depth", True)
        self.declare_parameter("sync_tolerance", 0.5)  # 500ms tolerance (default)

        # Load parameters.
        yolo_weights = (
            self.get_parameter("yolo_weights").get_parameter_value().string_value
        )
        self.visualize = (
            self.get_parameter("visualize").get_parameter_value().bool_value
        )
        log_level_str = (
            self.get_parameter("log_level").get_parameter_value().string_value
        )

        # Set logger level
        from rclpy.impl.logging_severity import LoggingSeverity

        log_level_map = {
            "DEBUG": LoggingSeverity.DEBUG,
            "INFO": LoggingSeverity.INFO,
            "WARN": LoggingSeverity.WARN,
            "ERROR": LoggingSeverity.ERROR,
            "FATAL": LoggingSeverity.FATAL,
        }
        log_level = log_level_map.get(log_level_str.upper(), LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)
        self.get_logger().info(f"Log level set to: {log_level_str.upper()}")

        self.use_depth = (
            self.get_parameter("use_depth").get_parameter_value().bool_value
        )
        self.sync_tolerance = (
            self.get_parameter("sync_tolerance").get_parameter_value().double_value
        )

        # Check if using segmentation model based on filename
        self.use_segmentation = "-seg.pt" in yolo_weights
        if self.use_segmentation:
            self.get_logger().info("Using YOLO segmentation model.")
        else:
            self.get_logger().info("Using YOLO detection model.")

        # Init CV bridge and YOLO model.
        self.bridge = CvBridge()
        self.model = YOLO(yolo_weights)
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {device}.")
        self.model.to(device)
        # Use half precision (FP16) for faster GPU inference
        if device == "cuda":
            self.model.half()
            self.get_logger().info("Using FP16 (half precision) for GPU inference")

        # Close any existing OpenCV windows from previous runs
        if self.visualize:
            cv2.destroyAllWindows()

        # Flag to track if a frame is currently being processed
        self._processing_frame = False

        # Frame statistics
        self._frames_processed = 0
        self._frames_skipped = 0

        # Thread pool for YOLO inference (max_workers=1 to avoid queue buildup)
        # Frame-skipping ensures we don't create multiple tasks
        self._yolo_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="yolo"
        )

        self.camera_intrinsics: dict[str, float] | None = None
        self._camera_intrinsics_set = False

        # Service clients.
        self.service_client_group = MutuallyExclusiveCallbackGroup()
        self.prefix = "/arlab/knowledge"

        self.client_get_entities = self.create_client(
            GetEntities,
            f"{self.prefix}/get_entities",
            callback_group=self.service_client_group,
        )
        self.client_del_entities = self.create_client(
            DelEntities,
            f"{self.prefix}/del_entities",
            callback_group=self.service_client_group,
        )
        self.client_add_entities = self.create_client(
            AddEntity,
            f"{self.prefix}/add_entity",
            callback_group=self.service_client_group,
        )

        # Flag to track if KB services are available
        self._kb_services_available = False
        # Track if periodic check task is running
        self._periodic_check_task = None
        # Check services availability initially
        self._check_kb_services()

        # Subscribe to camera info (async; sets intrinsics once available).
        self.create_subscription(
            CameraInfo,
            "camera_info",
            self.camera_info_callback,
            qos_profile=1,
        )
        self.get_logger().info("Subscribed to camera_info topic")

        # Subscribe to RGB and depth image streams with synchronization.
        if self.use_depth:
            # Create subscribers for message_filters
            rgb_sub = Subscriber(self, Image, "camera_color_image")
            depth_sub = Subscriber(self, Image, "camera_depth_image")

            # Create time synchronizer
            # queue_size=10 allows buffering of messages for better synchronization
            # when topics have different rates
            self.sync = ApproximateTimeSynchronizer(
                [rgb_sub, depth_sub],
                queue_size=10,
                slop=self.sync_tolerance,
            )
            self.sync.registerCallback(self._synced_callback)
            self.get_logger().info(
                f"Subscribed to synchronized RGB and depth topics "
                f"(tolerance: {self.sync_tolerance}s)"
            )
        else:
            # Subscribe to RGB image stream only.
            self.create_subscription(
                Image,
                "camera_color_image",
                self._process_data_sync,
                qos_profile=1,  # Only keep latest frame to reduce delay
            )
            self.get_logger().info("Subscribed to camera_color_image topic")

    async def async_init(self):
        """Async initialization for AsyncIOExecutor.

        This method is required by AsyncIOExecutor but doesn't need
        to perform any initialization in this node.
        """
        # Start background task to periodically check KB services if not available
        if not self._kb_services_available:
            self._start_periodic_service_check()

        # Start frame statistics reporting task
        asyncio.create_task(self._frame_statistics_reporter())

    def _check_kb_services(self) -> bool:
        """Check if all KB services are available.

        Returns:
            bool: True if all services are available, False otherwise.
        """
        available = (
            self.client_get_entities.wait_for_service(timeout_sec=1.0)
            and self.client_add_entities.wait_for_service(timeout_sec=1.0)
            and self.client_del_entities.wait_for_service(timeout_sec=1.0)
        )
        self._kb_services_available = available
        if available:
            self.get_logger().info("Knowledge base services are available.")
        else:
            self.get_logger().warn(
                "Knowledge base services not available. Will retry periodically."
            )
        return available

    async def _periodic_service_check(self):
        """Periodically check KB services availability every 30 seconds
        if not available.
        """
        while True:
            await asyncio.sleep(30.0)
            if not self._kb_services_available:
                self.get_logger().info(
                    "Retrying to connect to knowledge base services..."
                )
                self._check_kb_services()
            else:
                # If services are available, stop periodic checking
                break

    def _start_periodic_service_check(self):
        """Start periodic service check task if not already running."""
        if self._periodic_check_task is None or self._periodic_check_task.done():
            self._periodic_check_task = asyncio.create_task(
                self._periodic_service_check()
            )

    async def _frame_statistics_reporter(self):
        """Report frame statistics every 10 seconds."""
        while True:
            await asyncio.sleep(10.0)
            total = self._frames_processed + self._frames_skipped
            if total > 0:
                processed_pct = (
                    self._frames_processed / total * 100 if total > 0 else 0.0
                )
                skipped_pct = self._frames_skipped / total * 100 if total > 0 else 0.0
                self.get_logger().info(
                    f"Frame statistics (last 10s): "
                    f"Processed: {self._frames_processed} ({processed_pct:.1f}%), "
                    f"Skipped: {self._frames_skipped} ({skipped_pct:.1f}%), "
                    f"Total: {total}"
                )
                # Reset counters
                self._frames_processed = 0
                self._frames_skipped = 0

    def _process_data_sync(
        self, rgb_msg: Image, depth_msg: Image | None = None
    ) -> None:
        """Synchronous wrapper for async process_data callback.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.
            depth_msg: Optional incoming depth `sensor_msgs/Image`.
        """
        self.get_logger().debug(
            f"_process_data_sync called: RGB {rgb_msg.width}x{rgb_msg.height}, "
            f"Depth: {'present' if depth_msg is not None else 'missing'}"
        )
        try:
            asyncio.get_running_loop()
            self.get_logger().debug("Event loop found, creating task")
            asyncio.create_task(self.process_data(rgb_msg, depth_msg))
        except RuntimeError:
            # No event loop running, create one
            self.get_logger().warn("No event loop running, creating new one")
            asyncio.run(self.process_data(rgb_msg, depth_msg))

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Extract camera intrinsics from CameraInfo message.

        Args:
            msg: CameraInfo message with intrinsic matrix K.
        """
        K = np.array(msg.k, dtype=float).reshape(3, 3)
        new_intrinsics = {
            "fx": K[0, 0],
            "fy": K[1, 1],
            "cx": K[0, 2],
            "cy": K[1, 2],
        }

        # Only log if this is the first time or if values changed
        if not self._camera_intrinsics_set:
            self.camera_intrinsics = new_intrinsics
            self._camera_intrinsics_set = True
            self.get_logger().info(
                f"Camera intrinsics set: {msg.width}x{msg.height}, "
                f"fx={K[0, 0]:.1f}, fy={K[1, 1]:.1f}"
            )
        else:
            # Update silently (intrinsics shouldn't change, but update just in case)
            self.camera_intrinsics = new_intrinsics

    def _depth_image_callback(self, depth_msg: Image) -> None:
        """Callback for depth image messages.

        Args:
            depth_msg: Incoming depth `sensor_msgs/Image`.
        """
        self.get_logger().debug(
            f"Received depth image: {depth_msg.width}x{depth_msg.height}, "
            f"encoding: {depth_msg.encoding}"
        )

    def _synced_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        """Callback for synchronized RGB and depth image messages.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.
            depth_msg: Incoming depth `sensor_msgs/Image`.
        """
        # Calculate timestamp difference for debugging
        rgb_stamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        depth_stamp = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        time_diff = abs(rgb_stamp - depth_stamp)

        self.get_logger().info(
            f"Synchronized frames received: RGB {rgb_msg.width}x{rgb_msg.height}, "
            f"Depth {depth_msg.width}x{depth_msg.height}, "
            f"time_diff={time_diff * 1000:.1f}ms"
        )
        # Forward to processing with depth
        self._process_data_sync(rgb_msg, depth_msg)

    async def process_data(
        self, rgb_msg: Image, depth_msg: Image | None = None
    ) -> None:
        """Process incoming RGB images and sync detections with KB.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.
            depth_msg: Optional incoming depth `sensor_msgs/Image`.

        Notes:
            - Returns early if camera intrinsics are unknown.
            - Depth processing will be implemented in later phases.
            - Uses async service calls for KB operations.
        """
        self.get_logger().debug(
            f"process_data called: RGB {rgb_msg.width}x{rgb_msg.height}, "
            f"Depth: {'present' if depth_msg is not None else 'missing'}"
        )

        # Skip frame if camera intrinsics not yet set (only check once)
        if not self._camera_intrinsics_set:
            self.get_logger().warn("Camera intrinsics not set yet, skipping frame")
            return

        # Skip frame if already processing one (frame-skipping to avoid queue buildup)
        if self._processing_frame:
            self._frames_skipped += 1
            self.get_logger().debug("Frame skipped - already processing")
            return
        self._processing_frame = True
        self._frames_processed += 1
        self.get_logger().debug("Frame processing started")

        try:
            # Convert ROS image to numpy array.
            # cv_bridge returns BGR by default, but YOLO expects RGB
            self.get_logger().debug("Converting image...")
            bgr_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding="bgr8",
            )
            # Convert BGR to RGB for YOLO inference
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            self.get_logger().debug("Image converted to RGB")

            # Keep BGR copy for visualization (OpenCV imshow expects BGR)
            bgr_image_for_viz = bgr_image if self.visualize else None

            # TODO: Integrate depth image handling if available.

            # Run YOLO inference (segmentation/detection) in thread pool.
            # Frame-skipping ensures we don't create multiple tasks that wait.
            self.get_logger().debug("Starting YOLO inference...")
            try:
                loop = asyncio.get_event_loop()
                results = await loop.run_in_executor(
                    self._yolo_executor, self.model, rgb_image
                )
                self.get_logger().debug("YOLO inference returned")
                result = results[0]
                num_detections = len(result.boxes) if hasattr(result, "boxes") else 0
                self.get_logger().debug(
                    f"YOLO inference complete: {num_detections} detections "
                    f"in {rgb_msg.width}x{rgb_msg.height} image"
                )
            except Exception as e:
                self.get_logger().error(f"YOLO inference failed: {e}", exc_info=True)
                return

            # Convert YOLO results to entity dicts.
            self.get_logger().debug("Converting YOLO results to entities...")
            entities_cv = generate_entities_from_yolo_result(
                result=result,
                class_names=self.model.names,
                frame=None,  # Don't visualize during entity conversion
                use_segmentation=self.use_segmentation,
            )
            self.get_logger().debug(f"Generated {len(entities_cv)} entities")

            # Visualize separately if needed (non-blocking)
            if self.visualize and bgr_image_for_viz is not None:
                self._visualize_detections(bgr_image_for_viz, result, self.model.names)

            # Log detected objects
            if len(entities_cv) > 0:
                object_names = [e["name"].data for e in entities_cv]
                self.get_logger().info(
                    f"Detected {len(entities_cv)} object(s): {', '.join(object_names)}"
                )
            else:
                self.get_logger().debug("No objects detected in this frame")

            # Debug purpose
            return

            # Check if KB services are available (flag set in init/periodic check)
            if not self._kb_services_available:
                self.get_logger().debug(
                    "Knowledge base services not available, skipping KB update."
                )
                return

            # Verify services are still available
            # (they might have failed during runtime)
            if not (
                self.client_get_entities.service_is_ready()
                and self.client_add_entities.service_is_ready()
                and self.client_del_entities.service_is_ready()
            ):
                self.get_logger().warn(
                    "Knowledge base services became unavailable during runtime. "
                    "Will retry periodically."
                )
                self._kb_services_available = False
                self._start_periodic_service_check()
                return

            # Retrieve existing entities from KB (optional, currently unused).
            get_entities_req = GetEntities.Request()
            get_entities_req.entity_type.id = EntityType.ENTITY
            # resp: GetEntities.Response = await self.client_get_entities.call_async(
            #     get_entities_req
            # )

            # Insert (or upsert) detected entities into KB.
            # Parallelize service calls for better performance
            if len(entities_cv) > 0:
                now = self.get_clock().now().to_msg()
                tasks = []
                for entity_cv in entities_cv:
                    add_entity_req = AddEntity.Request()
                    add_entity_req.data = Entity(
                        description=f"Detected: {entity_cv['name'].data}",
                        pose=entity_cv["pose"],
                        pose_reference_frame=rgb_msg.header.frame_id,
                        stamp=now,
                        reference_frame="camera_link",
                    )
                    tasks.append(self.client_add_entities.call_async(add_entity_req))
                # Execute all service calls in parallel
                await asyncio.gather(*tasks)
                self.get_logger().debug(
                    f"Added {len(entities_cv)} entities to knowledge base in parallel"
                )
        finally:
            self._processing_frame = False

    def _visualize_detections(
        self, frame: np.ndarray, result, class_names: dict
    ) -> None:
        """Visualize YOLO detections on frame (non-blocking).

        Args:
            frame: BGR image frame to draw on.
            result: YOLO result object.
            class_names: Mapping from class indices to names.
        """
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            cv2.imshow("YOLO Detections", frame)
            cv2.waitKey(1)
            return

        xywh_all = boxes.xywh.detach().cpu().numpy()
        class_ids = boxes.cls.detach().cpu().numpy().astype(int)

        for i in range(len(boxes)):
            cx, cy, w, h = xywh_all[i]
            label = str(class_names[class_ids[i]])

            x1 = int(cx - w / 2.0)
            y1 = int(cy - h / 2.0)
            x2 = int(cx + w / 2.0)
            y2 = int(cy + h / 2.0)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                label,
                (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        cv2.imshow("YOLO Detections", frame)
        cv2.waitKey(1)


def pose_from_point2d(point2d: Point2D) -> Pose:
    """Convert a 2D point to a 3D pose with fixed orientation.

    Args:
        point2d: 2D point (e.g., bounding box center) in image coordinates.

    Returns:
        Pose: Pose with z=0 and identity orientation.
    """
    return Pose(
        position=Point(x=point2d.x, y=point2d.y, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def generate_entities_from_yolo_result(
    result, class_names, frame: np.ndarray | None = None, use_segmentation: bool = False
) -> list[dict]:
    """Convert a YOLO result into a list of entity dicts.

    Builds minimal entities used by this node:
        - `name` (std_msgs/String)
        - `pose` (geometry_msgs/Pose from bbox center)

    Args:
        result: Ultralytics YOLO result for one image.
        class_names: Mapping from class indices to names (list or dict).
        frame: Optional RGB frame for visualization overlay.
        use_segmentation: Whether the model is a segmentation model.

    Returns:
        list[dict]: Each dict has keys `name` and `pose`.
    """
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return []

    # Get all boxes at once - works for both detection and segmentation
    xywh_all = boxes.xywh.detach().cpu().numpy()  # Shape: [N, 4]
    class_ids = boxes.cls.detach().cpu().numpy().astype(int)  # Shape: [N]
    entities: list[dict] = []

    for i in range(len(boxes)):
        cx, cy, w, h = xywh_all[i]

        label = str(class_names[class_ids[i]])
        name_msg = String(data=label)

        pose_msg = pose_from_point2d(Point2D(x=float(cx), y=float(cy)))

        entities.append(
            {
                "name": name_msg,
                "pose": pose_msg,
            }
        )

    return entities


def main(args=None):
    """Entry point for the object_detection node."""
    rclpy.init(args=args)
    object_detection_node = ObjectDetection()
    executor = AsyncIOExecutor(async_init=object_detection_node.async_init())
    executor.add_node(object_detection_node)

    object_detection_node.get_logger().info(
        "Object detection node started. Shut down with CTRL-C"
    )
    try:
        executor.spin()
    finally:
        if object_detection_node.visualize:
            cv2.destroyAllWindows()
        # Shutdown thread pool
        object_detection_node._yolo_executor.shutdown(wait=True)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
