"""ROS2 YOLO-based object detection node with KB integration.

This module defines a ROS2 node that performs real-time object detection
using Ultralytics YOLO segmentation, converts results into
semantic/geometric entities, and communicates with a knowledge base (KB)
via ROS services.


Maintainers:
    Aleksander Michalak <aleksander1.michalak@uni-a.de>
    Peter Viechter <peter.viechter@uni-a.de>
"""

import os
import time
from copy import deepcopy
from threading import Lock
from time import sleep
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import rclpy.executors
import ros2_numpy
import torch
from ament_index_python.packages import get_package_share_directory
from arlab_common_interfaces.action import VisionSnapshotAction
from arlab_common_interfaces.msg import VisionSnapshotCommand, VisionSnapshotResponse
from arlab_knowledge_interfaces.msg import (
    Entity,
    EntityPickable,
    EntityType,
    Result,
    Shape,
)
from arlab_knowledge_interfaces.srv import AddEntity, DelEntities, GetEntities, UpdShape
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from numpy.typing import NDArray
from rclpy.action.server import ActionServer, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from ros2_numpy.point_cloud2 import (
    array_to_pointcloud2,
    pointcloud2_to_array,
    pointcloud2_to_xyz_array,
)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from torchvision.transforms.functional import resize
from ultralytics import YOLO


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
        bridge (CvBridge): ROS-OpenCV conversion bridge.
        model (YOLO): Ultralytics YOLO segmentation model.
        camera_intrinsics (dict[str, float] | None): Focal lengths and principal
            point.
        service_client_group (MutuallyExclusiveCallbackGroup): Callback group for
            services.
        prefix (str): Namespace prefix for KB services.
        client_get_entities: Service client for fetching entities.
        client_del_entities: Service client for deleting entities.
        client_add_entities: Service client for adding an entity.
        client_upd_shape: Service client for updating entity shapes.
    """

    def __init__(self) -> None:
        """Initialize the node, parameters, subscriptions, and service clients."""
        super().__init__(type(self).__name__)

        package_share_dir = get_package_share_directory("arlab_computer_vision")
        default_yolo_weights = os.path.join(
            package_share_dir,
            "yolo_weights",
            "yolo11n-seg_fruit_dataset_strong_geom_fruit.pt",
        )

        # Declare configurable parameters.
        self.declare_parameter("yolo_weights", default_yolo_weights)
        self.declare_parameter("visualize", True)
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("use_depth", True)
        self.declare_parameter("sync_tolerance", 0.5)  # 500ms tolerance (default)
        # Enable/disable depth clustering (default: False for better performance)
        self.declare_parameter("use_clustering", True)
        # Delete old entities before adding new ones
        self.declare_parameter("delete_old_entities", True)
        # Clear DB when no objects are detected (keeps DB in sync with camera)
        self.declare_parameter("clear_db_on_no_detection", True)
        # Maximum image width for YOLO inference (0 = no scaling)
        # Reduces memory usage and speeds up inference for large images
        self.declare_parameter("max_image_width", 640)
        # Target frame for TF transformations (default: "world")
        self.declare_parameter("target_frame", "camera_tool_link")

        # Enable snapshot mode
        self.declare_parameter("snapshot_mode", True)

        # If the model output should be plotted and published
        self.visualize = (
            self.get_parameter("visualize").get_parameter_value().bool_value
        )

        # Load parameters.
        yolo_weights = (
            self.get_parameter("yolo_weights").get_parameter_value().string_value
        )
        # Visualization parameter removed for performance reasons
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

        # Load delete old entities parameter
        self._delete_old_entities = (
            self.get_parameter("delete_old_entities").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Delete old entities: {self._delete_old_entities}")

        # Load clear DB on no detection parameter
        self._clear_db_on_no_detection = (
            self.get_parameter("clear_db_on_no_detection")
            .get_parameter_value()
            .bool_value
        )
        self.get_logger().info(
            f"Clear DB on no detection: {self._clear_db_on_no_detection}"
        )

        # Load max image width parameter
        self.max_image_width = (
            self.get_parameter("max_image_width").get_parameter_value().integer_value
        )
        if self.max_image_width > 0:
            self.get_logger().info(
                f"YOLO input size: {self.max_image_width}x{self.max_image_width} "
                f"(images will be scaled to square size)"
            )
        else:
            self.get_logger().info(
                "Image scaling disabled - using original image size for YOLO"
            )

        # Enable snapshot mode?
        self._snapshot_mode = (
            self.get_parameter("snapshot_mode").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Snapshot mode: {self._snapshot_mode}")

        if self._snapshot_mode:
            self._snapshot_group = MutuallyExclusiveCallbackGroup()
            self._action_server = ActionServer(
                self,
                VisionSnapshotAction,
                "/vision/snapshot",
                execute_callback=self._snapshot_execute_callback,
                goal_callback=self._snapshot_goal_callback,
                callback_group=self._snapshot_group,
            )

        self.vision_data_mutex = Lock()

        # Check if using segmentation model based on filename
        self.use_segmentation = "-seg.pt" in yolo_weights
        if self.use_segmentation:
            self.get_logger().info("Using YOLO segmentation model.")
        else:
            self.get_logger().info("Using YOLO detection model.")

        # Init CV bridge and YOLO model.
        self.bridge = CvBridge()
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {device}.")

        # Initialize YOLO model
        # Note: YOLO doesn't support device parameter in __init__,
        # but will use the device specified during first inference
        self.model = YOLO(yolo_weights)
        # Store device - will be verified/updated below
        self.device = device

        # Use half precision (FP16) for faster GPU inference
        if device == "cuda":
            # Verify CUDA is actually available
            if not torch.cuda.is_available():
                self.get_logger().warn(
                    "CUDA not available despite device='cuda'. Falling back to CPU."
                )
                self.device = "cpu"
            else:
                # Explicitly move model to GPU
                self.model.to("cuda")
                # Verify GPU is accessible
                try:
                    gpu_name = torch.cuda.get_device_name(0)
                    gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1e9
                    self.get_logger().info(
                        f"GPU detected: {gpu_name} ({gpu_memory:.1f} GB total memory)"
                    )
                except Exception as e:
                    self.get_logger().warn(f"GPU verification failed: {e}")

                self.get_logger().info("Using FP16 (half precision) for GPU inference")
                # Warmup: Load model on GPU and compile CUDA kernels
                try:
                    import numpy as np

                    # Calculate warmup size based on max_image_width
                    # This matches the actual scaled image size used in inference
                    if self.max_image_width > 0:
                        warmup_size = (
                            self.max_image_width,
                            self.max_image_width,
                            3,
                        )
                    else:
                        warmup_size = (640, 640, 3)

                    self.get_logger().debug(
                        f"Warmup image size: {warmup_size[1]}x{warmup_size[0]}"
                    )

                    # Create dummy input and run warmup inference
                    # This loads the model on GPU and compiles CUDA kernels
                    dummy_input = np.zeros(warmup_size, dtype=np.uint8)
                    for i in range(3):  # 3 warmup runs to compile all kernels
                        _ = self.model(dummy_input, verbose=False, device=device)
                        if torch.cuda.is_available():
                            torch.cuda.synchronize()

                    # Verify GPU was actually used
                    if torch.cuda.is_available():
                        model_device = (
                            self.model.device
                            if hasattr(self.model, "device")
                            else "unknown"
                        )
                        self.get_logger().info(
                            f"Model warmup completed on GPU "
                            f"(device: {model_device}, "
                            f"warmup size: {warmup_size[1]}x{warmup_size[0]})"
                        )
                    else:
                        self.get_logger().warn(
                            "Model warmup completed but GPU not verified"
                        )
                except Exception as e:
                    self.get_logger().warn(f"Model warmup failed: {e}")

        # Frame statistics
        self._frames_processed = 0
        self._frames_skipped = 0

        self.camera_intrinsics_matrix: Optional[NDArray] = None
        self.camera_intrinsics: dict[str, float] | None = None
        self._camera_intrinsics_set = False
        # Cached intrinsics values for faster access
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None
        # Store RGB image resolution for intrinsics scaling
        self._rgb_width: int | None = None
        self._rgb_height: int | None = None
        # Store depth image resolution
        self._depth_width: int | None = None
        self._depth_height: int | None = None

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
        self.client_upd_shape = self.create_client(
            UpdShape,
            f"{self.prefix}/upd_shape",
            callback_group=self.service_client_group,
        )

        # Flag to track if KB services are available
        self._kb_services_available = False

        # Load target frame parameter
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.get_logger().info(
            f"Target frame for TF transformations: '{self.target_frame}'"
        )

        # TF2 Buffer for coordinate frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
            pointcloud_sub = Subscriber(self, PointCloud2, "camera_point_cloud")

            # Create time synchronizer
            # queue_size=1 minimizes memory usage - only buffer 1 synchronized pair
            # This prevents OOM during slow inference and reduces swap pressure
            self.sync = ApproximateTimeSynchronizer(
                [rgb_sub, pointcloud_sub],
                queue_size=1,  # Minimized to reduce memory footprint
                slop=self.sync_tolerance,
            )
            self.sync.registerCallback(self._image_data_callback)
            self.get_logger().info(
                f"Subscribed to synchronized RGB and depth topics "
                f"(tolerance: {self.sync_tolerance}s)"
            )
        else:
            # Subscribe to RGB image stream only.
            self.create_subscription(
                Image,
                "camera_color_image",
                self._image_data_callback,
                qos_profile=1,  # Only keep latest frame to reduce delay
            )
            self.get_logger().info("Subscribed to camera_color_image topic")
        self.color_image: Optional[Image] = None
        self.pointcloud: Optional[PointCloud2] = None

        self.segmented_image_pub = self.create_publisher(
            Image, "/vision/segmented_image", qos_profile=1
        )
        self.debug_pointcloud_pub = self.create_publisher(
            PointCloud2, "/vision/debug_pc", qos_profile=1
        )

        self._knowledge_timer = self.create_timer(
            timer_period_sec=5.0, callback=self._check_kb_services
        )
        # Check services availability initially
        self._check_kb_services()
        self.create_timer(
            timer_period_sec=5.0, callback=self._frame_statistics_reporter
        )

    def _check_kb_services(self) -> bool:
        """Check if all KB services are available.

        Returns:
            bool: True if all services are available, False otherwise.
        """
        available = (
            self.client_get_entities.wait_for_service(timeout_sec=0.001)
            and self.client_add_entities.wait_for_service(timeout_sec=0.001)
            and self.client_del_entities.wait_for_service(timeout_sec=0.001)
        )
        self._kb_services_available = available
        if available:
            self.get_logger().info("Knowledge base services are available.")
            self._knowledge_timer.cancel()
        else:
            self.get_logger().warn(
                "Knowledge base services not available. Will retry periodically."
            )
        return available

    def _frame_statistics_reporter(self):
        """Report frame statistics every 10 seconds."""
        total = self._frames_processed + self._frames_skipped
        if total > 0:
            processed_pct = self._frames_processed / total * 100 if total > 0 else 0.0
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

    def _image_data_callback(
        self,
        rgb_msg: Image,
        pointcloud_msg: PointCloud2 | None = None,
    ) -> None:
        with self.vision_data_mutex:
            self.color_image = rgb_msg
            self.pointcloud = pointcloud_msg
            if not self._snapshot_mode:
                self._process_data(
                    rgb_msg,
                    pointcloud_msg,
                    delete_old_entities=self._delete_old_entities,
                )

    def _process_data(
        self,
        rgb_msg: Image,
        pointcloud_msg: PointCloud2 | None = None,
        delete_old_entities: bool = False,
        mask_hand: bool = False,
    ) -> None:
        """Fully synchronous frame processing - no async, no threads.

        This ensures Ultralytics YOLO has complete control over the CUDA
        context without interference from async event loops.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.
            depth_msg: Optional incoming depth `sensor_msgs/Image`.
        """
        t_start = time.perf_counter()

        # === 1. PREPROCESSING (CPU) ===
        t_pre = time.perf_counter()
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        original_height, original_width = rgb_image.shape[:2]

        # Scale to YOLO input size
        if self.max_image_width > 0:
            yolo_size = self.max_image_width
            scale_x = yolo_size / original_width
            scale_y = yolo_size / original_height
            yolo_image = cv2.resize(
                rgb_image, (yolo_size, yolo_size), interpolation=cv2.INTER_LINEAR
            )
        else:
            yolo_image = rgb_image
            scale_x = scale_y = 1.0
        preprocess_ms = (time.perf_counter() - t_pre) * 1000

        # === 2. YOLO INFERENCE (GPU) ===
        t_yolo = time.perf_counter()
        result, num_detections = self._run_yolo_inference(yolo_image)
        inference_ms = (time.perf_counter() - t_yolo) * 1000

        if self.visualize:
            # Plot model result if enabled
            self.segmented_image_pub.publish(
                self.bridge.cv2_to_imgmsg(result.plot(), "bgr8")
            )

        # === 3. POSTPROCESSING (CPU) ===
        t_post = time.perf_counter()
        old_entity_ids = self._kb_get_entities_for_deletion()
        entities: List[Tuple[Entity, Shape]] = []

        if num_detections > 0:
            # Scale boxes back
            if scale_x != 1.0 or scale_y != 1.0:
                if hasattr(result, "boxes") and result.boxes is not None:
                    boxes = result.boxes
                    if hasattr(boxes, "xywh") and boxes.xywh is not None:
                        boxes.xywh[:, 0] /= scale_x
                        boxes.xywh[:, 1] /= scale_y
                        boxes.xywh[:, 2] /= scale_x
                        boxes.xywh[:, 3] /= scale_y

            # Extract masks
            masks = self._extract_masks(
                result, original_height, original_width, mask_hand=mask_hand
            )

            # Depth processing
            if (
                pointcloud_msg is not None
                and masks is not None
                and self.use_depth
                and self.camera_intrinsics_matrix is not None
            ):
                # Transform from depth frame to target(world) frame
                depth_to_target_msg = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    pointcloud_msg.header.frame_id,
                    Time(seconds=0.0),
                    timeout=Duration(seconds=1.0),
                )
                depth_to_target: NDArray = ros2_numpy.numpify(
                    depth_to_target_msg.transform
                )

                structured_points = deepcopy(pointcloud2_to_array(pointcloud_msg))
                sp_np = np.stack(
                    (
                        structured_points["x"],
                        structured_points["y"],
                        structured_points["z"],
                        np.ones(structured_points.shape[0]),
                    )
                )
                target_sp = depth_to_target @ sp_np
                target_sp = target_sp / target_sp[3]
                structured_points["x"] = target_sp[0]
                structured_points["y"] = target_sp[1]
                structured_points["z"] = target_sp[2]
                # structured_points now contains the pointcloud in target coordinates

                # Transform from depth frame to color image frame
                depth_to_color_msg = self.tf_buffer.lookup_transform(
                    rgb_msg.header.frame_id,
                    pointcloud_msg.header.frame_id,
                    Time.from_msg(rgb_msg.header.stamp),
                    timeout=Duration(seconds=1.0),
                )
                depth_to_color: NDArray = ros2_numpy.numpify(
                    depth_to_color_msg.transform
                )

                np_points = pointcloud2_to_xyz_array(pointcloud_msg)
                np_points = np_points.transpose(1, 0)
                np_points = np.concatenate(
                    [np_points, np.ones((1, np_points.shape[-1]))]
                )
                np_points = depth_to_color @ np_points
                # np_points contains pointcloud in color frame coordinates
                camera_points = self.camera_intrinsics_matrix @ np_points[:3]
                camera_points = camera_points / camera_points[2]
                camera_points = camera_points[:2]
                # camera_points contains pointcloud in color image coordinates
                camera_points_idxs = camera_points.astype(np.int32)
                # x, y -> y, x
                camera_points_idxs[[0, 1]] = camera_points_idxs[[1, 0]]
                camera_points_idxs[0] = np.clip(
                    camera_points_idxs[0], 0, original_height - 1
                )
                camera_points_idxs[1] = np.clip(
                    camera_points_idxs[1], 0, original_width - 1
                )
                for i, mask in enumerate(masks):
                    # Converts the image mask into a per point mask
                    point_mask = mask[camera_points_idxs[0], camera_points_idxs[1]]
                    # Filters the points based on the mask.
                    # The structured_points were already transformed
                    # into the target frame.
                    # -> Final points for the entity
                    entity_points = structured_points[point_mask > 0.5]

                    if (
                        self.get_parameter("use_clustering")
                        .get_parameter_value()
                        .bool_value
                    ):
                        entity_points = self.cluster_entity_points(entity_points)

                    if len(entity_points) == 0:
                        continue
                    entity_pointcloud = array_to_pointcloud2(entity_points)
                    entity_pointcloud.header = pointcloud_msg.header
                    entity_pointcloud.header.frame_id = self.target_frame
                    self.debug_pointcloud_pub.publish(entity_pointcloud)

                    # Extract label
                    label = result.names[int(result.boxes.cls[i].item())]
                    entities.append(
                        create_entity(label, entity_points, entity_pointcloud.header)
                    )
            else:
                if pointcloud_msg is None:
                    self.get_logger().debug("No pointcloud message available")
                if not self.use_depth:
                    self.get_logger().debug("use_depth is False")

        # Update database
        if self._kb_services_available:
            self.kb_add_entities(entities)
        if delete_old_entities:
            if len(entities) > 0 or self._clear_db_on_no_detection:
                self._kb_delete_entities(ids=old_entity_ids)

        postprocess_ms = (time.perf_counter() - t_post) * 1000

        # Timing summary
        total_ms = (time.perf_counter() - t_start) * 1000
        self.get_logger().info(
            f"[Timing] Pre:{preprocess_ms:.1f}ms | "
            f"YOLO:{inference_ms:.1f}ms | "
            f"Post:{postprocess_ms:.1f}ms | "
            f"Total:{total_ms:.1f}ms | "
            # f"{queued_count} queued"
        )

    def cluster_entity_points(self, entity_points):
        if len(entity_points) == 0:
            return []
        # 1. Preparing data for sklearn dbscan
        xyz = np.stack(
            [entity_points["x"], entity_points["y"], entity_points["z"]], axis=1
        )

        # 2. Exectute DBSCAN clustering
        # eps= max distance, min_samples= min density
        db = DBSCAN(eps=0.01, min_samples=10).fit(xyz)
        labels = db.labels_

        # 3. Determine biggest cluster
        unique_labels, counts = np.unique(labels[labels >= 0], return_counts=True)

        if len(unique_labels) > 0:
            best_cluster = unique_labels[np.argmax(counts)]
            entity_points = entity_points[labels == best_cluster]
            return entity_points
        else:
            return []

    def _kb_get_entities_for_deletion(self) -> List[int]:
        if not self._kb_services_available:
            return []
        get_req = GetEntities.Request()
        # Only delete pickables
        get_req.entity_type.id = EntityType.PICKABLE
        response: GetEntities.Response = self.client_get_entities.call(get_req)

        if response.result.result_type != Result.SUCCESS:
            self.get_logger().error(f"Failed to get entities: {response.result.error}")
            return []

        return response.entities

    def _kb_delete_entities(self, ids: List[int]):
        if not self._kb_services_available:
            self.get_logger().warn(
                "Not deleting entities: Kb services not available!",
                throttle_duration_sec=2,
            )
            return
        if len(ids) == 0:
            return
        self.get_logger().info(f"Clearing {len(ids)} entities from kb")

        del_req = DelEntities.Request()
        del_req.entityids = ids
        del_response: DelEntities.Response = self.client_del_entities.call(del_req)
        if del_response.result.result_type != Result.SUCCESS:
            self.get_logger().error(
                f"Failed to delete entities: {del_response.result.error}"
            )

    def kb_add_entities(self, entities: List[Tuple[Entity, Shape]]):
        if not self._kb_services_available:
            self.get_logger().warn(
                "Not adding entities: Kb services not available!",
                throttle_duration_sec=2,
            )
            return
        # Add new entities
        for entity, shape in entities:
            add_req = AddEntity.Request()
            add_req.data = entity

            response: AddEntity.Response = self.client_add_entities.call(add_req)

            if response.result.result_type != Result.SUCCESS:
                self.get_logger().error(
                    f"Failed to save entity in database: {response.result.error}"
                )
                continue

            # Update shape in KB
            upd_shape_req = UpdShape.Request()
            upd_shape_req.entityid = response.entityid
            upd_shape_req.shape = shape
            upd_shape_req.stamp = shape.pointcloud.header.stamp

            shape_response: UpdShape.Response = self.client_upd_shape.call(
                upd_shape_req
            )
            if shape_response.result.result_type != Result.SUCCESS:
                self.get_logger().error(
                    f"Failed to update shape in database: {shape_response.result.error}"
                )
                continue

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

        # Store RGB image resolution for intrinsics scaling
        self._rgb_width = msg.width
        self._rgb_height = msg.height

        # Only log if this is the first time or if values changed
        if not self._camera_intrinsics_set:
            self.camera_intrinsics = new_intrinsics
            self._camera_intrinsics_set = True
            # Cache intrinsics values for faster access
            self._fx = new_intrinsics["fx"]
            self._fy = new_intrinsics["fy"]
            self._cx = new_intrinsics["cx"]
            self._cy = new_intrinsics["cy"]
            self.get_logger().info(
                f"Camera intrinsics set: {msg.width}x{msg.height}, "
                f"fx={K[0, 0]:.1f}, fy={K[1, 1]:.1f}"
            )
        else:
            # Update silently (intrinsics shouldn't change, but update just in case)
            self.camera_intrinsics = new_intrinsics
            # Update cached values
            self._fx = new_intrinsics["fx"]
            self._fy = new_intrinsics["fy"]
            self._cx = new_intrinsics["cx"]
            self._cy = new_intrinsics["cy"]

        self.camera_intrinsics_matrix = K

    def _run_yolo_inference(self, rgb_image: np.ndarray) -> tuple[Any, int]:
        """Run YOLO inference on RGB image (synchronous).

        Args:
            rgb_image: RGB image as numpy array.

        Returns:
            Tuple of (YOLO result, number of detections).
        """
        # Simple synchronous inference - Ultralytics has full control
        results = self.model(rgb_image, verbose=False, device=self.device)
        result = results[0]
        num_detections = len(result.boxes) if hasattr(result, "boxes") else 0
        return result, num_detections

    def _extract_masks(
        self, result: Any, image_height: int, image_width: int, mask_hand: bool
    ) -> np.ndarray | None:
        """Extract segmentation masks from YOLO result.

        Optimized version: Performs batch resize on GPU before transferring
        to CPU, avoiding multiple CPU-GPU transfers and sequential processing.

        Args:
            result: YOLO result object.
            image_height: Height of the image.
            image_width: Width of the image.
            mask_hand: ignore the hand portion of the image

        Returns:
            Binary masks as numpy array (N x H x W) or None if no masks.
        """
        has_masks = (
            hasattr(result, "masks")
            and result.masks is not None
            and len(result.masks) > 0
        )
        if not has_masks:
            return None

        # Keep masks on GPU for batch processing
        masks_gpu = result.masks.data.detach()  # Shape: [N, H, W]
        n, h, w = masks_gpu.shape
        if mask_hand:
            masks_gpu[:, h * 0.8 : h, :] = 0.0

        if masks_gpu.shape[1] != image_height or masks_gpu.shape[2] != image_width:
            masks_resized = resize(
                masks_gpu,
                size=[image_height, image_width],
            )

            # Single sync + transfer to CPU
            return masks_resized.cpu().numpy().astype(np.uint8)
        else:
            # No resize needed - single sync + transfer
            return masks_gpu.cpu().numpy().astype(np.uint8)

    def _snapshot_goal_callback(self, goal_request: VisionSnapshotAction.Goal):
        self.get_logger().info(
            "Received goal from Decision Making. "
            f"Delete old: {goal_request.command.clear_database}"
        )
        return GoalResponse.ACCEPT

    def _snapshot_execute_callback(self, goal_handle):
        with self.vision_data_mutex:
            self.color_image = None
            self.pointcloud = None
        # sleep to make sure position has stabilized
        for _ in range(5):
            sleep(1.0)
            with self.vision_data_mutex:
                if self.color_image is not None and self.pointcloud is not None:
                    break

        with self.vision_data_mutex:
            goal_command: VisionSnapshotCommand = goal_handle.request.command
            action_result = VisionSnapshotAction.Result()
            if self.color_image is None or self.pointcloud is None:
                action_result.response.result = (
                    VisionSnapshotResponse.ERROR_NO_IMAGE_DATA
                )
                action_result.response.error_msg = "No image data"
                self.get_logger().error(
                    "Failed to take vision snapshot: "
                    f"{action_result.response.error_msg}."
                )
                goal_handle.succeed()
                return action_result

            try:
                self._process_data(
                    self.color_image,
                    self.pointcloud,
                    delete_old_entities=goal_command.clear_database,
                    mask_hand=goal_command.mask_hand,
                )
                action_result.response.result = VisionSnapshotResponse.SUCCESS
            except Exception as e:
                action_result.response.result = VisionSnapshotResponse.ERROR_UNKNOWN
                action_result.response.error_msg = f"Exception: {e}"
            goal_handle.succeed()
            return action_result


def create_entity(
    label: str, structured_points: NDArray, points_header: Header
) -> Tuple[Entity, Shape]:
    entity = Entity()
    entity.entity_type.id = EntityType.PICKABLE
    entity.stamp = points_header.stamp
    entity.description = label
    entity.pose_reference_frame = points_header.frame_id
    entity.pose.position.x = float(np.average(structured_points["x"]))
    entity.pose.position.y = float(np.average(structured_points["y"]))
    entity.pose.position.z = float(np.average(structured_points["z"]))
    entity.pickable.object_name = label
    # TODO: update object category based om class
    entity.pickable.object_category = EntityPickable.OBJECT_CATEGORY_CYLINDER
    shape = Shape()
    shape.has_pointcloud = True
    shape.pointcloud = array_to_pointcloud2(structured_points)
    shape.pointcloud.header = points_header
    return (entity, shape)


def main(args=None):
    """Entry point for the object_detection node."""
    # from arlab_common.debugging import start_debugger

    # start_debugger(wait_for_client=True)

    rclpy.init(args=args)

    # Executor with exactly three threads
    # - One for the vision data callback
    # - One for the snapshot execution callback
    # - One for internal ros callback (action)
    # Note that this thread split is not enforced, but the two threads
    #   are necessary to not deadlock the node when issuing service calls
    # IMPORTANT: services must only be called
    #   from inside the timer callback -> from inside the behaviours
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)

    try:
        node = ObjectDetection()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
