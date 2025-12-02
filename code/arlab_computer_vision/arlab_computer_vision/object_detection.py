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
from typing import Any

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
from sklearn.cluster import DBSCAN
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
        # Enable/disable depth clustering (default: False for better performance)
        self.declare_parameter("use_clustering", False)

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
        self.use_clustering = (
            self.get_parameter("use_clustering").get_parameter_value().bool_value
        )
        if self.use_depth and self.use_clustering:
            self.get_logger().info("Depth clustering enabled")
        elif self.use_depth and not self.use_clustering:
            self.get_logger().info("Depth clustering disabled (point cloud only)")

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

        # Thread pool for depth processing (clustering)
        self._depth_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="depth"
        )

        self.camera_intrinsics: dict[str, float] | None = None
        self._camera_intrinsics_set = False
        # Cached intrinsics values for faster access
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None

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

    def _depth_to_point_cloud(
        self, depth_image: np.ndarray, intrinsics: dict[str, float]
    ) -> np.ndarray:
        """Convert depth image to 3D point cloud using camera intrinsics.

        Args:
            depth_image: Depth image as numpy array (H x W), values in meters.
            intrinsics: Camera intrinsics dict with keys: fx, fy, cx, cy.

        Returns:
            Point cloud as numpy array (N x 3) where each row is [x, y, z]
            in camera frame. Invalid points (depth=0 or NaN) are filtered out.
        """
        height, width = depth_image.shape
        fx = intrinsics["fx"]
        fy = intrinsics["fy"]
        cx = intrinsics["cx"]
        cy = intrinsics["cy"]

        # Create pixel coordinate grids
        u, v = np.meshgrid(np.arange(width), np.arange(height))

        # Convert to camera coordinates
        # x = (u - cx) * z / fx
        # y = (v - cy) * z / fy
        # z = depth
        z = depth_image.astype(np.float32)

        # Filter invalid depth values (0, NaN, Inf)
        valid_mask = (z > 0) & np.isfinite(z)

        # Extract valid points
        u_valid = u[valid_mask]
        v_valid = v[valid_mask]
        z_valid = z[valid_mask]

        # Convert to 3D coordinates
        x_valid = (u_valid - cx) * z_valid / fx
        y_valid = (v_valid - cy) * z_valid / fy

        # Stack into point cloud (N x 3)
        point_cloud = np.column_stack([x_valid, y_valid, z_valid])

        return point_cloud

    def _cluster_depth_points(
        self,
        point_cloud: np.ndarray,
        eps: float = 0.05,
        min_samples: int = 15,
        max_distance: float = 3.0,
    ) -> list[dict[str, Any]]:
        """Cluster 3D points using DBSCAN and extract cluster information.

        Args:
            point_cloud: Point cloud as numpy array (N x 3).
            eps: DBSCAN epsilon parameter (maximum distance between points
                in a cluster).
            min_samples: Minimum number of points to form a cluster.
            max_distance: Maximum distance from camera to consider (filter far points).

        Returns:
            List of cluster dictionaries, each containing:
                - 'points': numpy array of cluster points (N x 3)
                - 'centroid': numpy array [x, y, z] of cluster centroid
                - 'size': number of points in cluster
                - 'bbox_3d': bounding box in 3D (min/max x, y, z)
        """
        if point_cloud.shape[0] == 0:
            return []

        # Filter points by distance (remove points too far from camera)
        distances = np.linalg.norm(point_cloud, axis=1)
        distance_mask = distances <= max_distance
        filtered_points = point_cloud[distance_mask]

        if filtered_points.shape[0] < min_samples:
            return []

        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(filtered_points)
        labels = clustering.labels_

        # Extract clusters (exclude noise points with label=-1)
        unique_labels = set(labels) - {-1}
        clusters = []

        for label in unique_labels:
            cluster_mask = labels == label
            cluster_points = filtered_points[cluster_mask]

            # Skip if cluster is too small (shouldn't happen due to min_samples,
            # but check anyway)
            if cluster_points.shape[0] < min_samples:
                continue

            # Calculate centroid
            centroid = np.mean(cluster_points, axis=0)

            # Calculate 3D bounding box
            bbox_3d = {
                "min_x": np.min(cluster_points[:, 0]),
                "max_x": np.max(cluster_points[:, 0]),
                "min_y": np.min(cluster_points[:, 1]),
                "max_y": np.max(cluster_points[:, 1]),
                "min_z": np.min(cluster_points[:, 2]),
                "max_z": np.max(cluster_points[:, 2]),
            }

            clusters.append(
                {
                    "points": cluster_points,
                    "centroid": centroid,
                    "size": cluster_points.shape[0],
                    "bbox_3d": bbox_3d,
                    "label": label,
                }
            )

        return clusters

    def _project_3d_to_2d(
        self, point_3d: np.ndarray, intrinsics: dict[str, float] | None = None
    ) -> tuple[float, float]:
        """Project a 3D point to 2D image coordinates.

        Args:
            point_3d: 3D point as numpy array [x, y, z] in camera frame.
            intrinsics: Optional camera intrinsics dict. If None, uses cached values.

        Returns:
            Tuple of (u, v) pixel coordinates in image space.
        """
        x, y, z = point_3d

        # Use cached values if available, otherwise use provided intrinsics
        if intrinsics is None:
            if self._fx is None:
                raise ValueError("Camera intrinsics not set")
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
        else:
            fx = intrinsics["fx"]
            fy = intrinsics["fy"]
            cx = intrinsics["cx"]
            cy = intrinsics["cy"]

        # Project to image plane: u = fx * x / z + cx, v = fy * y / z + cy
        if z <= 0:
            return (-1, -1)  # Invalid (behind camera)

        u = fx * x / z + cx
        v = fy * y / z + cy

        return (float(u), float(v))

    def _calculate_cluster_bbox_2d(
        self, cluster: dict[str, Any], intrinsics: dict[str, float] | None = None
    ) -> tuple[float, float, float, float] | None:
        """Calculate 2D bounding box for a cluster in image space (vectorized).

        Args:
            cluster: Cluster dictionary with 'points' (N x 3) and 'bbox_3d'.
            intrinsics: Optional camera intrinsics dict. If None, uses cached values.

        Returns:
            Tuple of (min_u, min_v, max_u, max_v) in image coordinates,
            or None if projection fails.
        """
        points_3d = cluster["points"]  # Shape: [N, 3]
        if len(points_3d) == 0:
            return None

        # Use cached values if available
        if intrinsics is None:
            if self._fx is None:
                return None
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
        else:
            fx = intrinsics["fx"]
            fy = intrinsics["fy"]
            cx = intrinsics["cx"]
            cy = intrinsics["cy"]

        # Vectorized projection: [N, 3] -> [N, 2]
        x, y, z = points_3d[:, 0], points_3d[:, 1], points_3d[:, 2]

        # Filter valid points (z > 0)
        valid_mask = z > 0
        if not np.any(valid_mask):
            return None

        # Project valid points
        u = fx * x[valid_mask] / z[valid_mask] + cx
        v = fy * y[valid_mask] / z[valid_mask] + cy

        # Filter points with valid projections (u >= 0, v >= 0)
        valid_2d = (u >= 0) & (v >= 0)
        if not np.any(valid_2d):
            return None

        u_valid = u[valid_2d]
        v_valid = v[valid_2d]

        return (
            float(np.min(u_valid)),
            float(np.min(v_valid)),
            float(np.max(u_valid)),
            float(np.max(v_valid)),
        )

    def _calculate_iou(
        self,
        bbox1: tuple[float, float, float, float],
        bbox2: tuple[float, float, float, float],
    ) -> float:
        """Calculate Intersection over Union (IoU) between two bounding boxes.

        Args:
            bbox1: Bounding box as (min_u, min_v, max_u, max_v).
            bbox2: Bounding box as (min_u, min_v, max_u, max_v).

        Returns:
            IoU value between 0 and 1.
        """
        min_u1, min_v1, max_u1, max_v1 = bbox1
        min_u2, min_v2, max_u2, max_v2 = bbox2

        # Calculate intersection
        inter_min_u = max(min_u1, min_u2)
        inter_min_v = max(min_v1, min_v2)
        inter_max_u = min(max_u1, max_u2)
        inter_max_v = min(max_v1, max_v2)

        if inter_max_u <= inter_min_u or inter_max_v <= inter_min_v:
            return 0.0  # No intersection

        inter_area = (inter_max_u - inter_min_u) * (inter_max_v - inter_min_v)

        # Calculate union
        area1 = (max_u1 - min_u1) * (max_v1 - min_v1)
        area2 = (max_u2 - min_u2) * (max_v2 - min_v2)
        union_area = area1 + area2 - inter_area

        if union_area <= 0:
            return 0.0

        return inter_area / union_area

    def _create_cluster_mask(
        self,
        cluster: dict[str, Any],
        intrinsics: dict[str, float] | None = None,
        image_width: int | None = None,
        image_height: int | None = None,
    ) -> np.ndarray | None:
        """Create binary mask from cluster points in image space (vectorized).

        Args:
            cluster: Cluster dictionary with 'points' (N x 3).
            intrinsics: Optional camera intrinsics dict. If None, uses cached values.
            image_width: Width of the image.
            image_height: Height of the image.

        Returns:
            Binary mask as numpy array (H x W) with 1s where cluster points are,
            or None if projection fails.
        """
        points_3d = cluster["points"]  # Shape: [N, 3]
        if len(points_3d) == 0:
            return None

        # Use cached values if available
        if intrinsics is None:
            if self._fx is None:
                return None
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
        else:
            fx = intrinsics["fx"]
            fy = intrinsics["fy"]
            cx = intrinsics["cx"]
            cy = intrinsics["cy"]

        # Vectorized projection: [N, 3] -> [N, 2]
        x, y, z = points_3d[:, 0], points_3d[:, 1], points_3d[:, 2]

        # Filter valid points (z > 0)
        valid_mask = z > 0
        if not np.any(valid_mask):
            return None

        # Project valid points
        u = (fx * x[valid_mask] / z[valid_mask] + cx).astype(int)
        v = (fy * y[valid_mask] / z[valid_mask] + cy).astype(int)

        # Filter points within image bounds
        if image_width is not None and image_height is not None:
            in_bounds = (u >= 0) & (u < image_width) & (v >= 0) & (v < image_height)
            if not np.any(in_bounds):
                return None
            u_valid = u[in_bounds]
            v_valid = v[in_bounds]
        else:
            # If no image dimensions provided, use all valid projections
            u_valid = u
            v_valid = v

        # Create mask using advanced indexing
        if image_width is None or image_height is None:
            # If dimensions not provided, determine from valid points
            if len(u_valid) == 0:
                return None
            max_u = int(np.max(u_valid)) + 1
            max_v = int(np.max(v_valid)) + 1
            image_width = max_u
            image_height = max_v

        mask = np.zeros((image_height, image_width), dtype=np.uint8)
        mask[v_valid, u_valid] = 1

        return mask

    def _calculate_mask_iou(self, mask1: np.ndarray, mask2: np.ndarray) -> float:
        """Calculate Intersection over Union (IoU) between two binary masks.

        Args:
            mask1: Binary mask as numpy array (H x W).
            mask2: Binary mask as numpy array (H x W).

        Returns:
            IoU value between 0 and 1.
        """
        if mask1.shape != mask2.shape:
            return 0.0

        # Calculate intersection (logical AND)
        intersection = np.logical_and(mask1, mask2).sum()

        # Calculate union (logical OR)
        union = np.logical_or(mask1, mask2).sum()

        if union == 0:
            return 0.0

        return float(intersection) / float(union)

    def _match_clusters_to_detections(
        self,
        yolo_result,
        clusters: list[dict[str, Any]],
        intrinsics: dict[str, float] | None,
        image_width: int,
        image_height: int,
        iou_threshold: float = 0.1,
    ) -> list[dict[str, Any]]:
        """Match depth clusters to YOLO detections using mask-based IoU.

        Uses segmentation masks if available, falls back to bounding boxes.

        Args:
            yolo_result: YOLO result object (with boxes and optionally masks).
            clusters: List of cluster dictionaries.
            intrinsics: Camera intrinsics dict.
            image_width: Width of the image.
            image_height: Height of the image.
            iou_threshold: Minimum IoU for a valid match (default: 0.1).

        Returns:
            List of association dictionaries, each containing:
                - 'detection_idx': index of YOLO detection
                - 'cluster_idx': index of matched cluster (or None if no match)
                - 'iou': IoU value (or 0.0 if no match)
                - 'cluster': matched cluster dict (or None)
        """
        if len(clusters) == 0:
            return []

        yolo_boxes = yolo_result.boxes
        num_detections = len(yolo_boxes)

        # Initialize variables for both code paths
        yolo_masks = None
        xywh_all = None
        cluster_bboxes_2d = None

        # Check if segmentation masks are available
        has_masks = (
            hasattr(yolo_result, "masks")
            and yolo_result.masks is not None
            and len(yolo_result.masks) > 0
        )

        if has_masks:
            self.get_logger().debug("Using segmentation masks for matching")
            # Get YOLO masks: shape [N, H, W] where N is number of detections
            yolo_masks = yolo_result.masks.data.detach().cpu().numpy()
            # Resize masks to image dimensions if needed
            if (
                yolo_masks.shape[1] != image_height
                or yolo_masks.shape[2] != image_width
            ):
                resized_masks = []
                for mask in yolo_masks:
                    mask_resized = cv2.resize(
                        mask.astype(np.float32),
                        (image_width, image_height),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    resized_masks.append(mask_resized > 0.5)  # Binarize
                yolo_masks = np.array(resized_masks, dtype=np.uint8)
            else:
                yolo_masks = (yolo_masks > 0.5).astype(np.uint8)
        else:
            self.get_logger().debug(
                "Masks not available, falling back to bounding boxes"
            )
            # Fallback to bounding boxes
            xywh_all = yolo_boxes.xywh.detach().cpu().numpy()
            cluster_bboxes_2d = []
            for cluster in clusters:
                bbox_2d = self._calculate_cluster_bbox_2d(cluster, None)  # Use cached
                cluster_bboxes_2d.append(bbox_2d)

        # Create cluster masks for all clusters
        cluster_masks = []
        for cluster in clusters:
            cluster_mask = self._create_cluster_mask(
                cluster, intrinsics, image_width, image_height
            )
            cluster_masks.append(cluster_mask)

        associations = []

        # For each YOLO detection, find best matching cluster
        for det_idx in range(num_detections):
            best_iou = 0.0
            best_cluster_idx = None
            best_cluster = None

            if has_masks and yolo_masks is not None:
                # Use mask-based IoU
                yolo_mask = yolo_masks[det_idx]

                for cluster_idx, cluster_mask in enumerate(cluster_masks):
                    if cluster_mask is None:
                        continue

                    iou = self._calculate_mask_iou(yolo_mask, cluster_mask)
                    if iou > best_iou and iou >= iou_threshold:
                        best_iou = iou
                        best_cluster_idx = cluster_idx
                        best_cluster = clusters[cluster_idx]
            elif xywh_all is not None and cluster_bboxes_2d is not None:
                # Fallback to bounding box IoU
                cx, cy, w, h = xywh_all[det_idx]
                det_min_u = cx - w / 2
                det_min_v = cy - h / 2
                det_max_u = cx + w / 2
                det_max_v = cy + h / 2
                det_bbox = (det_min_u, det_min_v, det_max_u, det_max_v)

                for cluster_idx, cluster_bbox_2d in enumerate(cluster_bboxes_2d):
                    if cluster_bbox_2d is None:
                        continue

                    iou = self._calculate_iou(det_bbox, cluster_bbox_2d)
                    if iou > best_iou and iou >= iou_threshold:
                        best_iou = iou
                        best_cluster_idx = cluster_idx
                        best_cluster = clusters[cluster_idx]

            associations.append(
                {
                    "detection_idx": det_idx,
                    "cluster_idx": best_cluster_idx,
                    "iou": best_iou,
                    "cluster": best_cluster,
                }
            )

        return associations

    def _process_depth_image(
        self, depth_msg: Image
    ) -> tuple[np.ndarray, list[dict[str, Any]]]:
        """Process depth image: convert to point cloud and cluster.

        Args:
            depth_msg: Depth image message.

        Returns:
            Tuple of (point_cloud, clusters) where:
                - point_cloud: numpy array (N x 3) of all valid 3D points
                - clusters: list of cluster dictionaries
        """
        # Check if camera intrinsics are available
        if self.camera_intrinsics is None:
            self.get_logger().warn(
                "Camera intrinsics not available for depth processing"
            )
            return np.array([]).reshape(0, 3), []

        # Convert depth image to numpy array
        # Depth images are typically 16UC1 (uint16, millimeters) or
        # 32FC1 (float32, meters)
        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
            self.get_logger().debug(
                f"Depth image converted: {depth_image.shape}, dtype={depth_image.dtype}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return np.array([]).reshape(0, 3), []

        # Handle different encodings
        if depth_image.dtype == np.uint16:
            # Convert from millimeters to meters
            depth_image = depth_image.astype(np.float32) / 1000.0
        elif depth_image.dtype != np.float32:
            depth_image = depth_image.astype(np.float32)

        # Convert to point cloud
        point_cloud = self._depth_to_point_cloud(depth_image, self.camera_intrinsics)
        self.get_logger().debug(f"Point cloud generated: {len(point_cloud)} points")

        # Cluster points only if clustering is enabled
        if self.use_clustering:
            # Downsample point cloud if too large (for faster clustering)
            # Keep every Nth point to reduce computation time
            if len(point_cloud) > 50000:
                downsample_factor = max(1, len(point_cloud) // 50000)
                point_cloud = point_cloud[::downsample_factor]
                self.get_logger().info(
                    f"Downsampled point cloud to {len(point_cloud)} points "
                    f"(factor: {downsample_factor})"
                )

            # Cluster points
            self.get_logger().info("Starting DBSCAN clustering...")
            clusters = self._cluster_depth_points(point_cloud)
            self.get_logger().info(f"Clustering complete: {len(clusters)} clusters")
        else:
            # Clustering disabled, return empty clusters
            clusters = []
            self.get_logger().debug("Clustering disabled, skipping DBSCAN")

        return point_cloud, clusters

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

            # Run YOLO inference and depth processing in parallel
            loop = asyncio.get_event_loop()

            # Start YOLO inference task
            self.get_logger().debug("Starting YOLO inference...")
            yolo_task = loop.run_in_executor(self._yolo_executor, self.model, rgb_image)

            # Start depth processing task if depth data is available
            depth_task = None
            if depth_msg is not None and self.use_depth:
                self.get_logger().info("Starting depth processing...")
                depth_task = loop.run_in_executor(
                    self._depth_executor, self._process_depth_image, depth_msg
                )

            # Wait for both tasks to complete (or just YOLO if no depth)
            try:
                if depth_task is not None:
                    self.get_logger().debug("Waiting for YOLO and depth processing...")
                    (results, (point_cloud, clusters)) = await asyncio.gather(
                        yolo_task, depth_task
                    )
                    self.get_logger().info(
                        f"Depth processing complete: {len(point_cloud)} points, "
                        f"{len(clusters)} clusters found"
                    )
                    if len(clusters) > 0:
                        cluster_sizes = [c["size"] for c in clusters]
                        self.get_logger().info(
                            f"Found {len(clusters)} depth cluster(s) with sizes: "
                            f"{cluster_sizes}"
                        )
                    else:
                        self.get_logger().info("No depth clusters found")
                else:
                    results = await yolo_task
                    point_cloud = None
                    clusters = []

                self.get_logger().debug("YOLO inference returned")
                result = results[0]
                num_detections = len(result.boxes) if hasattr(result, "boxes") else 0
                self.get_logger().debug(
                    f"YOLO inference complete: {num_detections} detections "
                    f"in {rgb_msg.width}x{rgb_msg.height} image"
                )
            except Exception as e:
                self.get_logger().error(f"Processing failed: {e}", exc_info=True)
                return

            # Phase 5: Match clusters to detections (before entity generation)
            associations = []
            if (
                depth_msg is not None
                and self.use_depth
                and clusters
                and num_detections > 0
                and self.use_clustering
                and self.camera_intrinsics is not None
            ):
                self.get_logger().debug(
                    f"Matching {len(clusters)} clusters to {num_detections} "
                    f"detections..."
                )
                associations = self._match_clusters_to_detections(
                    result,
                    clusters,
                    None,  # Use cached intrinsics
                    rgb_msg.width,
                    rgb_msg.height,
                )

                # Log association results
                matched_count = sum(
                    1 for a in associations if a["cluster_idx"] is not None
                )
                self.get_logger().info(
                    f"Association complete: {matched_count}/{num_detections} "
                    f"detections matched to clusters"
                )
                if matched_count > 0:
                    avg_iou = np.mean(
                        [a["iou"] for a in associations if a["cluster_idx"] is not None]
                    )
                    self.get_logger().debug(f"Average IoU: {avg_iou:.3f}")
            elif depth_msg is not None and self.use_depth and not self.use_clustering:
                self.get_logger().debug(
                    "Clustering disabled, skipping cluster-to-detection matching"
                )

            # Phase 6: Convert YOLO results to entity dicts with cluster associations
            # Uses cluster centroid for pose if available, otherwise falls back to BB
            self.get_logger().debug("Converting YOLO results to entities...")
            entities_cv = generate_entities_from_yolo_result(
                result=result,
                class_names=self.model.names,
                frame=None,  # Don't visualize during entity conversion
                use_segmentation=self.use_segmentation,
                cluster_associations=associations if associations else None,
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
                    pose = entity_cv["pose"]
                    # Log pose values for debugging (INFO level to verify Phase 6)
                    frame_id = rgb_msg.header.frame_id
                    self.get_logger().info(
                        f"Adding entity '{entity_cv['name'].data}' with pose: "
                        f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, "
                        f"z={pose.position.z:.3f} (ref_frame: {frame_id})"
                    )
                    add_entity_req = AddEntity.Request()
                    add_entity_req.data = Entity(
                        description=f"Detected: {entity_cv['name'].data}",
                        pose=pose,
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
        """Visualize YOLO detections on frame with segmentation masks.

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

        # Check if segmentation masks are available
        has_masks = (
            hasattr(result, "masks")
            and result.masks is not None
            and len(result.masks) > 0
        )

        if has_masks:
            # Get masks and resize to frame dimensions
            masks = result.masks.data.detach().cpu().numpy()
            frame_h, frame_w = frame.shape[:2]

            # Create overlay for masks
            overlay = frame.copy()

            for i in range(len(boxes)):
                # Get mask for this detection
                mask = masks[i]
                if mask.shape[0] != frame_h or mask.shape[1] != frame_w:
                    mask = cv2.resize(
                        mask.astype(np.float32),
                        (frame_w, frame_h),
                        interpolation=cv2.INTER_NEAREST,
                    )
                mask_binary = (mask > 0.5).astype(np.uint8)

                # Generate color for this detection
                color = self._get_color_for_class(class_ids[i])
                color_bgr = (int(color[2]), int(color[1]), int(color[0]))

                # Draw mask overlay
                overlay[mask_binary > 0] = (
                    overlay[mask_binary > 0] * 0.6 + np.array(color_bgr) * 0.4
                ).astype(np.uint8)

                # Draw bounding box
                cx, cy, w, h = xywh_all[i]
                x1 = int(cx - w / 2.0)
                y1 = int(cy - h / 2.0)
                x2 = int(cx + w / 2.0)
                y2 = int(cy + h / 2.0)

                cv2.rectangle(overlay, (x1, y1), (x2, y2), color_bgr, 2)

                # Draw label
                label = str(class_names[class_ids[i]])
                cv2.putText(
                    overlay,
                    label,
                    (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color_bgr,
                    2,
                )

            frame = overlay
        else:
            # Fallback to bounding boxes only
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

    def _get_color_for_class(self, class_id: int) -> tuple[int, int, int]:
        """Generate a consistent color for a class ID.

        Args:
            class_id: Class index.

        Returns:
            RGB color tuple.
        """
        # Use a color palette that provides good contrast
        colors = [
            (255, 0, 0),  # Red
            (0, 255, 0),  # Green
            (0, 0, 255),  # Blue
            (255, 255, 0),  # Yellow
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Cyan
            (128, 0, 128),  # Purple
            (255, 165, 0),  # Orange
        ]
        return colors[class_id % len(colors)]


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


def pose_from_point3d(point_3d: np.ndarray) -> Pose:
    """Convert a 3D point (cluster centroid) to a Pose.

    Args:
        point_3d: numpy array [x, y, z] in camera frame.

    Returns:
        Pose: Pose with position from centroid and identity orientation.
    """
    return Pose(
        position=Point(
            x=float(point_3d[0]), y=float(point_3d[1]), z=float(point_3d[2])
        ),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def generate_entities_from_yolo_result(
    result,
    class_names,
    frame: np.ndarray | None = None,
    use_segmentation: bool = False,
    cluster_associations: list[dict[str, Any]] | None = None,
) -> list[dict]:
    """Convert a YOLO result into a list of entity dicts.

    Builds minimal entities used by this node:
        - `name` (std_msgs/String)
        - `pose` (geometry_msgs/Pose from cluster centroid if available,
                  otherwise from bbox center)

    Args:
        result: Ultralytics YOLO result for one image.
        class_names: Mapping from class indices to names (list or dict).
        frame: Optional RGB frame for visualization overlay.
        use_segmentation: Whether the model is a segmentation model.
        cluster_associations: Optional list of cluster associations from
            _match_clusters_to_detections(). Each dict contains:
            - 'detection_idx': index of YOLO detection
            - 'cluster_idx': index of matched cluster (or None)
            - 'cluster': matched cluster dict with 'centroid' (or None)

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

        # Use cluster centroid if available, otherwise fallback to BB center
        if cluster_associations and i < len(cluster_associations):
            assoc = cluster_associations[i]
            if assoc.get("cluster") is not None:
                # Use 3D cluster centroid for pose
                centroid = assoc["cluster"]["centroid"]  # numpy array [x, y, z]
                pose_msg = pose_from_point3d(centroid)
                # Log 3D pose for debugging
                if __debug__:
                    import logging

                    logger = logging.getLogger("ObjectDetection")
                    logger.debug(
                        f"Entity '{label}' using 3D cluster centroid: "
                        f"({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f})"
                    )
            else:
                # Fallback to 2D bounding box center
                pose_msg = pose_from_point2d(Point2D(x=float(cx), y=float(cy)))
        else:
            # No associations available, use 2D bounding box center
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
