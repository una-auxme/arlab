"""ROS2 YOLO-based object detection node with KB integration.

This module defines a ROS2 node that performs real-time object detection
using Ultralytics YOLO segmentation, converts results into
semantic/geometric entities, and communicates with a knowledge base (KB)
via ROS services.


Maintainers:
    Aleksander Michalak <aleksander1.michalak@uni-a.de>
"""

import asyncio
import os
import threading
import time
from typing import Any

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from arlab_asyncio_executor.executors import AsyncIOExecutor
from arlab_knowledge_interfaces.msg import Entity, Shape
from arlab_knowledge_interfaces.srv import AddEntity, DelEntities, GetEntities, UpdShape
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header, String  # <-- needed for 'name' field in entities
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Point2D, Pose2D


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
        client_upd_shape: Service client for updating entity shapes.
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
        use_clustering_param = self.get_parameter(
            "use_clustering"
        ).get_parameter_value()
        # Handle both bool and string values (ROS2 parameter parsing issue)
        if isinstance(use_clustering_param.bool_value, bool):
            self.use_clustering = use_clustering_param.bool_value
        else:
            # Try to parse as string (workaround for ROS2 parameter parsing)
            param_str = str(use_clustering_param.bool_value).lower()
            self.use_clustering = param_str in ("true", "1", "yes", "on")
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
            # Warmup the model to avoid first inference delay
            try:
                import numpy as np

                dummy_input = np.zeros((480, 640, 3), dtype=np.uint8)
                _ = self.model(dummy_input, verbose=False)
                if torch.cuda.is_available():
                    torch.cuda.synchronize()  # Wait for warmup to complete
                self.get_logger().info("Model warmup completed")
            except Exception as e:
                self.get_logger().warn(f"Model warmup failed: {e}")

        # Visualization removed - no need to close windows

        # Flag to track if a frame is currently being processed
        self._processing_frame = False
        # Thread-safe lock for frame processing (synchron)
        self._processing_lock_sync = threading.Lock()
        # Async lock for frame processing (async)
        self._processing_lock_async = None  # Will be created in async_init

        # Frame statistics
        self._frames_processed = 0
        self._frames_skipped = 0

        # Thread pool removed - using asyncio.to_thread for sequential processing

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
        self.client_upd_shape = self.create_client(
            UpdShape,
            f"{self.prefix}/upd_shape",
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
        # Create async lock for frame processing
        self._processing_lock_async = asyncio.Lock()

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
        # Early frame skipping: Check if already processing BEFORE creating task
        with self._processing_lock_sync:
            if self._processing_frame:
                self._frames_skipped += 1
                return

        try:
            asyncio.get_running_loop()
            asyncio.create_task(self.process_data(rgb_msg, depth_msg))
        except RuntimeError:
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

    def _filter_depth_by_masks(
        self,
        depth_image: np.ndarray,
        masks: np.ndarray,
        intrinsics: dict[str, float],
    ) -> np.ndarray:
        """Filter depth points using segmentation masks.

        Only depth points that lie within at least one segmentation mask
        are kept. Points in overlapping mask regions are kept once.

        Args:
            depth_image: Depth image as numpy array (H x W), values in meters.
            masks: Binary masks as numpy array (N x H x W) where N is number of
                detections. Each mask is a binary array (0 or 1).
            intrinsics: Camera intrinsics dict with keys: fx, fy, cx, cy.

        Returns:
            Filtered point cloud as numpy array (M x 3) where each row is [x, y, z]
            in camera frame. Only points within mask regions are included.
            Invalid points (depth=0 or NaN) are filtered out.
        """
        height, width = depth_image.shape

        # Validate mask dimensions
        if len(masks) == 0:
            return np.array([]).reshape(0, 3)

        # Check if masks match image dimensions
        if masks.shape[1] != height or masks.shape[2] != width:
            self.get_logger().warn(
                f"Mask dimensions {masks.shape[1]}x{masks.shape[2]} do not match "
                f"depth image dimensions {height}x{width}"
            )
            return np.array([]).reshape(0, 3)

        # Create combined mask: OR operation across all masks
        # A point is kept if it's in ANY mask
        combined_mask = np.zeros((height, width), dtype=bool)
        for mask in masks:
            # Ensure mask is boolean
            mask_bool = mask.astype(bool)
            combined_mask = combined_mask | mask_bool

        # Extract camera intrinsics
        fx = intrinsics["fx"]
        fy = intrinsics["fy"]
        cx = intrinsics["cx"]
        cy = intrinsics["cy"]

        # Create pixel coordinate grids
        u, v = np.meshgrid(np.arange(width), np.arange(height))

        # Convert depth to float32
        z = depth_image.astype(np.float32)

        # Filter: valid depth AND within mask
        valid_depth_mask = (z > 0) & np.isfinite(z)
        final_mask = valid_depth_mask & combined_mask

        # Extract valid points
        u_valid = u[final_mask]
        v_valid = v[final_mask]
        z_valid = z[final_mask]

        # Convert to 3D coordinates
        x_valid = (u_valid - cx) * z_valid / fx
        y_valid = (v_valid - cy) * z_valid / fy

        # Stack into point cloud (N x 3)
        return np.column_stack([x_valid, y_valid, z_valid])

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

    def _process_depth_with_masks(
        self, depth_msg: Image, masks: np.ndarray | None
    ) -> tuple[np.ndarray, list[dict[str, Any]]]:
        """Process depth image with mask-based filtering before clustering.

        Args:
            depth_msg: Depth image message.
            masks: Binary masks as numpy array (N x H x W) or None if no masks.

        Returns:
            Tuple of (point_cloud, clusters) where:
                - point_cloud: numpy array (M x 3) of filtered 3D points
                - clusters: list of cluster dictionaries
        """
        # Check if camera intrinsics are available
        if self.camera_intrinsics is None:
            self.get_logger().warn(
                "Camera intrinsics not available for depth processing"
            )
            return np.array([]).reshape(0, 3), []

        # Convert depth image to numpy array
        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
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

        # Filter depth points using masks (if available)
        if masks is not None and len(masks) > 0:
            point_cloud = self._filter_depth_by_masks(
                depth_image, masks, self.camera_intrinsics
            )
        else:
            point_cloud = self._depth_to_point_cloud(
                depth_image, self.camera_intrinsics
            )

        # Cluster points only if clustering is enabled
        clusters = []
        if self.use_clustering and len(point_cloud) > 0:
            clusters = self._cluster_depth_points(point_cloud)

        return point_cloud, clusters

    def _associate_clusters_to_masks(
        self,
        clusters: list[dict[str, Any]],
        masks: np.ndarray,
        intrinsics: dict[str, float],
        image_width: int,
        image_height: int,
        iou_threshold: float = 0.1,
    ) -> list[dict[str, Any]]:
        """Associate clusters to detection masks using point overlap.

        Since clusters are created from mask-filtered points, we can directly
        associate clusters to masks by checking which mask contains the most
        cluster points. Returns detection-centric associations (one per detection).

        Args:
            clusters: List of cluster dictionaries.
            masks: Binary masks as numpy array (N x H x W).
            intrinsics: Camera intrinsics dict.
            image_width: Width of the image.
            image_height: Height of the image.
            iou_threshold: Minimum IoU for a valid match (default: 0.1).

        Returns:
            List of association dictionaries (one per detection), each containing:
                - 'detection_idx': index of YOLO detection
                - 'cluster_idx': index of matched cluster (or None if no match)
                - 'iou': IoU value (or 0.0 if no match)
                - 'cluster': matched cluster dict (or None)
        """
        if len(clusters) == 0 or len(masks) == 0:
            # Return empty associations for each detection
            return [
                {
                    "detection_idx": i,
                    "cluster_idx": None,
                    "iou": 0.0,
                    "cluster": None,
                }
                for i in range(len(masks))
            ]

        fx = intrinsics["fx"]
        fy = intrinsics["fy"]
        cx = intrinsics["cx"]
        cy = intrinsics["cy"]

        # Initialize associations: one per detection
        associations: list[dict[str, Any]] = [
            {
                "detection_idx": i,
                "cluster_idx": None,
                "iou": 0.0,
                "cluster": None,
            }
            for i in range(len(masks))
        ]

        # For each cluster, find the detection mask with highest point overlap
        for cluster_idx, cluster in enumerate(clusters):
            cluster_points = cluster["points"]

            # Project cluster points to 2D
            x, y, z = cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2]

            # Filter valid points (z > 0)
            valid_mask = z > 0
            if not np.any(valid_mask):
                continue

            # Project to 2D pixel coordinates
            u = (fx * x[valid_mask] / z[valid_mask] + cx).astype(int)
            v = (fy * y[valid_mask] / z[valid_mask] + cy).astype(int)

            # Filter points within image bounds
            in_bounds = (u >= 0) & (u < image_width) & (v >= 0) & (v < image_height)
            if not np.any(in_bounds):
                continue

            u_valid = u[in_bounds]
            v_valid = v[in_bounds]

            # For each detection mask, count how many cluster points fall within it
            best_detection_idx = None
            best_iou = 0.0
            best_overlap_count = 0

            for det_idx, mask in enumerate(masks):
                # Ensure mask dimensions match image dimensions
                if mask.shape[0] != image_height or mask.shape[1] != image_width:
                    continue
                # Count points that fall within this mask
                mask_bool = mask.astype(bool)
                # Use safe indexing to prevent IndexError
                # Points are already filtered to be within image bounds,
                # but double-check for safety
                safe_mask = (
                    (v_valid >= 0)
                    & (v_valid < image_height)
                    & (u_valid >= 0)
                    & (u_valid < image_width)
                )
                if np.any(safe_mask):
                    points_in_mask = np.sum(
                        mask_bool[v_valid[safe_mask], u_valid[safe_mask]]
                    )
                else:
                    points_in_mask = 0

                if points_in_mask > best_overlap_count:
                    best_overlap_count = points_in_mask
                    best_detection_idx = det_idx

            # Calculate IoU if we found a match
            if best_detection_idx is not None and best_overlap_count > 0:
                # Create cluster mask for IoU calculation
                cluster_mask = self._create_cluster_mask(
                    cluster, intrinsics, image_width, image_height
                )
                if cluster_mask is not None:
                    yolo_mask = masks[best_detection_idx].astype(np.uint8)
                    best_iou = self._calculate_mask_iou(cluster_mask, yolo_mask)

                    # Only keep match if IoU is above threshold
                    if best_iou >= iou_threshold:
                        # Update association for this detection if this cluster
                        # is better
                        current_assoc = associations[best_detection_idx]
                        current_iou = current_assoc.get("iou", 0.0)
                        if (
                            current_assoc.get("cluster_idx") is None
                            or best_iou > current_iou
                        ):
                            assoc_dict: dict[str, Any] = {
                                "detection_idx": best_detection_idx,
                                "cluster_idx": cluster_idx,
                                "iou": best_iou,
                                "cluster": cluster,
                            }
                            associations[best_detection_idx] = assoc_dict

        return associations

    def _synced_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        """Callback for synchronized RGB and depth image messages.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.
            depth_msg: Incoming depth `sensor_msgs/Image`.
        """
        self._process_data_sync(rgb_msg, depth_msg)

    async def _run_yolo_inference(self, rgb_image: np.ndarray) -> tuple[Any, int]:
        """Run YOLO inference on RGB image.

        Args:
            rgb_image: RGB image as numpy array.

        Returns:
            Tuple of (YOLO result, number of detections).
        """
        results = await asyncio.to_thread(self.model, rgb_image, verbose=False)
        result = results[0]
        num_detections = len(result.boxes) if hasattr(result, "boxes") else 0
        return result, num_detections

    def _extract_masks(
        self, result: Any, image_height: int, image_width: int
    ) -> np.ndarray | None:
        """Extract segmentation masks from YOLO result.

        Args:
            result: YOLO result object.
            image_height: Height of the image.
            image_width: Width of the image.

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

        masks_raw = result.masks.data.detach().cpu().numpy()
        if masks_raw.shape[1] != image_height or masks_raw.shape[2] != image_width:
            masks_resized = []
            for mask in masks_raw:
                mask_tensor = torch.from_numpy(mask).unsqueeze(0)
                mask_resized = torch.nn.functional.interpolate(
                    mask_tensor.unsqueeze(0),
                    size=(image_height, image_width),
                    mode="bilinear",
                    align_corners=False,
                ).squeeze()
                masks_resized.append(mask_resized.numpy() > 0.5)
            return np.array(masks_resized, dtype=np.uint8)
        else:
            return (masks_raw > 0.5).astype(np.uint8)

    def _should_associate_clusters(
        self,
        depth_msg: Image | None,
        clusters: list[dict[str, Any]],
        masks: np.ndarray | None,
        num_detections: int,
    ) -> bool:
        """Check if cluster association should be performed.

        Args:
            depth_msg: Optional depth image message.
            clusters: List of cluster dictionaries.
            masks: Optional binary masks.
            num_detections: Number of YOLO detections.

        Returns:
            True if association should be performed, False otherwise.
        """
        if depth_msg is None or not self.use_depth:
            return False
        if not clusters:
            return False
        if masks is None or len(masks) == 0:
            return False
        if num_detections == 0:
            return False
        if not self.use_clustering:
            return False
        if self.camera_intrinsics is None:
            return False
        return True

    async def _update_knowledge_base(self, entities: list[dict], frame_id: str) -> None:
        """Update knowledge base with detected entities.

        Args:
            entities: List of entity dictionaries.
            frame_id: Reference frame ID.
        """
        if not self._kb_services_available:
            return

        # Verify services are still available
        service_ready = (
            self.client_get_entities.service_is_ready()
            and self.client_add_entities.service_is_ready()
            and self.client_del_entities.service_is_ready()
        )
        if not service_ready:
            self._kb_services_available = False
            self._start_periodic_service_check()
            return

        if len(entities) == 0:
            return

        now = self.get_clock().now().to_msg()
        KB_TIMEOUT = 2.0

        # Add entities sequentially
        add_responses = []
        for entity in entities:
            add_entity_req = AddEntity.Request()
            add_entity_req.data = Entity(
                description=f"Detected: {entity['name'].data}",
                pose=entity["pose"],
                pose_reference_frame=frame_id,
                stamp=now,
                reference_frame="camera_link",
            )
            try:
                response = await asyncio.wait_for(
                    self.client_add_entities.call_async(add_entity_req),
                    timeout=KB_TIMEOUT,
                )
                add_responses.append(response)
            except (asyncio.TimeoutError, Exception):
                add_responses.append(None)

        # Update entities with point clouds and bounding boxes sequentially
        for i, entity in enumerate(entities):
            # Skip if no shape data to update
            if entity.get("pointcloud") is None and entity.get("boundingbox2d") is None:
                continue
            if i >= len(add_responses) or add_responses[i] is None:
                continue
            if not hasattr(add_responses[i], "entityid"):
                continue
            entity_id = getattr(add_responses[i], "entityid", None)
            if entity_id is None:
                continue

            shape_msg = Shape()
            if entity.get("pointcloud") is not None:
                shape_msg.has_pointcloud = True
                shape_msg.pointcloud = entity["pointcloud"]
            else:
                shape_msg.has_pointcloud = False

            if entity.get("boundingbox2d") is not None:
                shape_msg.has_boundingbox2d = True
                shape_msg.boundingbox2d = entity["boundingbox2d"]
            else:
                shape_msg.has_boundingbox2d = False

            upd_shape_req = UpdShape.Request()
            upd_shape_req.entityid = entity_id
            upd_shape_req.shape = shape_msg
            upd_shape_req.stamp = now

            try:
                await asyncio.wait_for(
                    self.client_upd_shape.call_async(upd_shape_req),
                    timeout=KB_TIMEOUT,
                )
            except (asyncio.TimeoutError, Exception):
                pass  # Continue with next entity

    async def process_data(
        self, rgb_msg: Image, depth_msg: Image | None = None
    ) -> None:
        """Process incoming RGB images and sync detections with KB.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.
            depth_msg: Optional incoming depth `sensor_msgs/Image`.
        """
        if not self._camera_intrinsics_set:
            return

        if self._processing_lock_async is None:
            return

        async with self._processing_lock_async:
            if self._processing_frame:
                self._frames_skipped += 1
                return
            self._processing_frame = True
            self._frames_processed += 1

            # Start timer for total processing time
            t_start = time.perf_counter()

            try:
                # Convert ROS image to numpy array (BGR to RGB)
                bgr_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
                rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
                self.get_logger().info(
                    f"Image received: {rgb_msg.width}x{rgb_msg.height}"
                )

                # Run YOLO inference
                result, num_detections = await self._run_yolo_inference(rgb_image)
                if num_detections > 0:
                    self.get_logger().info(f"{num_detections} object(s) detected")

                # Extract segmentation masks
                masks = self._extract_masks(result, rgb_msg.height, rgb_msg.width)

                # Process depth data with mask-based filtering
                clusters = []
                if depth_msg is not None and self.use_depth:
                    try:
                        _, clusters = await asyncio.to_thread(
                            self._process_depth_with_masks,
                            depth_msg,
                            masks,
                        )
                        if len(clusters) > 0:
                            self.get_logger().info(
                                f"{len(clusters)} cluster(s) detected"
                            )
                    except Exception as e:
                        self.get_logger().error(
                            f"Depth processing failed: {e}", exc_info=True
                        )
                        clusters = []

                # Associate clusters to detections using masks
                associations = []
                if (
                    self._should_associate_clusters(
                        depth_msg, clusters, masks, num_detections
                    )
                    and masks is not None
                    and self.camera_intrinsics is not None
                ):
                    associations = self._associate_clusters_to_masks(
                        clusters,
                        masks,
                        self.camera_intrinsics,
                        rgb_msg.width,
                        rgb_msg.height,
                    )

                # Generate entities from YOLO results
                boxes = getattr(result, "boxes", None)
                if boxes is None or len(boxes) == 0:
                    entities = []
                else:
                    entities = generate_entities_from_yolo_result(
                        result=result,
                        class_names=self.model.names,
                        frame=None,
                        use_segmentation=self.use_segmentation,
                        cluster_associations=associations if associations else None,
                        frame_id=rgb_msg.header.frame_id,
                        timestamp=rgb_msg.header.stamp,
                    )
                    if len(entities) > 0:
                        entity_names = [e["name"].data for e in entities]
                        entity_str = f"{len(entities)} entit(y/ies) created: "
                        entity_str += ", ".join(entity_names)
                        self.get_logger().info(entity_str)

                # Update knowledge base
                if len(entities) > 0:
                    await self._update_knowledge_base(entities, rgb_msg.header.frame_id)
                    t_total = (time.perf_counter() - t_start) * 1000
                    self.get_logger().info(
                        f"{len(entities)} entit(y/ies) saved in KB "
                        f"(total processing time: {t_total:.2f}ms)"
                    )
                else:
                    t_total = (time.perf_counter() - t_start) * 1000
                    self.get_logger().info(
                        f"Frame processed (no entities) "
                        f"(total processing time: {t_total:.2f}ms)"
                    )
            finally:
                # Always reset processing flag, even if error occurred
                self._processing_frame = False

    # Visualization functions removed for performance reasons


def voxel_downsample(
    points: np.ndarray,  # Shape: [N, 3] - x, y, z coordinates
    voxel_size: float = 0.01,  # 1cm voxel size
    max_points: int = 10000,  # Maximum number of points after downsampling
) -> np.ndarray:
    """Downsample point cloud using voxel grid method.

    This method preserves the shape better than random sampling by averaging
    points within each voxel. The voxel size is automatically adjusted if
    the result would exceed max_points.

    Args:
        points: numpy array of shape [N, 3] with x, y, z coordinates.
        voxel_size: Size of each voxel in meters (default: 1cm).
        max_points: Maximum number of points after downsampling.

    Returns:
        Downsampled numpy array of shape [M, 3] where M <= max_points.
    """
    if len(points) == 0:
        return points

    if len(points) <= max_points:
        # No downsampling needed
        return points

    # Calculate bounding box
    min_bounds = points.min(axis=0)
    max_bounds = points.max(axis=0)

    # Adjust voxel size if needed to meet max_points constraint
    # Estimate: if we have N points and want max_points, voxel_size should be
    # approximately (N/max_points)^(1/3) times the current size
    if len(points) > max_points:
        scale_factor = (len(points) / max_points) ** (1.0 / 3.0)
        voxel_size = voxel_size * scale_factor

    # Calculate number of voxels in each dimension
    voxel_counts = ((max_bounds - min_bounds) / voxel_size).astype(int) + 1

    # Assign each point to a voxel
    voxel_indices = ((points - min_bounds) / voxel_size).astype(int)
    # Clamp to valid range
    voxel_indices = np.clip(voxel_indices, 0, voxel_counts - 1)

    # Create unique voxel keys (flatten 3D voxel indices to 1D)
    # Use a large prime number to avoid collisions
    voxel_keys = (
        voxel_indices[:, 0] * 73856093
        + voxel_indices[:, 1] * 19349663
        + voxel_indices[:, 2] * 83492791
    )

    # Group points by voxel and compute centroids
    unique_keys, inverse_indices = np.unique(voxel_keys, return_inverse=True)

    # Compute centroid for each voxel
    downsampled_points = np.zeros((len(unique_keys), 3), dtype=np.float32)
    for i, key in enumerate(unique_keys):
        mask = inverse_indices == i
        downsampled_points[i] = points[mask].mean(axis=0)

    # If still too many points, apply random sampling as fallback
    if len(downsampled_points) > max_points:
        indices = np.random.choice(len(downsampled_points), max_points, replace=False)
        downsampled_points = downsampled_points[indices]

    return downsampled_points


def numpy_to_pointcloud2(
    points: np.ndarray,  # Shape: [N, 3] - x, y, z coordinates
    frame_id: str,
    timestamp=None,
) -> PointCloud2:
    """Convert numpy point cloud array to sensor_msgs/PointCloud2.

    Args:
        points: numpy array of shape [N, 3] with x, y, z coordinates.
        frame_id: Reference frame for the point cloud.
        timestamp: Optional timestamp (builtin_interfaces/Time). If None,
            uses current time.

    Returns:
        sensor_msgs/PointCloud2 message.
    """
    if len(points) == 0:
        # Return empty point cloud
        pc2 = PointCloud2()
        pc2.header = Header()
        pc2.header.frame_id = frame_id
        if timestamp:
            pc2.header.stamp = timestamp
        pc2.height = 1
        pc2.width = 0
        pc2.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc2.point_step = 12
        pc2.row_step = 0
        pc2.data = bytes()
        pc2.is_bigendian = False
        pc2.is_dense = True
        return pc2

    # Create PointCloud2 message
    pc2 = PointCloud2()

    # Set header
    pc2.header = Header()
    pc2.header.frame_id = frame_id
    if timestamp:
        pc2.header.stamp = timestamp

    # Set dimensions
    pc2.height = 1  # Unorganized point cloud
    pc2.width = len(points)

    # Define point fields (x, y, z)
    pc2.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # Set point step (12 bytes: 3 floats * 4 bytes each)
    pc2.point_step = 12
    pc2.row_step = pc2.point_step * pc2.width

    # Convert numpy array to bytes (optimized: ensure float32 and contiguous)
    if points.dtype != np.float32:
        points = points.astype(np.float32, copy=False)
    if not points.flags["C_CONTIGUOUS"]:
        points = np.ascontiguousarray(points)
    # Use memoryview for faster conversion (if available)
    pc2.data = points.tobytes()

    # Set flags
    pc2.is_bigendian = False
    pc2.is_dense = True  # Assume no invalid points

    return pc2


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
    frame_id: str | None = None,
    timestamp=None,
) -> list[dict]:
    """Convert a YOLO result into a list of entity dicts.

    Builds minimal entities used by this node:
        - `name` (std_msgs/String)
        - `pose` (geometry_msgs/Pose from cluster centroid if available,
                  otherwise from bbox center)
        - `boundingbox2d` (vision_msgs/BoundingBox2D from YOLO output)
        - `pointcloud` (Optional sensor_msgs/PointCloud2 if cluster available)

    Args:
        result: Ultralytics YOLO result for one image.
        class_names: Mapping from class indices to names (list or dict).
        frame: Optional RGB frame for visualization overlay.
        use_segmentation: Whether the model is a segmentation model.
        cluster_associations: Optional list of cluster associations from
            _associate_clusters_to_masks(). Each dict contains:
            - 'detection_idx': index of YOLO detection
            - 'cluster_idx': index of matched cluster (or None)
            - 'cluster': matched cluster dict with 'centroid' and 'points' (or None)
        frame_id: Optional frame ID for point cloud header.
        timestamp: Optional timestamp for point cloud header.

    Returns:
        list[dict]: Each dict has keys `name`, `pose`, and optionally `pointcloud`.
    """
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return []

    # Get all boxes at once - works for both detection and segmentation
    # Ensure GPU operations are complete before copying to CPU (prevents blocking)
    if boxes.xywh.is_cuda:
        import torch

        torch.cuda.synchronize()
    xywh_all = boxes.xywh.detach().cpu().numpy()  # Shape: [N, 4]
    class_ids = boxes.cls.detach().cpu().numpy().astype(int)  # Shape: [N]
    entities: list[dict] = []

    for i in range(len(boxes)):
        cx, cy, w, h = xywh_all[i]

        label = str(class_names[class_ids[i]])
        name_msg = String(data=label)

        # Create 2D bounding box from YOLO output
        bbox2d = BoundingBox2D()
        bbox2d.center = Pose2D(x=float(cx), y=float(cy), theta=0.0)
        bbox2d.size_x = float(w)
        bbox2d.size_y = float(h)

        # Use cluster centroid if available, otherwise fallback to BB center
        pointcloud = None
        if cluster_associations and i < len(cluster_associations):
            assoc = cluster_associations[i]
            if assoc.get("cluster") is not None:
                cluster = assoc["cluster"]
                # Use 3D cluster centroid for pose
                centroid = cluster["centroid"]  # numpy array [x, y, z]
                pose_msg = pose_from_point3d(centroid)

                # Extract point cloud if available
                points = cluster.get("points")  # numpy array [N, 3]
                if points is not None and len(points) > 0 and frame_id:
                    # Apply intelligent downsampling to preserve shape
                    points = voxel_downsample(points, voxel_size=0.01, max_points=10000)
                    pointcloud = numpy_to_pointcloud2(
                        points, frame_id=frame_id, timestamp=timestamp
                    )
            else:
                # Fallback to 2D bounding box center
                pose_msg = pose_from_point2d(Point2D(x=float(cx), y=float(cy)))
        else:
            # No associations available, use 2D bounding box center
            pose_msg = pose_from_point2d(Point2D(x=float(cx), y=float(cy)))

        entity_dict: dict[str, Any] = {
            "name": name_msg,
            "pose": pose_msg,
            "boundingbox2d": bbox2d,
        }
        if pointcloud is not None:
            entity_dict["pointcloud"] = pointcloud

        entities.append(entity_dict)

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
        # Visualization removed - no need to close windows
        rclpy.shutdown()


if __name__ == "__main__":
    main()
