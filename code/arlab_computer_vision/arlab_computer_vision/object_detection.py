"""ROS2 YOLO-based object detection node with KB integration.

This module defines a ROS2 node that performs snapshot-based object detection
using Ultralytics YOLO segmentation. The node provides a predict_snapshot
function for on-demand processing of RGB+Depth images, converts results into
semantic/geometric entities, and communicates with a knowledge base (KB)
via ROS services.


Maintainers:
    Aleksander Michalak <aleksander1.michalak@uni-a.de>
"""

import asyncio
import os
import time
from typing import Any

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_sensor_msgs
import torch
from ament_index_python.packages import get_package_share_directory
from arlab_asyncio_executor.executors import AsyncIOExecutor
from arlab_common_interfaces.action import PredictSnapshot
from arlab_knowledge_interfaces.msg import Entity, Result, Shape
from arlab_knowledge_interfaces.srv import AddEntity, DelEntities, GetEntities, UpdShape
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.action.server import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header, String  # <-- needed for 'name' field in entities
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Point2D, Pose2D


class ObjectDetection(Node):
    """ROS2 node for snapshot-based object detection and KB synchronization.

    The node:
        1) Loads and initializes YOLO model during startup (model stays loaded).
        2) Subscribes to camera_info topic to receive camera intrinsics.
        3) Provides predict_snapshot() method for on-demand processing of
           RGB+Depth image snapshots.
        4) Runs YOLO segmentation on provided snapshots.
        5) Converts detections to semantic entities (label, bbox, point cloud,
           pose).
        6) Interacts with a knowledge base through ROS services to insert or
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
        """Initialize the node, parameters, and service clients.

        Loads YOLO model, sets up camera_info subscription for intrinsics,
        and initializes knowledge base service clients. The model remains
        loaded and ready for snapshot processing via predict_snapshot().
        """
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
        self.declare_parameter("sync_tolerance", 0.1)  # 500ms tolerance (default)
        # Enable/disable depth clustering (default: False for better performance)
        self.declare_parameter("use_clustering", True)
        # Processing timeout (0.0 = disabled, use _processing_frame flag only)
        self.declare_parameter("processing_timeout", 0.0)
        # Delete old entities before adding new ones
        self.declare_parameter("delete_old_entities", True)
        # Maximum number of points per entity point cloud
        # (0 = no limit, not recommended)
        self.declare_parameter("point_cloud_max_points", 0)
        # Maximum image width for YOLO inference (0 = no scaling)
        # Reduces memory usage and speeds up inference for large images
        self.declare_parameter("max_image_width", 640)
        # Target frame for TF transformations (default: "world")
        # If frame doesn't exist, entities will be saved in source frame
        self.declare_parameter("target_frame", "world")

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

        # Load processing timeout parameter (0.0 = disabled)
        self._processing_timeout = (
            self.get_parameter("processing_timeout").get_parameter_value().double_value
        )
        if self._processing_timeout > 0.0:
            self.get_logger().info(
                f"Processing timeout set to {self._processing_timeout}s"
            )
        else:
            self.get_logger().info(
                "Processing timeout disabled (using _processing_frame flag only)"
            )

        # Load delete old entities parameter
        self._delete_old_entities = (
            self.get_parameter("delete_old_entities").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Delete old entities: {self._delete_old_entities}")

        # Load point cloud max points parameter
        self.point_cloud_max_points = (
            self.get_parameter("point_cloud_max_points")
            .get_parameter_value()
            .integer_value
        )
        self.get_logger().info(
            f"Point cloud max points per entity: {self.point_cloud_max_points}"
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
        # Store device - will be used during inference
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
                # Set model to use GPU (Ultralytics handles this internally)
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
                # Warmup the model to avoid first inference delay
                # Use actual scaled image size to pre-compile CUDA kernels
                try:
                    import numpy as np

                    # Calculate warmup image size based on max_image_width
                    # This matches the actual scaled image size used in inference
                    # (640x640 square)
                    if self.max_image_width > 0:
                        # Use square size matching YOLO input (640x640)
                        warmup_size = (
                            self.max_image_width,
                            self.max_image_width,
                            3,
                        )
                    else:
                        # Fallback to typical size
                        warmup_size = (640, 640, 3)

                    self.get_logger().debug(
                        f"Warmup image size: {warmup_size[1]}x{warmup_size[0]}"
                    )

                    dummy_input = np.zeros(warmup_size, dtype=np.uint8)
                    # Set device during warmup to load model on GPU initially
                    # After first call, model stays on this device
                    # Run warmup multiple times to compile all CUDA kernels
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

        # Visualization removed - no need to close windows

        # Flag to track if a frame is currently being processed
        self._processing_frame = False
        # Async lock for frame processing (async)
        self._processing_lock_async = None  # Will be created in async_init

        # Timeout for processing (tracks last processing time)
        # Note: Only used if processing_timeout > 0.0 (currently disabled)
        self._last_processing_time = 0.0

        # Thread pool removed - using asyncio.to_thread for sequential processing

        self.camera_intrinsics: dict[str, float] | None = None
        self._camera_intrinsics_set = False
        # Cached intrinsics values for faster access
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None

        # Track warnings to avoid spam
        self._warned_missing_intrinsics = False
        self._warned_missing_async_lock = False

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
        self.source_frame = "camera_tool_link"  # Source frame for transformations

        # Subscribe to camera info (async; sets intrinsics once available).
        self.create_subscription(
            CameraInfo,
            "camera_info",
            self.camera_info_callback,
            qos_profile=1,
        )
        self.get_logger().info("Subscribed to camera_info topic")

        # Create Action Server for predict_snapshot
        self._action_server = ActionServer(
            self,
            PredictSnapshot,
            "predict_snapshot",
            self._predict_snapshot_goal_handler,
            callback_group=self.service_client_group,
        )
        self.get_logger().info("Action server 'predict_snapshot' started")

        # Live image subscriptions removed - using snapshot-based prediction instead
        # Images will be provided via predict_snapshot function call or action

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

        This legacy helper keeps depth points that lie within at least one
        segmentation mask by OR-ing all masks together. It is still used for
        non-segmentation clustering paths, but mask-aware clustering is handled
        by `_cluster_depth_points_per_mask`.

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

    def _cluster_depth_points_per_mask(
        self,
        depth_image: np.ndarray,
        masks: np.ndarray,
        intrinsics: dict[str, float],
        eps: float = 0.05,
        min_samples: int = 15,
        max_distance: float = 3.0,
    ) -> tuple[np.ndarray, list[dict[str, Any]]]:
        """Cluster depth points separately for each segmentation mask.

        This function ensures that clusters are generated per YOLO segmentation
        mask instead of over a global OR of all masks. DBSCAN is then used per
        mask to remove spurious, isolated points (outliers) inside the mask
        region based on local density.

        Args:
            depth_image: Depth image as numpy array (H x W), values in meters.
            masks: Binary masks as numpy array (N x H x W).
            intrinsics: Camera intrinsics dict.
            eps: DBSCAN epsilon parameter (maximum distance between points
                in a cluster).
            min_samples: Minimum number of points to form a cluster.
            max_distance: Maximum distance from camera to consider (filter far points).

        Returns:
            Tuple of:
                - Combined point cloud of all masked points (M x 3).
                - List of cluster dictionaries, each containing:
                    - 'points': numpy array of cluster points (K x 3)
                    - 'centroid': numpy array [x, y, z]
                    - 'size': number of points
                    - 'bbox_3d': 3D bounding box
                    - 'detection_idx': index of the mask / YOLO detection
        """
        height, width = depth_image.shape

        if len(masks) == 0:
            return np.array([]).reshape(0, 3), []

        if masks.shape[1] != height or masks.shape[2] != width:
            self.get_logger().warn(
                f"Mask dimensions {masks.shape[1]}x{masks.shape[2]} do not match "
                f"depth image dimensions {height}x{width}"
            )
            return np.array([]).reshape(0, 3), []

        # Extract camera intrinsics
        fx = intrinsics["fx"]
        fy = intrinsics["fy"]
        cx = intrinsics["cx"]
        cy = intrinsics["cy"]

        # Create pixel coordinate grids once
        u_grid, v_grid = np.meshgrid(np.arange(width), np.arange(height))

        # Convert depth to float32 once
        z_full = depth_image.astype(np.float32)

        # Valid depth mask (shared across all masks)
        valid_depth_mask = (z_full > 0) & np.isfinite(z_full)

        all_points: list[np.ndarray] = []
        all_clusters: list[dict[str, Any]] = []

        for det_idx, mask in enumerate(masks):
            mask_bool = mask.astype(bool)
            # Points must be valid depth and inside this specific mask
            final_mask = valid_depth_mask & mask_bool
            if not np.any(final_mask):
                continue

            u_valid = u_grid[final_mask]
            v_valid = v_grid[final_mask]
            z_valid = z_full[final_mask]

            # Filter points by distance (remove points too far from camera)
            x_valid = (u_valid - cx) * z_valid / fx
            y_valid = (v_valid - cy) * z_valid / fy
            points = np.column_stack([x_valid, y_valid, z_valid])

            if points.shape[0] == 0:
                continue

            distances = np.linalg.norm(points, axis=1)
            distance_mask = distances <= max_distance
            filtered_points = points[distance_mask]

            if filtered_points.shape[0] < min_samples:
                # Not enough points to form a cluster for this mask
                continue

            # Apply DBSCAN clustering per mask
            clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(filtered_points)
            labels = clustering.labels_

            unique_labels = set(labels) - {-1}
            if not unique_labels:
                continue

            for label in unique_labels:
                cluster_mask = labels == label
                cluster_points = filtered_points[cluster_mask]
                if cluster_points.shape[0] < min_samples:
                    continue

                centroid = np.mean(cluster_points, axis=0)
                bbox_3d = {
                    "min_x": np.min(cluster_points[:, 0]),
                    "max_x": np.max(cluster_points[:, 0]),
                    "min_y": np.min(cluster_points[:, 1]),
                    "max_y": np.max(cluster_points[:, 1]),
                    "min_z": np.min(cluster_points[:, 2]),
                    "max_z": np.max(cluster_points[:, 2]),
                }

                all_clusters.append(
                    {
                        "points": cluster_points,
                        "centroid": centroid,
                        "size": cluster_points.shape[0],
                        "bbox_3d": bbox_3d,
                        "label": label,
                        "detection_idx": int(det_idx),
                    }
                )

            all_points.append(points)

        if all_points:
            combined_pc = np.vstack(all_points)
        else:
            combined_pc = np.array([]).reshape(0, 3)

        return combined_pc, all_clusters

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

        # Filter and cluster depth points.
        # If segmentation masks are available, cluster per mask to avoid
        # mixing nearby objects into a single global cluster.
        if masks is not None and len(masks) > 0 and self.use_clustering:
            point_cloud, clusters = self._cluster_depth_points_per_mask(
                depth_image, masks, self.camera_intrinsics
            )
        else:
            # Fallback: global point cloud and optional clustering
            point_cloud = self._depth_to_point_cloud(
                depth_image, self.camera_intrinsics
            )
            clusters: list[dict[str, Any]] = []
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


    def _run_yolo_inference(self, rgb_image: np.ndarray) -> tuple[Any, int]:
        """Run YOLO inference on RGB image (synchronous).

        Args:
            rgb_image: RGB image as numpy array.

        Returns:
            Tuple of (YOLO result, number of detections).
        """
        # Model is already on the correct device (set during initialization)
        # No need to pass device parameter - avoids device transfer overhead
        results = self.model(rgb_image, verbose=False)
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

    async def _transform_to_world_frame(
        self,
        pose: Pose,
        pointcloud: PointCloud2 | None,
        source_frame: str,
        timestamp,
    ) -> tuple[Pose, PointCloud2 | None, str]:
        """Transform pose and point cloud from source frame to world frame.

        This function performs a critical transformation that must succeed.
        If the transform lookup fails, a fatal error is raised.

        Args:
            pose: Pose in source frame.
            pointcloud: Optional point cloud in source frame.
            source_frame: Source frame ID (e.g., camera frame).
            timestamp: ROS timestamp for the transform lookup.

        Returns:
            Tuple of (transformed_pose, transformed_pointcloud, actual_frame).
            actual_frame is the frame the pose is actually in (target_frame if
            transform succeeded, source_frame if fallback was used).

        Raises:
            RuntimeError: If the transform lookup fails or times out.
                This is a critical error - no fallback is provided.
        """
        # Timeout: Maximum time to wait for the transform to become available.
        # If the transform from source_frame to "world" is not available within
        # this time, the lookup will fail with a LookupException.
        # This typically happens if:
        # - The TF tree is not properly set up
        # - The robot_localization or SLAM system is not running
        # - There's a network/communication issue with the TF broadcaster
        from rclpy.duration import Duration

        # Early return if source and target frames are the same
        if source_frame == self.target_frame:
            # No transformation needed - return as-is
            return pose, pointcloud, source_frame

        transform_timeout = Duration(seconds=2.0)

        try:
            # Use Time(0) to request the latest available transform.
            # This works around the issue where ROS clock time and TF time
            # may use different epochs, causing "extrapolation into the future" errors.
            from rclpy.time import Time

            # Time(0) means "use the most recent transform available"
            # This is the recommended way to get the latest transform when
            # you don't care about the exact timestamp
            latest_time = Time(seconds=0, nanoseconds=0)

            # Lookup transform from source_frame to "world" frame
            # This is a blocking call that waits up to transform_timeout seconds
            # for the transform to become available in the TF buffer.
            transform = await asyncio.to_thread(
                self.tf_buffer.lookup_transform,
                self.target_frame,  # "world"
                source_frame,
                latest_time,  # Time(0) = use latest available
                timeout=transform_timeout,
            )

            # Extract the actual transform time from the result
            # This ensures we use the timestamp from the TF data, not our request time
            transform_time = transform.header.stamp

            # Transform pose using the retrieved transform
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = source_frame
            pose_stamped.header.stamp = transform_time
            transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(
                pose_stamped, transform
            )
            transformed_pose = transformed_pose_stamped.pose

            # Transform point cloud if available using tf2_sensor_msgs
            transformed_pc = None
            if pointcloud is not None:
                # Create a copy to avoid modifying the original
                from copy import deepcopy

                pc_copy = deepcopy(pointcloud)
                # Point cloud should already be in source_frame (camera_tool_link)
                # Transform directly from source_frame to target_frame (world)
                pc_copy.header.frame_id = source_frame
                pc_copy.header.stamp = transform_time
                transformed_pc = tf2_sensor_msgs.do_transform_cloud(pc_copy, transform)
                # do_transform_cloud should set the frame_id correctly, but ensure it
                transformed_pc.header.frame_id = self.target_frame

            return transformed_pose, transformed_pc, self.target_frame

        except Exception as e:
            # Transform failed - check if target frame exists
            # If not, use source frame as fallback
            error_str = str(e).lower()
            if "does not exist" in error_str or "not available" in error_str:
                # Frame doesn't exist - use source frame as fallback
                self.get_logger().warn(
                    f"Target frame '{self.target_frame}' not available in TF tree. "
                    f"Saving entity in source frame '{source_frame}' instead. "
                    f"Error: {e}"
                )
                # Return pose and pointcloud unchanged (in source frame)
                return pose, pointcloud, source_frame
            else:
                # Other error - raise fatal error
                error_msg = (
                    f"CRITICAL: Failed to transform from '{source_frame}' to "
                    f"'{self.target_frame}': {e}. "
                    f"This indicates a problem with the TF tree or robot localization."
                )
                self.get_logger().fatal(error_msg)
                raise RuntimeError(error_msg) from e

    async def _update_knowledge_base(
        self, entities: list[dict], frame_id: str, timestamp
    ) -> int:
        """Update knowledge base with detected entities.

        Entities are transformed from the camera frame to the "world" frame
        before being saved. If the transform fails, a critical error is raised.

        Args:
            entities: List of entity dictionaries.
            frame_id: Source frame ID (camera frame).
            timestamp: ROS timestamp for transform lookup.

        Returns:
            Number of entities successfully saved in the knowledge base.
        """
        if not self._kb_services_available:
            return 0

        # Verify services are still available
        service_ready = (
            self.client_get_entities.service_is_ready()
            and self.client_add_entities.service_is_ready()
            and self.client_del_entities.service_is_ready()
        )
        if not service_ready:
            self._kb_services_available = False
            self._start_periodic_service_check()
            return 0

        if len(entities) == 0:
            return 0

        now = self.get_clock().now().to_msg()
        KB_TIMEOUT = 2.0

        # Delete old entities created by this node before adding new ones
        # This prevents accumulation of old entities with incorrect frames
        # Only delete if parameter is enabled (default: True)
        if self._delete_old_entities:
            try:
                # Get all existing entities
                get_entities_req = GetEntities.Request()
                get_entities_resp = await asyncio.wait_for(
                    self.client_get_entities.call_async(get_entities_req),
                    timeout=KB_TIMEOUT,
                )
                if (
                    get_entities_resp is not None
                    and get_entities_resp.result.result_type == Result.SUCCESS
                ):
                    # Get entity details to filter by description
                    entity_ids_to_delete = []
                    for entity_id in get_entities_resp.entities:
                        # Delete all entities to ensure clean state
                        # TODO: Implement smarter filtering (e.g., by description
                        # "Detected:" or timestamp) to avoid deleting other entities
                        entity_ids_to_delete.append(entity_id)

                    # Delete old entities if any
                    if entity_ids_to_delete:
                        del_req = DelEntities.Request()
                        del_req.entityids = entity_ids_to_delete
                        try:
                            del_resp = await asyncio.wait_for(
                                self.client_del_entities.call_async(del_req),
                                timeout=KB_TIMEOUT,
                            )
                            if del_resp is not None:
                                self.get_logger().info(
                                    f"Deleted {len(entity_ids_to_delete)} old "
                                    f"entit(y/ies) before adding new ones"
                                )
                        except (asyncio.TimeoutError, Exception) as e:
                            self.get_logger().warn(
                                f"Failed to delete old entities: {e}. "
                                f"Continuing with adding new entities."
                            )
            except (asyncio.TimeoutError, Exception) as e:
                self.get_logger().warn(
                    f"Failed to get existing entities for cleanup: {e}. "
                    f"Continuing with adding new entities."
                )

        # Add entities sequentially
        add_responses = []
        saved_count = 0  # Track how many entities were successfully saved
        for entity in entities:
            try:
                # Transform pose and point cloud from camera frame to target frame
                # If target frame doesn't exist, fallback to source frame
                (
                    transformed_pose,
                    transformed_pc,
                    actual_frame,
                ) = await self._transform_to_world_frame(
                    pose=entity["pose"],
                    pointcloud=entity.get("pointcloud"),
                    source_frame=frame_id,
                    timestamp=timestamp,
                )

                # Update entity dict with transformed point cloud
                if transformed_pc is not None:
                    entity["pointcloud"] = transformed_pc

                add_entity_req = AddEntity.Request()
                add_entity_req.data = Entity(
                    description=f"Detected: {entity['name'].data}",
                    pose=transformed_pose,
                    pose_reference_frame=actual_frame,
                    stamp=now,
                )
                self.get_logger().info(
                    f"Saving entity '{entity['name'].data}' "
                    f"with pose_reference_frame='{actual_frame}' "
                    f"(pose: x={transformed_pose.position.x:.3f}, "
                    f"y={transformed_pose.position.y:.3f}, "
                    f"z={transformed_pose.position.z:.3f})"
                )
                try:
                    response = await asyncio.wait_for(
                        self.client_add_entities.call_async(add_entity_req),
                        timeout=KB_TIMEOUT,
                    )
                    if response is not None and hasattr(response, "entityid"):
                        saved_count += 1
                    add_responses.append(response)
                except (asyncio.TimeoutError, Exception):
                    add_responses.append(None)
            except RuntimeError as e:
                # Transform failed - skip this entity (critical error, no fallback)
                self.get_logger().error(
                    f"Skipping entity '{entity['name'].data}' "
                    f"due to transform failure: {e}"
                )
                add_responses.append(None)  # Mark as failed for shape update
                continue

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

        return saved_count

    async def process_data(
        self,
        rgb_msg: Image,
        depth_msg: Image | None = None,
        return_entities: bool = False,
        use_locks: bool = True,
    ) -> list[dict] | None:
        """Process incoming RGB images and sync detections with KB.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.
            depth_msg: Optional incoming depth `sensor_msgs/Image`.
            return_entities: If True, return list of entity dictionaries.
                If False (default), return None and only save to KB.
            use_locks: If True, use processing locks to prevent parallel execution.
                If False, skip locks (for direct calls from predict_snapshot).

        Returns:
            List of entity dictionaries if return_entities=True, None otherwise.
        """
        if not self._camera_intrinsics_set:
            if not self._warned_missing_intrinsics:
                self.get_logger().warn(
                    "Camera intrinsics not set - cannot process frame. "
                    "Waiting for camera_info topic..."
                )
                self._warned_missing_intrinsics = True
            if return_entities:
                return []
            return None

        if use_locks:
            if self._processing_lock_async is None:
                if not self._warned_missing_async_lock:
                    self.get_logger().warn(
                        "Async processing lock not initialized - dropping frame. "
                        "This should not happen if async_init() was called."
                    )
                    self._warned_missing_async_lock = True
                if return_entities:
                    return []
                return None

            # Check if already processing (async lock prevents parallel processing)
            async with self._processing_lock_async:
                if self._processing_frame:
                    if return_entities:
                        return []
                    return None
                self._processing_frame = True

                # Update last processing time (for processing_timeout, if enabled)
                current_time = time.perf_counter()
                self._last_processing_time = current_time

                # Start timer for total processing time
                t_start = time.perf_counter()

                try:
                    entities = await self._process_data_internal(
                        rgb_msg, depth_msg, t_start, return_entities
                    )
                finally:
                    # Always reset processing flag, even if error occurred
                    self._processing_frame = False
                return entities
        else:
            # Direct call without locks (for predict_snapshot)
            t_start = time.perf_counter()
            return await self._process_data_internal(
                rgb_msg, depth_msg, t_start, return_entities
            )

    async def _process_data_internal(
        self,
        rgb_msg: Image,
        depth_msg: Image | None,
        t_start: float,
        return_entities: bool,
    ) -> list[dict] | None:
        """Internal processing logic without locks.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.
            depth_msg: Optional incoming depth `sensor_msgs/Image`.
            t_start: Start time for performance measurement.
            return_entities: Whether to return entities list.

        Returns:
            List of entity dictionaries if return_entities=True, None otherwise.
        """
        try:
            # Convert ROS image to numpy array (BGR to RGB)
                bgr_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
                rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
                original_height, original_width = rgb_image.shape[:2]

                # Scale image to 640x640 for YOLO inference (square input)
                scale_factor_x = 1.0
                scale_factor_y = 1.0
                yolo_image = rgb_image
                if self.max_image_width > 0:
                    # Scale to square 640x640
                    yolo_size = self.max_image_width
                    scale_factor_x = yolo_size / original_width
                    scale_factor_y = yolo_size / original_height
                    yolo_image = cv2.resize(
                        rgb_image,
                        (yolo_size, yolo_size),
                        interpolation=cv2.INTER_LINEAR,
                    )

                # Run YOLO inference (synchronous)
                t_yolo_start = time.perf_counter()
                result, num_detections = self._run_yolo_inference(yolo_image)
                t_yolo_end = time.perf_counter()
                inference_time_ms = (t_yolo_end - t_yolo_start) * 1000

                # Free large image arrays immediately after inference to reduce memory
                # pressure (helps prevent swap during slow inference)
                if yolo_image is not rgb_image:  # Only delete if it's a separate array
                    del yolo_image
                del rgb_image
                del bgr_image

                # Get device info for diagnostics
                device_info = "unknown"
                if hasattr(result, "boxes") and result.boxes is not None:
                    if hasattr(result.boxes, "data") and result.boxes.data is not None:
                        if hasattr(result.boxes.data, "device"):
                            device_info = str(result.boxes.data.device)

                # Log inference time with device info
                self.get_logger().info(
                    f"YOLO inference: {inference_time_ms:.1f}ms (device: {device_info})"
                )
                if num_detections > 0:
                    # Log what YOLO classified
                    if hasattr(result, "boxes") and hasattr(result.boxes, "cls"):
                        class_ids = result.boxes.cls.cpu().numpy().astype(int)
                        confidences = (
                            result.boxes.conf.cpu().numpy()
                            if hasattr(result.boxes, "conf")
                            else None
                        )
                        detections = []
                        for i, class_id in enumerate(class_ids):
                            class_name = self.model.names[class_id]
                            if confidences is not None:
                                detections.append(
                                    f"{class_name} ({confidences[i]:.2f})"
                                )
                            else:
                                detections.append(class_name)
                        detections_str = ", ".join(detections)
                        self.get_logger().info(
                            f"{num_detections} object(s) detected: {detections_str}"
                        )
                    else:
                        self.get_logger().info(f"{num_detections} object(s) detected")

                # Scale bounding boxes back to original image size if image was scaled
                if (scale_factor_x != 1.0 or scale_factor_y != 1.0) and hasattr(
                    result, "boxes"
                ):
                    # YOLO boxes need to be scaled back to original image coordinates
                    # Only scale xywh (the base format) - xyxy is computed from xywh
                    boxes = result.boxes
                    if hasattr(boxes, "xywh") and boxes.xywh is not None:
                        # Scale xywh format (center x, center y, width, height)
                        boxes.xywh[:, 0] = boxes.xywh[:, 0] / scale_factor_x  # x_center
                        boxes.xywh[:, 1] = boxes.xywh[:, 1] / scale_factor_y  # y_center
                        boxes.xywh[:, 2] = boxes.xywh[:, 2] / scale_factor_x  # width
                        boxes.xywh[:, 3] = boxes.xywh[:, 3] / scale_factor_y  # height
                        # Note: xyxy is a computed property and will be automatically
                        # updated based on the scaled xywh values

                # Extract segmentation masks
                # For depth processing, scale masks to depth image size
                # For entity generation, scale masks to original RGB size
                masks_for_depth = None
                masks_for_entities = None

                if depth_msg is not None and self.use_depth and num_detections > 0:
                    # Extract masks scaled to depth image size for depth processing
                    masks_for_depth = self._extract_masks(
                        result, depth_msg.height, depth_msg.width
                    )
                    # Also extract masks scaled to original RGB size for entities
                    masks_for_entities = self._extract_masks(
                        result, original_height, original_width
                    )
                elif num_detections > 0:
                    # Only extract masks for entities (no depth processing)
                    masks_for_entities = self._extract_masks(
                        result, original_height, original_width
                    )

                # Use masks_for_entities for entity generation
                masks = masks_for_entities

                # Process depth data with mask-based filtering
                # Skip depth processing if no detections (saves memory)
                clusters = []
                if depth_msg is not None and self.use_depth and num_detections > 0:
                    try:
                        _, clusters = await asyncio.to_thread(
                            self._process_depth_with_masks,
                            depth_msg,
                            masks_for_depth,  # Use depth-scaled masks
                        )
                        if len(clusters) > 0:
                            self.get_logger().info(
                                f"{len(clusters)} cluster(s) detected"
                            )
                    except Exception as e:
                        self.get_logger().error(
                            f"Depth processing failed: {e}"
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
                    successful_associations = [
                        a for a in associations if a.get("cluster") is not None
                    ]
                    if len(successful_associations) < num_detections:
                        self.get_logger().warn(
                            f"Only {len(successful_associations)}/{num_detections} "
                            f"detections have associated clusters. "
                            f"This may indicate insufficient depth data or "
                            f"cluster detection issues."
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
                        frame_id=rgb_msg.header.frame_id,  # Original frame
                        # (will be transformed to camera_tool_link then world)
                        timestamp=rgb_msg.header.stamp,
                        max_points=self.point_cloud_max_points,
                    )
                    if len(entities) > 0:
                        entity_names = [e["name"].data for e in entities]
                        entity_str = f"{len(entities)} entit(y/ies) created: "
                        entity_str += ", ".join(entity_names)
                        self.get_logger().info(entity_str)

                # Update knowledge base
                if len(entities) > 0:
                    saved_count = await self._update_knowledge_base(
                        entities, self.source_frame, rgb_msg.header.stamp
                    )
                    t_total = (time.perf_counter() - t_start) * 1000
                    if saved_count > 0:
                        self.get_logger().info(
                            f"{saved_count} of {len(entities)} entit(y/ies) "
                            f"saved in KB (total processing time: {t_total:.2f}ms)"
                        )
                    else:
                        self.get_logger().warn(
                            f"None of {len(entities)} entit(y/ies) could be saved "
                            f"in KB (likely transform failures) "
                            f"(total time: {t_total:.2f}ms)"
                        )
                else:
                    t_total = (time.perf_counter() - t_start) * 1000
                    self.get_logger().info(
                        f"Frame processed (no entities) "
                        f"(total processing time: {t_total:.2f}ms)"
                    )

                # Free large arrays after processing to reduce memory pressure
                # This helps prevent swap during slow inference
                del masks_for_depth
                del masks_for_entities
                del masks
                del clusters
                del result

                # Return entities if requested
                if return_entities:
                    return entities
                return None
        except Exception as e:
            self.get_logger().error(
                f"Error processing frame: {e}"
            )
            if return_entities:
                return []
            return None

    async def predict_snapshot(
        self, rgb_image: Image, depth_image: Image | None = None
    ) -> list[dict]:
        """Process a snapshot (RGB + optional Depth) and return detected entities.

        This method processes a single image snapshot through the YOLO model
        and returns the detected entities. The model must be pre-loaded (done
        during node initialization).

        Args:
            rgb_image: RGB image as sensor_msgs/Image.
            depth_image: Optional depth image as sensor_msgs/Image.
                If None, depth processing is skipped.

        Returns:
            List of entity dictionaries. Each entity contains:
            - 'name': String message with object class name
            - 'pose': PoseStamped message
            - 'pointcloud': Optional PointCloud2 message
            - 'boundingbox2d': Optional BoundingBox2D message
            Empty list if no objects detected or on error.

        Raises:
            RuntimeError: If camera intrinsics are not set (required for processing).
        """
        # Validate camera intrinsics are set
        if not self._camera_intrinsics_set:
            error_msg = (
                "Camera intrinsics not set. "
                "Cannot process snapshot without camera calibration. "
                "Ensure camera_info topic is available."
            )
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)

        # Validate RGB image
        if rgb_image is None:
            error_msg = "RGB image is None. Cannot process snapshot."
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)

        # Process snapshot using existing process_data method
        # use_locks=False: Direct call, no need for frame processing locks
        # return_entities=True: Return entities list instead of just saving to KB
        try:
            entities = await self.process_data(
                rgb_msg=rgb_image,
                depth_msg=depth_image,
                return_entities=True,
                use_locks=False,
            )

            # Ensure we return a list
            # (process_data returns None if return_entities is False)
            if entities is None:
                return []
            return entities

        except Exception as e:
            self.get_logger().error(
                f"Error processing snapshot: {e}"
            )
            # Return empty list on error (don't raise, allow caller to handle)
            return []

    async def _predict_snapshot_goal_handler(self, goal_handle):
        """Handle incoming predict_snapshot action goals.

        This method is called when a new goal is received by the action server.
        It processes the RGB and optional depth images, sends feedback during
        processing, and returns the detected entities.

        Args:
            goal_handle: Action goal handle from rclpy.action.

        Returns:
            PredictSnapshot.Result: Result containing detected entities and status.
        """
        self.get_logger().info("Received predict_snapshot action goal")

        # Extract goal data
        goal = goal_handle.request
        rgb_image = goal.rgb_image
        depth_image = goal.depth_image if hasattr(goal, "depth_image") else None

        # Create result
        result = PredictSnapshot.Result()

        try:
            # Send initial feedback
            feedback = PredictSnapshot.Feedback()
            feedback.status = "Processing snapshot..."
            feedback.progress = 0.0
            feedback.objects_detected = 0
            goal_handle.publish_feedback(feedback)

            # Validate camera intrinsics
            if not self._camera_intrinsics_set:
                error_msg = (
                    "Camera intrinsics not set. "
                    "Cannot process snapshot without camera calibration."
                )
                self.get_logger().error(error_msg)
                result.success = False
                result.error_message = error_msg
                goal_handle.abort()
                return result

            # Validate RGB image
            if rgb_image is None:
                error_msg = "RGB image is None. Cannot process snapshot."
                self.get_logger().error(error_msg)
                result.success = False
                result.error_message = error_msg
                goal_handle.abort()
                return result

            # Send feedback: Starting YOLO inference
            feedback.status = "Running YOLO inference..."
            feedback.progress = 0.2
            goal_handle.publish_feedback(feedback)

            # Call predict_snapshot
            entities = await self.predict_snapshot(rgb_image, depth_image)

            # Send feedback: Processing complete
            feedback.status = "Processing complete"
            feedback.progress = 0.9
            feedback.objects_detected = len(entities)
            goal_handle.publish_feedback(feedback)

            # Convert entity dictionaries to Entity messages
            # Note: predict_snapshot returns list of dicts with keys:
            # - 'name': String message
            # - 'pose': PoseStamped message
            # - 'boundingbox2d': BoundingBox2D message (optional)
            # - 'pointcloud': PointCloud2 message (optional)
            result.entities = []
            for entity_dict in entities:
                # Create Entity message from dictionary
                entity_msg = Entity()

                # Extract name for description
                entity_name = "unknown"
                if "name" in entity_dict and entity_dict["name"] is not None:
                    if hasattr(entity_dict["name"], "data"):
                        entity_name = entity_dict["name"].data
                    else:
                        entity_name = str(entity_dict["name"])

                # Set description (Entity message doesn't have 'name' field)
                entity_msg.description = f"Detected: {entity_name}"

                # Set pose
                if "pose" in entity_dict and entity_dict["pose"] is not None:
                    pose_stamped = entity_dict["pose"]
                    # Check if it's a PoseStamped or just a Pose
                    if hasattr(pose_stamped, "pose"):
                        # It's a PoseStamped
                        entity_msg.pose = pose_stamped.pose
                        if hasattr(pose_stamped.header, "frame_id"):
                            entity_msg.pose_reference_frame = (
                                pose_stamped.header.frame_id
                            )
                        if hasattr(pose_stamped.header, "stamp"):
                            entity_msg.stamp = pose_stamped.header.stamp
                    elif (
                        hasattr(pose_stamped, "position")
                        and hasattr(pose_stamped, "orientation")
                    ):
                        # It's a Pose directly
                        entity_msg.pose = pose_stamped
                        # Use default frame if not set
                        entity_msg.pose_reference_frame = "unknown"
                        entity_msg.stamp = self.get_clock().now().to_msg()

                # Set entity type based on name (simplified - could be improved)
                # This is a placeholder - you may want to map class names to EntityType
                from arlab_knowledge_interfaces.msg import EntityType

                # Default to pickable for most objects
                # EntityType is a message with an 'id' field, not a direct enum
                entity_msg.entity_type = EntityType(id=EntityType.PICKABLE)
                # Could add logic here to map specific class names to
                # furniture/human types

                # Note: Entity message doesn't include pointcloud or boundingbox2d
                # These are stored separately in the KB via UpdShape service
                # If needed, we could extend the action result to include these

                result.entities.append(entity_msg)

            # Set result
            result.success = True
            result.error_message = ""

            # Send final feedback
            feedback.status = f"Successfully detected {len(entities)} objects"
            feedback.progress = 1.0
            feedback.objects_detected = len(entities)
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(
                f"Action completed successfully: {len(entities)} entities detected"
            )
            goal_handle.succeed()
            return result

        except Exception as e:
            error_msg = f"Error processing snapshot: {str(e)}"
            self.get_logger().error(f"{error_msg}: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            result.success = False
            result.error_message = error_msg
            goal_handle.abort()
            return result

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
    max_points: int = 10000,
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
        max_points: Maximum number of points per entity point cloud
            (0 = no limit, not recommended).

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
                    # Apply downsampling if max_points is set (> 0)
                    if max_points > 0 and len(points) > max_points:
                        points = voxel_downsample(
                            points, voxel_size=0.01, max_points=max_points
                        )
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
