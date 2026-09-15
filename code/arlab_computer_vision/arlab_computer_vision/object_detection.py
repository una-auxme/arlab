"""ROS2 YOLO-based object detection node with KB integration.

This module defines a ROS2 node that performs real-time object detection
using Ultralytics YOLO segmentation, converts results into
semantic/geometric entities, and communicates with a knowledge base (KB)
via ROS services.

Overview of the main components:
- ObjectDetection: the ROS node; owns the camera subscriptions, the
  `/vision/snapshot` action server and the KB service clients.
- ModelRegistry: caches YOLO models, loading pinned ones eagerly and all
  others on first use, evicts idle ones to free GPU memory.
- create_entity(): converts one detection (label + point cloud) into the
  Entity/Shape message pair the KB stores.

Both the set of available models and the YOLO label -> KB entity mapping are
configured in `config/models.yaml` and `config/labels.yaml` rather than in
this file; see `_load_model_definitions` and `_load_label_definitions`.

Maintainers:
    Simeon Wagner <simeon.wagner@uni-a.de>
    Lars Britz    <lars.britz@uni-a.de>
"""

import os
import time
from copy import deepcopy
from dataclasses import dataclass
from threading import Lock
from time import sleep
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import rclpy.executors
import ros2_numpy
import torch
import yaml
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


def _load_model_definitions(path: str) -> dict[int, dict]:
    """Load model definitions from a YAML config.

    The YAML keys (e.g. "DISHWASHER") are resolved to VisionSnapshotCommand.MODEL_*
    constants, a typo or stale entry fails fast at startup. pinned=True models are loaded at startup and never
    evicted; all others are lazy-loaded.

    Args:
        path: Path to the models YAML file in which a model is declared

    Returns:
        dict[int, dict]: Maps a VisionSnapshotCommand.MODEL_* ID to its
            definition with the keys "name" (str, for log output), "weights"
            (str, filename relative to the package's yolo_weights/ directory)
            and "pinned" (bool).

    Raises:
        AttributeError: If a YAML key has no matching VisionSnapshotCommand
            constant.
    """
    with open(path) as f:
        raw = yaml.safe_load(f)["models"]
    definitions: dict[int, dict] = {}
    for key, defn in raw.items():
        model_id = getattr(VisionSnapshotCommand, f"MODEL_{key}")
        definitions[model_id] = {
            "name": defn["name"],
            "weights": defn["weights"],
            "pinned": defn.get("pinned", False),
        }
    return definitions


def _load_label_definitions(path: str) -> dict[str, dict]:
    """Load YOLO label -> KB entity type/category mappings from a YAML config.

    Resolves each entry's "type" to an EntityType.* constant and optional
    "object_category" to an EntityPickable.OBJECT_CATEGORY_* constant at load
    time, so a typo fails fast at startup instead of at first detection.

    A "type" naming a specific EntityFurniture submessage field (e.g.
    "dishwasher", matching EntityFurniture.msg) may carry an "attributes" map,
    whose entries are set directly on that submessage (right now just the with its attribut dishwasher.open).

    Args:
        path: Path to the labels YAML file, expected to hold a top-level
            "labels" mapping keyed by YOLO class name.

    Returns:
        dict[str, dict]: Maps a YOLO label to its definition with the keys
            "entity_type_id" (int, an EntityType.* constant), "object_category"
            (int | None, an EntityPickable.OBJECT_CATEGORY_* constant),
            "furniture_field" (str | None, the EntityFurniture submessage field
            to populate) and "attributes" (dict, values to set on it). Labels
            missing from this mapping are handled by `create_entity`.

    Raises:
        AttributeError: If a "type" or "object_category" has no matching
            EntityType / EntityPickable constant.
    """
    with open(path) as f:
        raw = yaml.safe_load(f)["labels"]
    definitions: dict[str, dict] = {}
    for label, defn in raw.items():
        object_category = None
        # NOTE has the premise that only "pickable entities" have an object category (i.e. sphere, cylinder etc.). To be discussed.
        if "object_category" in defn:
            object_category = getattr(EntityPickable, f"OBJECT_CATEGORY_{defn['object_category'].upper()}")
        attributes = defn.get("attributes", {})
        definitions[label] = {
            "entity_type_id": getattr(EntityType, defn["type"].upper()),
            "object_category": object_category,
            "furniture_field": defn["type"].lower() if attributes else None,
            "attributes": attributes,
        }
    return definitions


@dataclass
class _ModelEntry:
    """One loaded YOLO model together with its cache bookkeeping.

    Attributes:
        model (YOLO): The loaded and (on CUDA) warmed up Ultralytics model.
        pinned (bool): If true, the entry is exempt from TTL eviction.
        last_used (float): `time.time()` epoch of the last `get_model` call, based on this timestamp eviction occurs if TTL is surpassed
    """

    model: Any
    pinned: bool
    last_used: float


class ModelRegistry:
    """Manages YOLO models with lazy loading and TTL eviction for non-pinned models.

    Pinned models are loaded once at startup and kept resident because they run
    on every frame. All other models are loaded on their first request and
    released again after an idle timeout, so rarely used weights do not occupy
    GPU memory permanently.

    All access is serialized through an internal lock, since models are
    requested from the ROS executor's worker threads.
    """

    def __init__(
        self,
        definitions: dict,
        weights_dir: str,
        device: str,
        logger: Any,
        warmup_size: int = 640,
    ) -> None:
        """Set up an empty registry.

        No weights are read here; call `load_pinned` to populate the registry
        with the pinned models.

        Args:
            definitions: Model definitions as returned by
                `_load_model_definitions`.
            weights_dir: Directory the definitions' "weights" filenames are
                resolved against.
            device: Torch device to run inference on, "cuda" or "cpu".
            logger: ROS logger used to report loading and eviction.
            warmup_size: Edge length of the square dummy image used to warm up
                a freshly loaded CUDA model.
        """
        self._definitions = definitions
        self._weights_dir = weights_dir
        self._device = device
        self._logger = logger
        self._warmup_size = warmup_size
        self._registry: dict[int, _ModelEntry] = {}
        self._lock = Lock()

    def load_pinned(self) -> None:
        """Load all pinned models into GPU memory with warmup.

        Called once during node construction so the first frame does not pay
        the loading cost. A model whose weights are missing is skipped with a
        warning rather than aborting startup.
        """
        for model_id, defn in self._definitions.items():
            if defn.get("pinned", False):
                with self._lock:
                    self._ensure_loaded(model_id)

    def get_pinned_ids(self) -> list[int]:
        """Return the IDs of the models that run on every processed frame.

        Returns:
            list[int]: VisionSnapshotCommand.MODEL_* IDs marked pinned in the
                configuration, regardless of whether they loaded successfully.
        """
        return [mid for mid, defn in self._definitions.items() if defn.get("pinned", False)]

    def get_model(self, model_id: int) -> Any | None:
        """Return a ready-to-use model, loading it on first request.

        Marks the entry as used, which resets its eviction timeout.

        Args:
            model_id: A VisionSnapshotCommand.MODEL_* constant.

        Returns:
            Any | None: The Ultralytics model, or None if the ID is unknown or
                its weights file is missing. Callers must handle None.
        """
        with self._lock:
            self._ensure_loaded(model_id)
            entry = self._registry.get(model_id)
            if entry is not None:
                entry.last_used = time.time()
                return entry.model
        return None

    def evict_stale(self, ttl_seconds: float) -> None:
        """Remove non-pinned models that have not been used within ttl_seconds.

        Dropping the reference releases the model's GPU memory; a later request
        simply reloads it.

        Args:
            ttl_seconds: Idle time after which a non-pinned model is released.
        """
        now = time.time()
        with self._lock:
            to_evict = [mid for mid, entry in self._registry.items() if not entry.pinned and (now - entry.last_used) > ttl_seconds]
            for mid in to_evict:
                del self._registry[mid]
                self._logger.info(f"Evicted model '{self._definitions[mid]['name']}'")

    def _ensure_loaded(self, model_id: int) -> None:
        """Load model if not already cached. Caller must hold self._lock.

        On CUDA the model is run over dummy images before being cached,
        to force kernel compilation and memory allocation.

        Args:
            model_id: A VisionSnapshotCommand.MODEL_* constant.

        Notes:
            An unknown ID or a missing weights file is reported as a warning
            and leaves the registry unchanged, so `get_model` returns None.
        """
        if model_id in self._registry:
            return
        if model_id not in self._definitions:
            self._logger.warn(f"Unknown model ID {model_id}")
            return
        defn = self._definitions[model_id]
        weights_path = os.path.join(self._weights_dir, defn["weights"])
        if not os.path.exists(weights_path):
            self._logger.warn(f"Weights not found for '{defn['name']}': {weights_path}")
            return
        self._logger.info(f"Loading model '{defn['name']}' ...")
        model = YOLO(weights_path)
        if self._device == "cuda":
            model.to("cuda")
            try:
                dummy = np.zeros((self._warmup_size, self._warmup_size, 3), dtype=np.uint8)
                for _ in range(3):
                    model(dummy, verbose=False, device=self._device)
                torch.cuda.synchronize()
            except Exception as e:
                self._logger.warn(f"Warmup failed for '{defn['name']}': {e}")
        self._registry[model_id] = _ModelEntry(
            model=model,
            pinned=defn.get("pinned", False),
            last_used=time.time(),
        )
        self._logger.info(f"Model '{defn['name']}' ready")


class ObjectDetection(Node):
    """ROS2 node for real-time object detection and KB synchronization.

    The node:
        1) Subscribes to RGB images, camera intrinsics and (when `use_depth` is
           set) a time-synchronized point cloud.
        2) Runs every pinned YOLO segmentation model on the frame, plus any
           extra models a snapshot request asks for.
        3) Converts detections to semantic entities (label, point cloud, pose)
           using the label mapping from `config/labels.yaml`.
        4) Interacts with a knowledge base through ROS services to insert or
           update entities.

    Processing runs in one of two modes, selected by the `snapshot_mode`
    parameter: continuously on every incoming frame, or only on demand when a
    `/vision/snapshot` action goal arrives.

    Attributes:
        bridge (CvBridge): ROS-OpenCV conversion bridge.
        device (str): Torch device used for inference, "cuda" or "cpu".
        _model_registry (ModelRegistry): Cache of the YOLO models this node may
            run; see `config/models.yaml`.
        _label_definitions (dict[str, dict]): YOLO label -> KB entity mapping;
            see `config/labels.yaml`.
        _snapshot_mode (bool): If true, frames are only processed on a
            `/vision/snapshot` goal instead of continuously.
        camera_intrinsics_matrix (NDArray | None): 3x3 intrinsic matrix K, set
            once the first `camera_info` message arrives.
        tf_buffer (Buffer): TF2 buffer used to transform the point cloud into
            `target_frame` and to project it into the color image.
        target_frame (str): TF frame the published entity poses refer to.
        prefix (str): Namespace prefix for KB services.
        client_get_entities: Service client for fetching entities.
        client_del_entities: Service client for deleting entities.
        client_add_entities: Service client for adding an entity.
        client_upd_shape: Service client for updating entity shapes.
        segmented_image_pub: Publisher for the annotated debug image.
        debug_pointcloud_pub: Publisher for the per-entity debug point cloud.
    """

    def __init__(self) -> None:
        """Initialize the node, parameters, subscriptions, and service clients.

        Loads the model and label configuration from the package share
        directory, brings up the pinned YOLO models, and connects the camera,
        TF and knowledge base interfaces.

        Parameters:
            model_ttl_minutes (int): Idle time after which a non-pinned model is
                released from memory. Read on every eviction cycle, so it takes
                effect at runtime.
            visualize (bool): Publish the annotated image on
                `/vision/segmented_image`.
            log_level (str): Logger severity, one of DEBUG, INFO, WARN, ERROR,
                FATAL.
            use_depth (bool): Consume a point cloud alongside the RGB image.
                Without it no 3D geometry, and therefore no entity, can be
                produced.
            sync_tolerance (float): Maximum timestamp difference in seconds
                between an RGB image and a point cloud for them to be paired.
            use_clustering (bool): Reduce each detection's points to its largest
                DBSCAN cluster. Read per detection, so it takes effect at
                runtime.
            delete_old_entities (bool): In continuous mode, delete the
                previously stored pickables after each frame. In snapshot mode
                the action goal decides instead.
            clear_db_on_no_detection (bool): Also delete the old entities when
                the current frame detected nothing, keeping the KB in sync with
                what the camera currently sees.
            max_image_width (int): Edge length the image is scaled to before
                inference; 0 keeps the original resolution.
            target_frame (str): TF frame the entity poses and point clouds are
                expressed in.
            snapshot_mode (bool): Process frames only on a `/vision/snapshot`
                goal rather than continuously.

        Side Effects:
            - Loads all pinned YOLO models onto the GPU.
            - Starts the `/vision/snapshot` action server when snapshot mode is
              enabled.
            - Starts timers for KB service discovery, frame statistics and model
              eviction.
        """
        super().__init__(type(self).__name__)

        package_share_dir = get_package_share_directory("arlab_computer_vision")
        yolo_weights_dir = os.path.join(package_share_dir, "yolo_weights")
        model_definitions = _load_model_definitions(os.path.join(package_share_dir, "config", "models.yaml"))
        self._label_definitions = _load_label_definitions(os.path.join(package_share_dir, "config", "labels.yaml"))

        # Declare configurable parameters; see the Parameters section above.
        self.declare_parameter("model_ttl_minutes", 10)
        self.declare_parameter("visualize", True)
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("use_depth", True)
        self.declare_parameter("sync_tolerance", 0.5)
        self.declare_parameter("use_clustering", True)
        self.declare_parameter("delete_old_entities", True)
        self.declare_parameter("clear_db_on_no_detection", True)
        self.declare_parameter("max_image_width", 640)
        self.declare_parameter("target_frame", "camera_tool_link")
        self.declare_parameter("snapshot_mode", True)

        self.visualize = self.get_parameter("visualize").get_parameter_value().bool_value

        log_level_str = self.get_parameter("log_level").get_parameter_value().string_value

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

        self.use_depth = self.get_parameter("use_depth").get_parameter_value().bool_value
        self.sync_tolerance = self.get_parameter("sync_tolerance").get_parameter_value().double_value

        self._delete_old_entities = self.get_parameter("delete_old_entities").get_parameter_value().bool_value
        self.get_logger().info(f"Delete old entities: {self._delete_old_entities}")

        self._clear_db_on_no_detection = self.get_parameter("clear_db_on_no_detection").get_parameter_value().bool_value
        self.get_logger().info(f"Clear DB on no detection: {self._clear_db_on_no_detection}")

        self.max_image_width = self.get_parameter("max_image_width").get_parameter_value().integer_value
        if self.max_image_width > 0:
            self.get_logger().info(f"YOLO input size: {self.max_image_width}x{self.max_image_width} (images will be scaled to square size)")
        else:
            self.get_logger().info("Image scaling disabled - using original image size for YOLO")

        self._snapshot_mode = self.get_parameter("snapshot_mode").get_parameter_value().bool_value
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

        # Guards the cached frame against concurrent access by the camera
        # callback and the snapshot execution thread.
        self.vision_data_mutex = Lock()

        self.bridge = CvBridge()
        device = "cuda" if torch.cuda.is_available() else "cpu"
        if device == "cuda":
            if not torch.cuda.is_available():
                self.get_logger().warn("CUDA not available despite device='cuda'. Falling back to CPU.")
                device = "cpu"
            else:
                try:
                    gpu_name = torch.cuda.get_device_name(0)
                    gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1e9
                    self.get_logger().info(f"GPU: {gpu_name} ({gpu_memory:.1f} GB)")
                except Exception as e:
                    self.get_logger().warn(f"GPU verification failed: {e}")
        self.device = device

        warmup_size = self.max_image_width if self.max_image_width > 0 else 640
        self._model_registry = ModelRegistry(
            definitions=model_definitions,
            weights_dir=yolo_weights_dir,
            device=self.device,
            logger=self.get_logger(),
            warmup_size=warmup_size,
        )
        self._model_registry.load_pinned()

        # Frame statistics
        self._frames_processed = 0
        self._frames_skipped = 0

        # Camera intrinsics; populated asynchronously by camera_info_callback.
        # Until they arrive, _process_data cannot build 3D geometry.
        self.camera_intrinsics_matrix: Optional[NDArray] = None
        self.camera_intrinsics: dict[str, float] | None = None
        self._camera_intrinsics_set = False
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None
        self._rgb_width: int | None = None
        self._rgb_height: int | None = None
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

        # Set by _check_kb_services; every KB interaction is skipped while false.
        self._kb_services_available = False

        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.get_logger().info(f"Target frame for TF transformations: '{self.target_frame}'")

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
            rgb_sub = Subscriber(self, Image, "camera_color_image")
            pointcloud_sub = Subscriber(self, PointCloud2, "camera_point_cloud")

            # queue_size=1 buffers only a single synchronized pair, which
            # prevents OOM during slow inference and reduces swap pressure.
            self.sync = ApproximateTimeSynchronizer(
                [rgb_sub, pointcloud_sub],
                queue_size=1,
                slop=self.sync_tolerance,
            )
            self.sync.registerCallback(self._image_data_callback)
            self.get_logger().info(f"Subscribed to synchronized RGB and depth topics (tolerance: {self.sync_tolerance}s)")
        else:
            self.create_subscription(
                Image,
                "camera_color_image",
                self._image_data_callback,
                qos_profile=1,  # Only keep latest frame to reduce delay
            )
            self.get_logger().info("Subscribed to camera_color_image topic")
        self.color_image: Optional[Image] = None
        self.pointcloud: Optional[PointCloud2] = None

        self.segmented_image_pub = self.create_publisher(Image, "/vision/segmented_image", qos_profile=1)
        self.debug_pointcloud_pub = self.create_publisher(PointCloud2, "/vision/debug_pc", qos_profile=1)

        # The timer cancels itself once the KB has come up; the immediate call
        # avoids waiting a full period when it is already running.
        self._knowledge_timer = self.create_timer(timer_period_sec=5.0, callback=self._check_kb_services)
        self._check_kb_services()
        self.create_timer(timer_period_sec=5.0, callback=self._frame_statistics_reporter)
        self.create_timer(timer_period_sec=60.0, callback=self._evict_stale_models)

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
            self.get_logger().warn("Knowledge base services not available. Will retry periodically.")
        return available

    def _frame_statistics_reporter(self):
        """Log how many frames were processed and dropped, and reset the counters.

        Timer callback (5 s). A high skip rate means inference is slower than
        the camera's frame rate.
        """
        total = self._frames_processed + self._frames_skipped
        if total > 0:
            processed_pct = self._frames_processed / total * 100 if total > 0 else 0.0
            skipped_pct = self._frames_skipped / total * 100 if total > 0 else 0.0
            self.get_logger().info(
                f"Frame statistics (last 5s): "
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
        """Cache the latest camera frame and, outside snapshot mode, process it.

        Registered either on the RGB subscription directly or on the
        RGB/point cloud `ApproximateTimeSynchronizer`, depending on `use_depth`.
        The cached frame is what a later `/vision/snapshot` goal operates on.

        Args:
            rgb_msg: Incoming `sensor_msgs/Image`.
            pointcloud_msg: Matching `sensor_msgs/PointCloud2`, or None when
                depth is disabled.
        """
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
        extra_models: List[int] | None = None,
    ) -> None:
        """Fully synchronous frame processing - no async, no threads.

        Runs all pinned models plus any extra_models requested by the caller.
        Depth preparation (TF lookups, point projection) is done once and
        shared across all model results.

        Each detection's segmentation mask selects the point cloud points that
        fall inside it, this points yield the 3D geometry and pose. Without
        a point cloud or camera intrinsics no entities can be produced and the method only
        publishes the annotated image.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.
            pointcloud_msg: Optional incoming `sensor_msgs/PointCloud2`.
            delete_old_entities: If true, delete previously stored entities first.
            mask_hand: If true, zero the lower image region to suppress hand detections.
            extra_models: Additional model IDs (VisionSnapshotCommand.MODEL_*) to run.

        Side Effects:
            - Publishes to `/vision/segmented_image` and `/vision/debug_pc`.
            - Adds the detected entities to the knowledge base and, when asked,
              deletes the pickables that were stored before this frame.

        Raises:
            tf2_ros.TransformException: If the point cloud cannot be transformed
                into `target_frame` or the color frame within the timeout.
        """
        t_start = time.perf_counter()

        # PREPROCESSING (CPU)
        t_pre = time.perf_counter()
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        original_height, original_width = rgb_image.shape[:2]

        if self.max_image_width > 0:
            yolo_size = self.max_image_width
            scale_x = yolo_size / original_width
            scale_y = yolo_size / original_height
            yolo_image = cv2.resize(rgb_image, (yolo_size, yolo_size), interpolation=cv2.INTER_LINEAR)
        else:
            yolo_image = rgb_image
            scale_x = scale_y = 1.0
        preprocess_ms = (time.perf_counter() - t_pre) * 1000

        # DEPTH PREPARATION
        structured_points = None
        camera_points_idxs = None
        points_header = None

        if pointcloud_msg is not None and self.use_depth and self.camera_intrinsics_matrix is not None:
            # (a) Express the points in target_frame, which is what the entity
            # poses are reported in. Time(seconds=0.0) requests the latest
            # available transform, as this pair is expected to be static.
            depth_to_target_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                pointcloud_msg.header.frame_id,
                Time(seconds=0.0),
                timeout=Duration(seconds=1.0),
            )
            depth_to_target: NDArray = ros2_numpy.numpify(depth_to_target_msg.transform)

            # Deep copy because the transformed coordinates are written back
            # into the array in place.
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

            # (b) Independently, project the points into the color image so each
            # point can be tested against a segmentation mask. Uses the image's
            # own stamp, since this transform may move with the camera.
            depth_to_color_msg = self.tf_buffer.lookup_transform(
                rgb_msg.header.frame_id,
                pointcloud_msg.header.frame_id,
                Time.from_msg(rgb_msg.header.stamp),
                timeout=Duration(seconds=1.0),
            )
            depth_to_color: NDArray = ros2_numpy.numpify(depth_to_color_msg.transform)

            np_points = pointcloud2_to_xyz_array(pointcloud_msg)
            np_points = np_points.transpose(1, 0)
            np_points = np.concatenate([np_points, np.ones((1, np_points.shape[-1]))])
            np_points = depth_to_color @ np_points
            camera_points = self.camera_intrinsics_matrix @ np_points[:3]
            # Perspective divide, leaving homogeneous pixel coordinates (u, v).
            camera_points = camera_points / camera_points[2]
            camera_points = camera_points[:2]
            camera_points_idxs = camera_points.astype(np.int32)
            # Swap to (row, column) order so the result indexes a mask directly.
            camera_points_idxs[[0, 1]] = camera_points_idxs[[1, 0]]
            # Points projecting outside the image are clamped to the border
            # rather than dropped, which keeps this array index-aligned with
            # structured_points.
            camera_points_idxs[0] = np.clip(camera_points_idxs[0], 0, original_height - 1)
            camera_points_idxs[1] = np.clip(camera_points_idxs[1], 0, original_width - 1)

            points_header = deepcopy(pointcloud_msg.header)
            points_header.frame_id = self.target_frame

        # RUN INFERENCE
        model_ids = self._model_registry.get_pinned_ids()
        if extra_models:
            for mid in extra_models:
                if mid not in model_ids:
                    model_ids.append(mid)

        all_entities: List[Tuple[Entity, Shape]] = []
        annotated_image = None
        t_yolo_total = 0.0
        # Snapshot the existing entities before inserting anything, so the
        # deletion below cannot remove entities this frame just added.
        old_entity_ids = self._kb_get_entities_for_deletion()

        for model_id in model_ids:
            model = self._model_registry.get_model(model_id)
            if model is None:
                self.get_logger().warn(f"Model {model_id} unavailable, skipping")
                continue

            t_yolo = time.perf_counter()
            result, num_detections = self._run_yolo_inference(yolo_image, model)
            t_yolo_total += (time.perf_counter() - t_yolo) * 1000

            # Accumulate every model's detections on one frame; must happen
            # before the box rescale below so coordinates match yolo_image.
            if self.visualize:
                annotated_image = result.plot(img=annotated_image)

            if num_detections == 0:
                continue

            # Undo the preprocessing scale so boxes refer to the original image.
            if scale_x != 1.0 or scale_y != 1.0:
                if hasattr(result, "boxes") and result.boxes is not None:
                    if hasattr(result.boxes, "xywh") and result.boxes.xywh is not None:
                        result.boxes.xywh[:, 0] /= scale_x
                        result.boxes.xywh[:, 1] /= scale_y
                        result.boxes.xywh[:, 2] /= scale_x
                        result.boxes.xywh[:, 3] /= scale_y

            masks = self._extract_masks(result, original_height, original_width, mask_hand=mask_hand)

            if structured_points is not None and masks is not None and camera_points_idxs is not None:
                for i, mask in enumerate(masks):
                    # Look up each point's pixel in the mask, giving one flag
                    # per point in structured_points order.
                    point_mask = mask[camera_points_idxs[0], camera_points_idxs[1]]
                    entity_points = structured_points[point_mask > 0.5]

                    if self.get_parameter("use_clustering").get_parameter_value().bool_value:
                        entity_points = self.cluster_entity_points(entity_points)

                    if len(entity_points) == 0:
                        continue

                    entity_pointcloud = array_to_pointcloud2(entity_points)
                    entity_pointcloud.header.stamp = points_header.stamp
                    entity_pointcloud.header.frame_id = points_header.frame_id
                    self.debug_pointcloud_pub.publish(entity_pointcloud)

                    label = result.names[int(result.boxes.cls[i].item())]
                    if label not in self._label_definitions:
                        self.get_logger().warn(f"No label definition for '{label}', defaulting to pickable/unknown")
                    all_entities.append(create_entity(label, entity_points, entity_pointcloud.header, self._label_definitions))

        # VISUALIZE
        if self.visualize and annotated_image is not None:
            self.segmented_image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))

        # UPDATE KNOWLEDGE BASE
        if self._kb_services_available:
            self.kb_add_entities(all_entities)
        if delete_old_entities:
            # An empty frame only clears the KB if that is explicitly wanted;
            # otherwise a single bad frame would wipe still-valid entities.
            if len(all_entities) > 0 or self._clear_db_on_no_detection:
                self._kb_delete_entities(ids=old_entity_ids)

        total_ms = (time.perf_counter() - t_start) * 1000
        self.get_logger().info(
            f"[Timing] Pre:{preprocess_ms:.1f}ms | YOLO:{t_yolo_total:.1f}ms | Total:{total_ms:.1f}ms | "
            f"Models:{len(model_ids)} | Entities:{len(all_entities)}"
        )

    def cluster_entity_points(self, entity_points):
        """Reduce a detection's points to its largest spatially connected cluster.

        A segmentation mask is rarely pixel-perfect, so points from the
        background behind the object are usually included as well. Those points
        are spatially separated from the object and would otherwise drag the
        entity's centroid away from it.

        Args:
            entity_points: Structured point array with "x", "y" and "z" fields.

        Returns:
            The subset of points forming the largest cluster, or an empty list
            if the input was empty or DBSCAN classified everything as noise.
        """
        if len(entity_points) == 0:
            return []
        # 1. Preparing data for sklearn dbscan
        xyz = np.stack([entity_points["x"], entity_points["y"], entity_points["z"]], axis=1)

        # 2. Execute DBSCAN clustering
        # eps = neighborhood radius; min_samples = minimum cluster density.
        #
        # Assumption: point cloud coordinates are in meters (eps=0.01 => 1 cm).
        # Tune these parameters if the point cloud scale changes.
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
        """Fetch the IDs of the pickables currently stored in the knowledge base.

        Only pickables are considered, so furniture and humans survive a
        refresh.

        Returns:
            List[int]: Entity IDs, or an empty list if the KB is unavailable or
                the query failed.
        """
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
        """Delete the given entities from the knowledge base.

        Does nothing if the KB is unavailable or the list is empty. Failures are
        logged but not raised, so a KB problem cannot abort frame processing.

        Args:
            ids: Entity IDs to remove.
        """
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
            self.get_logger().error(f"Failed to delete entities: {del_response.result.error}")

    def kb_add_entities(self, entities: List[Tuple[Entity, Shape]]):
        """Store detected entities and their geometry in the knowledge base.

        Each pair takes two service calls, because a shape can only be attached
        to an entity that already exists: `AddEntity` returns the new ID, which
        `UpdShape` then references. If the first call fails the shape is
        skipped and the next entity is attempted.

        Does nothing if the KB is unavailable. Failures are logged but not
        raised, so a KB problem cannot abort frame processing.

        Args:
            entities: (Entity, Shape) pairs as produced by `create_entity`.
        """
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
                self.get_logger().error(f"Failed to save entity in database: {response.result.error}")
                continue

            # Update shape in KB
            upd_shape_req = UpdShape.Request()
            upd_shape_req.entityid = response.entityid
            upd_shape_req.shape = shape
            upd_shape_req.stamp = shape.pointcloud.header.stamp

            shape_response: UpdShape.Response = self.client_upd_shape.call(upd_shape_req)
            if shape_response.result.result_type != Result.SUCCESS:
                self.get_logger().error(f"Failed to update shape in database: {shape_response.result.error}")
                continue

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Extract camera intrinsics from CameraInfo message.

        The intrinsics are required to project the point cloud into the color
        image; until the first message arrives `_process_data` cannot produce
        entities.

        Args:
            msg: `sensor_msgs/CameraInfo` message with intrinsic matrix K.
        """
        K = np.array(msg.k, dtype=float).reshape(3, 3)
        new_intrinsics = {
            "fx": K[0, 0],
            "fy": K[1, 1],
            "cx": K[0, 2],
            "cy": K[1, 2],
        }

        self._rgb_width = msg.width
        self._rgb_height = msg.height

        # This topic publishes continuously, so only the first message is
        # logged; later ones update the values silently.
        if not self._camera_intrinsics_set:
            self.camera_intrinsics = new_intrinsics
            self._camera_intrinsics_set = True
            self._fx = new_intrinsics["fx"]
            self._fy = new_intrinsics["fy"]
            self._cx = new_intrinsics["cx"]
            self._cy = new_intrinsics["cy"]
            self.get_logger().info(f"Camera intrinsics set: {msg.width}x{msg.height}, fx={K[0, 0]:.1f}, fy={K[1, 1]:.1f}")
        else:
            self.camera_intrinsics = new_intrinsics
            self._fx = new_intrinsics["fx"]
            self._fy = new_intrinsics["fy"]
            self._cx = new_intrinsics["cx"]
            self._cy = new_intrinsics["cy"]

        self.camera_intrinsics_matrix = K

    def _run_yolo_inference(self, rgb_image: np.ndarray, model: Any) -> tuple[Any, int]:
        """Run YOLO inference on RGB image (synchronous).

        Args:
            rgb_image: RGB image as numpy array.
            model: YOLO model instance to use.

        Returns:
            tuple[Any, int]: The YOLO result for the single input image and its
                number of detections.
        """
        results = model(rgb_image, verbose=False, device=self.device)
        result = results[0]
        num_detections = len(result.boxes) if hasattr(result, "boxes") else 0
        return result, num_detections

    def _evict_stale_models(self) -> None:
        """Release models that have been idle for longer than their TTL.

        Timer callback (60 s). The `model_ttl_minutes` parameter is read on each
        call so it can be changed at runtime.
        """
        ttl = self.get_parameter("model_ttl_minutes").get_parameter_value().integer_value * 60.0
        self._model_registry.evict_stale(ttl)

    def _extract_masks(self, result: Any, image_height: int, image_width: int, mask_hand: bool) -> np.ndarray | None:
        """Extract segmentation masks from YOLO result.

        Optimized version: Performs batch resize on GPU before transferring
        to CPU, avoiding multiple CPU-GPU transfers and sequential processing.

        Args:
            result: YOLO result object.
            image_height: Height of the image.
            image_width: Width of the image.
            mask_hand: ignore the hand portion of the image

        Returns:
            np.ndarray | None: Binary masks (N x H x W) scaled to the original
                image size, or None if the result carries no masks - which is
                the case for a detection-only model.
        """
        has_masks = hasattr(result, "masks") and result.masks is not None and len(result.masks) > 0
        if not has_masks:
            return None

        # Keep masks on GPU for batch processing
        masks_gpu = result.masks.data.detach()  # Shape: [N, H, W]

        if masks_gpu.shape[1] != image_height or masks_gpu.shape[2] != image_width:
            masks_resized = resize(
                masks_gpu,
                size=[image_height, image_width],
            )

            # Single sync + transfer to CPU
            result_masks = masks_resized.cpu().numpy().astype(np.uint8)
        else:
            # No resize needed - single sync + transfer
            result_masks = masks_gpu.cpu().numpy().astype(np.uint8)

        # After casting to uint8, mask values are expected to be in {0, 1}.
        if mask_hand:
            n, h, w = result_masks.shape
            # Zero out lower 20% of the image to suppress hand-like detections.
            result_masks[:, int(h * 0.8) : h, :] = 0.0
        return result_masks

    def _snapshot_goal_callback(self, goal_request: VisionSnapshotAction.Goal):
        """Accept every incoming `/vision/snapshot` goal.

        Args:
            goal_request: The requested `VisionSnapshotAction.Goal`.

        Returns:
            GoalResponse: Always ACCEPT. Goals are never rejected here; problems
                are reported through the result instead.
        """
        self.get_logger().info(f"Received goal from Decision Making. Delete old: {goal_request.command.clear_database}")
        return GoalResponse.ACCEPT

    def _snapshot_execute_callback(self, goal_handle):
        """Process one frame on request and report the outcome to the caller.

        The cached frame is discarded first and a fresh one awaited, because the
        camera is mounted on the arm: a buffered frame may still show the pose
        the robot was in before it moved to the observation position.

        Args:
            goal_handle: Handle of the accepted `VisionSnapshotAction` goal,
                carrying the `VisionSnapshotCommand` to execute.

        Returns:
            VisionSnapshotAction.Result: With `response.result` set to
                VisionSnapshotResponse.SUCCESS, ERROR_NO_IMAGE_DATA if no frame
                arrived in time, or ERROR_UNKNOWN if processing raised.

        Notes:
            The goal always succeeds, even on error - failures are communicated
            in the result rather than by aborting the goal.
        """
        with self.vision_data_mutex:
            self.color_image = None
            self.pointcloud = None
        # Wait up to 5s for a frame captured after the arm settled.
        for _ in range(5):
            sleep(1.0)
            with self.vision_data_mutex:
                if self.color_image is not None and self.pointcloud is not None:
                    break

        with self.vision_data_mutex:
            goal_command: VisionSnapshotCommand = goal_handle.request.command
            action_result = VisionSnapshotAction.Result()
            if self.color_image is None or self.pointcloud is None:
                action_result.response.result = VisionSnapshotResponse.ERROR_NO_IMAGE_DATA
                action_result.response.error_msg = "No image data"
                self.get_logger().error(f"Failed to take vision snapshot: {action_result.response.error_msg}.")
                goal_handle.succeed()
                return action_result

            try:
                self._process_data(
                    self.color_image,
                    self.pointcloud,
                    delete_old_entities=goal_command.clear_database,
                    mask_hand=goal_command.mask_hand,
                    extra_models=list(goal_command.extra_models),
                )
                action_result.response.result = VisionSnapshotResponse.SUCCESS
            except Exception as e:
                action_result.response.result = VisionSnapshotResponse.ERROR_UNKNOWN
                action_result.response.error_msg = f"Exception: {e}"
            goal_handle.succeed()
            return action_result


def create_entity(
    label: str,
    structured_points: NDArray,
    points_header: Header,
    label_definitions: dict[str, dict],
) -> Tuple[Entity, Shape]:
    """Convert one detection into the Entity/Shape pair the knowledge base stores.

    The label decides what kind of entity is created: `label_definitions` maps
    it to an EntityType and, depending on that type, either a pickable object
    category or a furniture submessage with its attributes (e.g. marking a
    detected "dishwasher_open" as a dishwasher whose `open` field is true).

    The entity's pose is the centroid of its points and carries no orientation;
    the full geometry is kept in the returned Shape's point cloud.

    Args:
        label: YOLO class name of the detection.
        structured_points: The detection's points, as a structured array with
            "x", "y" and "z" fields, already expressed in `points_header`'s
            frame.
        points_header: Header of the point cloud the points came from; supplies
            the entity's timestamp and reference frame.
        label_definitions: Label mapping as returned by
            `_load_label_definitions`.

    Returns:
        Tuple[Entity, Shape]: The semantic entity and its geometry.

    Notes:
        An unknown label degrades to a pickable of unknown category rather than
        being dropped, so a newly trained model still yields something graspable
        before its labels are added to `config/labels.yaml`.
    """
    defn = label_definitions.get(label)
    if defn is None:
        entity_type_id = EntityType.PICKABLE
        object_category = EntityPickable.OBJECT_CATEGORY_UNKNOWN
        furniture_field = None
        attributes: dict = {}
    else:
        entity_type_id = defn["entity_type_id"]
        object_category = defn["object_category"] or EntityPickable.OBJECT_CATEGORY_UNKNOWN
        furniture_field = defn["furniture_field"]
        attributes = defn["attributes"]

    entity = Entity()
    entity.entity_type.id = entity_type_id
    entity.stamp = points_header.stamp
    entity.description = label
    entity.pose_reference_frame = points_header.frame_id
    entity.pose.position.x = float(np.average(structured_points["x"]))
    entity.pose.position.y = float(np.average(structured_points["y"]))
    entity.pose.position.z = float(np.average(structured_points["z"]))
    entity.pickable.object_name = label
    entity.pickable.object_category = object_category
    if furniture_field is not None:
        # Select the EntityFurniture submessage matching this label's type and
        # apply the attributes configured for it in labels.yaml.
        furniture_msg = getattr(entity.furniture, furniture_field)
        for attr_name, attr_value in attributes.items():
            setattr(furniture_msg, attr_name, attr_value)
    shape = Shape()
    shape.has_pointcloud = True
    shape.pointcloud = array_to_pointcloud2(structured_points)
    shape.pointcloud.header = points_header
    return (entity, shape)


def main(args=None):
    """Entry point for the object_detection node.

    Spins the node on a multi-threaded executor with three threads, one for
    each concurrent activity: the camera data callback, the snapshot execution
    callback, and the action server's own internal callbacks. The split is not
    enforced by the executor, but with fewer threads the node deadlocks: the
    snapshot callback blocks on synchronous KB service calls, whose responses
    can then no longer be delivered.

    Args:
        args: Command line arguments passed to `rclpy.init`.
    """
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)

    try:
        node = ObjectDetection()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
