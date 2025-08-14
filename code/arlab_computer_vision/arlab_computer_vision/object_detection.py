"""ROS2 YOLO-based object detection node with KB integration.

This module defines a ROS2 node that performs real-time object detection
using Ultralytics YOLO segmentation, converts results into semantic/geometric
entities, and communicates with a knowledge base (KB) via ROS services.

Notes:
    - Comments and docstrings follow the Google Python Style Guide:
      https://google.github.io/styleguide/pyguide.html

    - The deletion of entities currently references undefined IDs
      (e.g., `self.entity_id`, `self.cupboard_id`, ...). This mirrors the
      original code and is kept as a TODO to avoid changing behavior.
"""

from __future__ import annotations

import numpy as np
import rclpy
import torch
import cv2

from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs_py import point_cloud2
from ultralytics import YOLO
from std_msgs.msg import Header, String
from vision_msgs.msg import BoundingBox2D, Point2D
from geometry_msgs.msg import Pose, Point, Quaternion

from arlab_knowledge_interfaces.msg import Entity, EntityType
from arlab_knowledge_interfaces.srv import AddEntity, DelEntities, GetEntities


class ObjectDetection(Node):
    """ROS2 node for real-time object detection and KB synchronization.

    The node:
      1) Subscribes to RGB images and camera intrinsics.
      2) Runs YOLO segmentation on incoming frames.
      3) Converts detections to semantic entities (label, bbox, point cloud, pose).
      4) Interacts with a knowledge base through ROS services to insert/update entities.

    Attributes:
        visualize (bool): Whether to display a live visualization window.
        bridge (CvBridge): Bridge to convert ROS <-> OpenCV images.
        model (YOLO): Ultralytics YOLO segmentation model.
        camera_intrinsics (dict[str, float] | None): Focal lengths and principal point.
        service_client_group (MutuallyExclusiveCallbackGroup): Callback group for services.
        prefix (str): Namespace prefix for KB services.
        client_get_entities: ROS service client for fetching entities.
        client_del_entities: ROS service client for deleting entities.
        client_add_entities: ROS service client for adding an entity.
    """

    def __init__(self) -> None:
        """Initialize the node, parameters, subscriptions, and service clients."""
        super().__init__(type(self).__name__)

        # Declare configurable parameters.
        self.declare_parameter("yolo_weights", "yolo_weights/yolo11n-seg.pt")
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter(
            "camera_info_topic", "/camera/aligned_depth_to_color/camera_info"
        )
        self.declare_parameter("visualize", True)

        # Load parameters.
        yolo_weights = (
            self.get_parameter("yolo_weights").get_parameter_value().string_value
        )
        rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        camera_info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.visualize = (
            self.get_parameter("visualize").get_parameter_value().bool_value
        )

        # Init CV bridge and YOLO model.
        self.bridge = CvBridge()
        self.model = YOLO(yolo_weights)
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {device}.")
        self.model.to(device)

        self.camera_intrinsics: dict[str, float] | None = None

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

        # Subscribe to camera info (async; sets intrinsics once available).
        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile=10
        )

        # Subscribe to RGB image stream.
        self.create_subscription(Image, rgb_topic, self.process_data, 10)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Handle CameraInfo and extract intrinsics.

        Args:
            msg: CameraInfo message with intrinsic matrix K.
        """
        # Extract intrinsics from row-major K (3x3).
        K = np.array(msg.K, dtype=float).reshape(3, 3)
        self.camera_intrinsics = {
            "fx": K[0, 0],
            "fy": K[1, 1],
            "cx": K[0, 2],
            "cy": K[1, 2],
        }
        self.get_logger().info("Camera intrinsics set.")

    async def process_data(self, rgb_msg: Image) -> None:
        """Main callback for incoming RGB images.

        Runs YOLO segmentation, builds entity payloads, and interacts with the KB.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.

        Notes:
            - Exits early until camera intrinsics are known.
            - Depth handling is currently a TODO.
            - Uses async service calls for KB operations.
        """
        if self.camera_intrinsics is None:
            # Wait for intrinsics before processing frames.
            return

        self.get_logger().info("Processing image...")

        # Convert ROS image to RGB numpy array.
        rgb_image_bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        rgb_image = cv2.cvtColor(rgb_image_bgr, cv2.COLOR_BGR2RGB)

        # TODO(birk): Integrate depth image handling if available.

        # Run YOLO inference (segmentation).
        results = self.model(rgb_image)
        result = results[0]

        # Convert YOLO results to entity dicts.
        entities_cv = generate_entities_from_yolo_result(
            result=result,
            class_names=self.model.names,
            rgb_header=rgb_msg.header,
            clock=self.get_clock(),
            frame=rgb_image if self.visualize else None,
        )

        # Ensure services are available.
        if not self.client_get_entities.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /get_entities not available.")
            return
        if not self.client_add_entities.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /add_entity not available.")
            return
        if not self.client_del_entities.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /del_entities not available.")
            return

        # Retrieve existing entities from KB.
        get_entities_req = GetEntities.Request()
        get_entities_req.entity_type.id = EntityType.ENTITY
        entities_knowledge_resp: GetEntities.Response = (
            await self.client_get_entities.call_async(get_entities_req)
        )
        self.get_logger().info(f"GetEntities response received.")

        # TODO(birk): Compare `entities_knowledge_resp` with `entities_cv` (e.g., IoU tracking).

        # Delete existing entities from KB (placeholder behavior as in original code).
        # NOTE: The following IDs are not defined in the original snippet.
        # Replace this block with actual IDs or derive them from the GetEntities response.
        del_entities_req = DelEntities.Request()
        del_entities_req.entityids = [
            # TODO(birk): Replace with actual IDs from knowledge base or tracking.
            # self.entity_id,
            # self.cupboard_id,
            # self.door_id,
            # self.furniture_id,
            # self.human_id,
            # self.pickable_id,
            # self.shelf_id,
            # self.table_id,
        ]
        if del_entities_req.entityids:
            del_resp = await self.client_del_entities.call_async(del_entities_req)
            self.get_logger().info(f"DelEntities result: {del_resp.result}.")
        else:
            self.get_logger().warn(
                "DelEntities skipped: no entity IDs provided (placeholder TODO)."
            )

        # Insert detected entities into KB.
        for entity_cv in entities_cv:
            add_entity_req = AddEntity.Request()
            add_entity_req.data = Entity(
                description=f"Detected: {entity_cv['name'].data}",
                pose=entity_cv["pose"],
                pose_reference_frame=rgb_msg.header.frame_id,
                stamp=self.get_clock().now().to_msg(),
            )
            add_resp = await self.client_add_entities.call_async(add_entity_req)
            self.get_logger().info(f"AddEntity response received.")

        self.get_logger().info("Frame processing complete.")

    # ---------------------------------------------------------------------


def pose_from_point2d(point2d: Point2D) -> Pose:
    """Create a Pose with XY from a 2D point and fixed orientation.

    Args:
        point2d: 2D point (e.g., bounding box center) in image coordinates.

    Returns:
        Pose: Pose with x=point2d.x, y=point2d.y, z=0 and identity orientation.
    """
    return Pose(
        position=Point(x=point2d.x, y=point2d.y, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def generate_entities_from_yolo_result(
    result, class_names, rgb_header: Header, clock, frame: np.ndarray | None = None
) -> list[dict]:
    """Convert YOLO segmentation outputs into entity dictionaries.

    Each entity includes:
      - Class label (`std_msgs/String`).
      - 2D bounding box (`vision_msgs/BoundingBox2D`).
      - 2D point cloud projected to z=0 (as `sensor_msgs/PointCloud2`).
      - Estimated pose (`geometry_msgs/Pose`) derived from bbox center.

    Args:
        result: YOLO result object for a single image.
        class_names: Mapping from class indices to names (list or dict).
        rgb_header: Header from the input RGB image.
        clock: ROS clock used to timestamp generated messages.
        frame: Optional RGB frame for live visualization. If None, no GUI.

    Returns:
        list[dict]: A list of dictionaries with keys:
            `name` (String), `pointcloud` (PointCloud2), `bbox` (BoundingBox2D), `pose` (Pose).
    """
    masks = result.masks
    boxes = result.boxes

    if masks is None or boxes is None:
        return []

    class_ids = boxes.cls.cpu().numpy().astype(int)
    masks_np = masks.data.cpu().numpy()

    entities: list[dict] = []

    for i, mask in enumerate(masks_np):
        coords_2d = np.argwhere(mask > 0)
        if coords_2d.size == 0:
            continue

        # Class label as ROS String.
        class_name_msg = String(data=class_names[class_ids[i]])

        # Build bounding box (XYWH from YOLO -> center + size).
        xywh = boxes[i].xywh[0].cpu().tolist()
        center_x, center_y, width, height = xywh
        bbox = BoundingBox2D()
        bbox.center = Point2D(x=float(center_x), y=float(center_y))
        bbox.size_x = float(width)
        bbox.size_y = float(height)

        # Header for point cloud.
        header = Header()
        header.stamp = clock.now().to_msg()
        header.frame_id = rgb_header.frame_id

        # Convert mask pixel coordinates to a sparse z=0 point cloud.
        pc2_msg = coords2d_to_pointcloud(coords_2d, header)

        # Derive pose at bbox center (z=0, identity orientation).
        pose_msg = pose_from_point2d(bbox.center)

        new_entity = {
            "name": class_name_msg,
            "pointcloud": pc2_msg,
            "bbox": bbox,
            "pose": pose_msg,
        }
        entities.append(new_entity)

        # Optional live visualization.
        if frame is not None:
            color = (0, 255, 0)
            alpha = 0.4

            # Resize mask to frame resolution for overlay.
            mask_resized = cv2.resize(
                mask.astype(np.uint8),
                (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

            # Color overlay for mask.
            colored_mask = np.zeros_like(frame, dtype=np.uint8)
            colored_mask[mask_resized > 0] = color
            overlay = cv2.addWeighted(frame, 1.0, colored_mask, alpha, 0)

            # Bounding box rectangle.
            x = int(center_x - width / 2)
            y = int(center_y - height / 2)
            w = int(width)
            h = int(height)
            cv2.rectangle(overlay, (x, y), (x + w, y + h), color, 2)

            # Class label text.
            label = class_names[class_ids[i]]
            cv2.putText(
                overlay,
                label,
                (x, max(0, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

            # Show window (non-blocking).
            cv2.imshow("YOLO Segmentation", overlay)
            cv2.waitKey(1)

    return entities


def coords2d_to_pointcloud(coords_2d: np.ndarray, header: Header):
    """Convert 2D pixel coordinates to a sparse z=0 PointCloud2.

    Downsamples points to reduce message size and CPU load.

    Args:
        coords_2d: N×2 array of (row, col) pixel indices where mask > 0.
        header: ROS Header for the resulting PointCloud2.

    Returns:
        sensor_msgs.msg.PointCloud2: Sparse point cloud with z=0.
    """
    # Downsample every 10th pixel for efficiency.
    coords_2d = coords_2d[::10]

    points = []
    for v, u in coords_2d:
        # Note: u -> x (column), v -> y (row).
        x = float(u)
        y = float(v)
        z = 0.0
        points.append([x, y, z])

    pc2_msg = point_cloud2.create_cloud_xyz32(header, points)
    return pc2_msg


def main(args=None) -> None:
    """Entry point for the ROS2 node lifecycle.

    Args:
        args: Optional CLI args forwarded to rclpy.init.
    """
    rclpy.init(args=args)
    node = ObjectDetection()
    try:
        rclpy.spin(node)
    finally:
        # Ensure clean shutdown even on exceptions.
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
