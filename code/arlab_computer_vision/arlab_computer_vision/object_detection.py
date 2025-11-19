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

import os

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from arlab_knowledge_interfaces.msg import Entity, EntityType
from arlab_knowledge_interfaces.srv import AddEntity, DelEntities, GetEntities
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion
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

        # Load parameters.
        yolo_weights = (
            self.get_parameter("yolo_weights").get_parameter_value().string_value
        )
        self.visualize = (
            self.get_parameter("visualize").get_parameter_value().bool_value
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
            CameraInfo,
            "camera_info",
            self.camera_info_callback,
            qos_profile=10,
        )

        # Subscribe to RGB image stream.
        self.create_subscription(Image, "camera_color_image", self.process_data, 10)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Extract camera intrinsics from CameraInfo message.

        Args:
            msg: CameraInfo message with intrinsic matrix K.
        """
        K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.camera_intrinsics = {
            "fx": K[0, 0],
            "fy": K[1, 1],
            "cx": K[0, 2],
            "cy": K[1, 2],
        }
        self.get_logger().info("Camera intrinsics set.")

    async def process_data(self, rgb_msg: Image) -> None:
        """Process incoming RGB images and sync detections with KB.

        Args:
            rgb_msg: Incoming RGB `sensor_msgs/Image`.

        Notes:
            - Returns early if camera intrinsics are unknown.
            - Depth handling is currently a TODO.
            - Uses async service calls for KB operations.
        """

        # comment in for real camera usage
        # if self.camera_intrinsics is None:
        #     return

        self.get_logger().info("Processing image...")

        # Convert ROS image to RGB numpy array.
        rgb_image = self.bridge.imgmsg_to_cv2(
            rgb_msg,
            desired_encoding="bgr8",
        )

        # TODO: Integrate depth image handling if available.

        # Run YOLO inference (segmentation/detection).
        results = self.model(rgb_image)
        result = results[0]

        # Convert YOLO results to entity dicts.
        entities_cv = generate_entities_from_yolo_result(
            result=result,
            class_names=self.model.names,
            frame=rgb_image if self.visualize else None,
            use_segmentation=self.use_segmentation,
        )

        # Debug: Zeige, wie viele Objekte erkannt wurden
        object_names = [e["name"].data for e in entities_cv]
        self.get_logger().info(f"Detected {len(entities_cv)} objects: {object_names}")

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

        # Retrieve existing entities from KB (optional, currently unused).
        get_entities_req = GetEntities.Request()
        get_entities_req.entity_type.id = EntityType.ENTITY
        # resp: GetEntities.Response = await self.client_get_entities.call_async(
        #     get_entities_req
        # )
        self.get_logger().info("GetEntities response received.")

        # Insert (or upsert) detected entities into KB.
        for entity_cv in entities_cv:
            add_entity_req = AddEntity.Request()
            add_entity_req.data = Entity(
                description=f"Detected: {entity_cv['name'].data}",
                pose=entity_cv["pose"],
                pose_reference_frame=rgb_msg.header.frame_id,
                stamp=self.get_clock().now().to_msg(),
            )
            add_resp = await self.client_add_entities.call_async(add_entity_req)
            self.get_logger().info(f"AddEntity response received: {add_resp}")


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

    # For segmentation models, we can optionally access masks
    masks = None
    if use_segmentation:
        masks = getattr(result, "masks", None)

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

        # Optional visualization.
        if frame is not None:
            x1 = int(cx - w / 2.0)
            y1 = int(cy - h / 2.0)
            x2 = int(cx + w / 2.0)
            y2 = int(cy + h / 2.0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # For segmentation, optionally draw mask overlay
            if use_segmentation and masks is not None:
                # Masks can be accessed via masks.data[i] if needed in the future
                pass

            cv2.putText(
                frame,
                label,
                (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

    if frame is not None:
        cv2.imshow("YOLO Detections", frame)
        cv2.waitKey(1)

    return entities


def main(args=None):
    """Entry point for the object_detection node."""
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
