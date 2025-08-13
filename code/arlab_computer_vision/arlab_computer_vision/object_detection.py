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
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D
from vision_msgs.msg import Point2D
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion

from arlab_knowledge_interfaces.msg import Entity
from arlab_knowledge_interfaces.msg import EntityType
from arlab_knowledge_interfaces.srv import (
    AddEntity,
    DelEntities,
    GetEntities,
)


class ObjectDetection(Node):
    """ROS2 node that performs real-time object detection using YOLO,
    extracts semantic and geometric information from masks, and communicates
    with a knowledge base via ROS services.
    """

    def __init__(self):
        super().__init__(type(self).__name__)

        self.bridge = CvBridge()
        self.model = YOLO("yolo_weights/yolo11n-seg.pt")
        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using device: {device}")
        self.model.to(device)
        self.camera_intrinsics = None

        # Service Clients
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

        # Subscribe to camera info (once, async)
        self.create_subscription(
            CameraInfo,
            "/camera/aligned_depth_to_color/camera_info",
            self.camera_info_callback,
            qos_profile=10,
        )

        # Subscribe to RGB image stream
        self.create_subscription(Image, "/camera/image_raw", self.process_data, 10)

    def camera_info_callback(self, msg):
        # Extract camera intrinsics from K matrix
        K = np.array(msg.K).reshape(3, 3)
        self.camera_intrinsics = {
            "fx": K[0, 0],
            "fy": K[1, 1],
            "cx": K[0, 2],
            "cy": K[1, 2],
        }

    async def process_data(self, rgb_msg: Image):
        """Main callback for processing incoming RGB images.
        Performs YOLO segmentation, generates semantic and geometric entity data,
        and communicates with the knowledge base to insert or update entities.
        """
        if self.camera_intrinsics is None:
            return

        print("\n-------------------------------------------------------------")
        print("Processing Image.")

        rgb_header = rgb_msg.header
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

        # TODO => Depth image handling should be done here.

        # depth_image = self.bridge.imgmsg_to_cv2(
        #    depth_msg, desired_encoding="passthrough"
        # )
        # depth_np = depth_image.astype(np.float32)  # z. B. 16UC1 in mm

        # YOLOv8-Inferenz
        results = self.model(rgb_image)
        result = results[0]

        entities_cv = generate_entities_from_yolo_result(
            result, self.model.names, rgb_header, self.get_clock(), frame=rgb_image
        )

        # 1. Generate new entities from current detections
        # Each entity consists of:
        # - class label (tag)
        # - segmented point cloud
        # - bounding box and pose

        # 2. Retrieve all existing entities from the knowledge base
        get_entities_req = GetEntities.Request()
        get_entities_req.entity_type.id = EntityType.ENTITY
        entities_knowledge_resp: GetEntities.Response = (
            await self.client_get_entities.call_async(get_entities_req)
        )

        print("\nGet Entities: ", entities_knowledge_resp, "\n")

        # entity_knowledge_ids = entities_knowledge_resp.entities

        # 3. Compare existing entities to new detections (e.g., for tracking via IoU)
        # TODO: Implement comparison between entities_knowledge and entities_cv

        # 4. Delete all existing entities from the knowledge base
        del_entities_req = DelEntities.Request()
        del_entities_req.entityids = [
            self.entity_id,
            self.cupboard_id,
            self.door_id,
            self.furniture_id,
            self.human_id,
            self.pickable_id,
            self.shelf_id,
            self.table_id,
        ]
        res = self.call_service(
            DelEntities, f"{self.prefix}/del_entities", del_entities_req
        )
        self.log_result("DelEntities", res.result)

        print("Del Entities: ", res.result, "\n")

        # 5. Insert or update detected entities into the knowledge base
        for entity_cv in entities_cv:
            add_entity_req = AddEntity.Request()
            add_entity_req.data = Entity(
                description="Testing Knowledgebase Services.",
                pose=entity_cv["pose"],
                pose_reference_frame=rgb_header.frame_id,
                stamp=self.get_clock().now().to_msg(),
            )

            add_resp = await self.client_add_entities.call_async(add_entity_req)
            print("Add Entities: ", add_resp, "\n")


def pose_from_point2d(point2d: Point2D) -> Pose:
    """Convert a 2D point (typically from bounding box center)
    into a 3D pose with fixed orientation."""
    return Pose(
        position=Point(x=point2d.x, y=point2d.y, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def generate_entities_from_yolo_result(
    result, class_names, rgb_header, clock, frame=None
):
    """Convert YOLO segmentation results into a list of entity dictionaries.
    Each entity contains:
    - class label as std_msgs/String
    - bounding box (vision_msgs/BoundingBox2D)
    - 2D point cloud (as fake 3D PointCloud2 with z=0)
    - estimated pose (geometry_msgs/Pose)
    """

    masks = result.masks
    boxes = result.boxes

    if masks is None or boxes is None:
        return []

    class_ids = boxes.cls.cpu().numpy().astype(int)
    masks_np = masks.data.cpu().numpy()

    entities = []

    for i, mask in enumerate(masks_np):
        coords_2d = np.argwhere(mask > 0)
        if coords_2d.size == 0:
            continue

        class_name_msg = String(data=class_names[class_ids[i]])

        xywh = boxes[i].xywh[0].cpu().tolist()
        center_x, center_y, width, height = xywh
        bbox = BoundingBox2D()
        bbox.center = Point2D(x=center_x, y=center_y)
        bbox.size_x = width
        bbox.size_y = height

        header = Header()
        header.stamp = clock.now().to_msg()
        header.frame_id = rgb_header.frame_id

        pc2_msg = coords2d_to_pointcloud(coords_2d, header)

        pose_msg = pose_from_point2d(bbox.center)

        new_entity = {
            "name": class_name_msg,
            "pointcloud": pc2_msg,
            "bbox": bbox,
            "pose": pose_msg,
        }

        entities.append(new_entity)

        # 🎯 Live-Visualisierung
        if frame is not None:
            color = (0, 255, 0)
            alpha = 0.4

            # 🟢 Maske in Farbe überlagern
            colored_mask = np.zeros_like(frame, dtype=np.uint8)
            # Skaliere Maske auf Bildgröße (same height/width as frame)
            mask_resized = cv2.resize(
                mask.astype(np.uint8),
                (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

            # Erstelle leere Maske in Bildgröße
            colored_mask = np.zeros_like(frame, dtype=np.uint8)
            colored_mask[mask_resized > 0] = color
            overlay = cv2.addWeighted(frame, 1.0, colored_mask, alpha, 0)

            # 🟥 Bounding Box zeichnen
            x = int(center_x - width / 2)
            y = int(center_y - height / 2)
            w = int(width)
            h = int(height)
            cv2.rectangle(overlay, (x, y), (x + w, y + h), color, 2)

            # 🏷️ Label schreiben
            label = class_names[class_ids[i]]
            cv2.putText(
                overlay, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
            )

            # Zeige das Bild
            cv2.imshow("YOLO Segmentation", overlay)
            cv2.waitKey(1)

    return entities


def coords2d_to_pointcloud(coords_2d, header: Header):
    """Convert 2D pixel coordinates to a fake 3D point cloud by setting z=0.
    Useful for debug visualization or simplified geometry reasoning.
    """
    points = []

    for v, u in coords_2d:
        x = float(u)  # column
        y = float(v)  # row
        z = 0.0  # no depth available
        points.append([x, y, z])

    pc2_msg = point_cloud2.create_cloud_xyz32(header, points)
    return pc2_msg


def main(args=None):
    rclpy.init(args=args)
    my_ros2_node = ObjectDetection()
    rclpy.spin(my_ros2_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
