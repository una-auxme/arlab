import rclpy
import numpy as np
import message_filters
import std_msgs.msg
import rospy

from sensor_msgs_py import point_cloud2
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from arlab_template_interfaces.srv import (
    AddEntity,
    GetEntities,
    DelEntities,
    UpdEntities,
)
from arlab_knowledge_interfaces.msg import EntityType
from knowledge_client import KnowledgeClient


class ObjectDetection(Node):
    """
    ROS2-Node: Abonniert RGB + Depth, führt YOLOv8-Segmentierung aus und
    (Platzhalter) bereitet die 3D-Auswertung vor.
    """

    def __init__(self):
        super().__init__(type(self).__name__)

        self.model = YOLO("yolo_weights/yolo11n-seg.pt")
        self.model.to("cuda")
        self.camera_intrinsics = None

        # Service Clients
        self.service_client_group = MutuallyExclusiveCallbackGroup()
        self.prefix = "/arlab/knowledge"

        self.client_get_entities = self.create_client(
            GetEntities,
            f"{self.prefix}/get_entities",
            callback_group=self.service_client_group,
        )

        # Kamera-Info abonnieren (einmalig, async)
        self.create_subscription(
            CameraInfo,
            "/camera/aligned_depth_to_color/camera_info",
            self.camera_info_cb,
            queue_size=1,
        )

        # Synchrone Subscriptions für RGB & Tiefe
        rgb_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_sub = message_filters.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.process_data)

    def camera_info_callback(self, msg):
        # Extrahiere Kamera-Intrinsic Matrix
        K = np.array(msg.K).reshape(3, 3)
        self.camera_intrinsics = {
            "fx": K[0, 0],
            "fy": K[1, 1],
            "cx": K[0, 2],
            "cy": K[1, 2],
        }

    def process_data(self, rgb_msg, depth_msg):
        """Empfängt synchronisierte RGB- und Tiefenbilder, führt YOLOv8-Segmentierung durch
        und berechnet 3D-Koordinaten pro Maske mit zugehörigem Klassennamen.
        """
        if self.camera_intrinsics is None:
            rospy.logwarn_throttle(5, "Noch keine Kameraintrinsik verfügbar.")
            return

        # Konvertiere Bilder
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough"
        )
        depth_np = depth_image.astype(np.float32)  # z. B. 16UC1 in mm

        # YOLOv8-Inferenz
        results = self.model(rgb_image)
        result = results[0]
        masks = result.masks
        boxes = result.boxes

        if masks is None or boxes is None:
            return

        # Kameraparameter
        fx, fy = self.camera_intrinsics["fx"], self.camera_intrinsics["fy"]
        cx, cy = self.camera_intrinsics["cx"], self.camera_intrinsics["cy"]

        # Hole Klassen-IDs + Namen
        class_ids = boxes.cls.cpu().numpy().astype(int)  # shape: (N,)
        class_names = self.model.names  # dict: {id: name}

        # Hole Masken als NumPy (N, H, W)
        masks_np = masks.data.cpu().numpy()

        for i, mask in enumerate(masks_np):
            class_id = class_ids[i]
            class_name = class_names[class_id]

            # Binarisiere Maske
            mask_binary = mask > 0.5

            # Wende Maske auf Tiefenbild an
            depth_masked = np.where(mask_binary, depth_np, 0)

            # Finde gültige Tiefenpunkte
            valid_y, valid_x = np.nonzero(depth_masked)
            z = depth_masked[valid_y, valid_x] / 1000.0  # mm → m

            if z.size == 0:
                continue

            # Vektorisierte Umprojektion
            x = valid_x.astype(np.float32)
            y = valid_y.astype(np.float32)

            X = (x - cx) * z / fx
            Y = (y - cy) * z / fy
            Z = z

            points_3d = np.stack((X, Y, Z), axis=-1)

            print(
                f"[Maske {i}] Klasse: {class_name} ({class_id}) → {len(points_3d)} 3D-Punkte extrahiert."
            )

        # Todo: Save classified data in knowledge base.

        # 1. Create Entities from object detection data
        """
            Tag_name = Klasse
            segmentierte punktwolke
            winkel
        """

        entities_knowledge = None

        # 2. Get all Entities
        entities_req = GetEntities.Request()
        entities_req.entity_type.id = EntityType.ENTITY
        entities_knowledge = await self.client_get_entities.call_async(entities_req)
        entities_knowledge = entities_knowledge.result()

        # 3. Compare Entities for tracking (IoU)
        # Todo enities_knowledge mit entities_cv

        # 4. Delete all Entities
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
        res = await self.call_service(
            DelEntities, f"{self.prefix}/del_entities", del_entities_req
        )
        self.log_result("DelEntities", res.result)

        # 5. Update / Insert Entities
        for entity in entities_cv:
            add_entity_req = AddEntity.Request()
            add_entity_req.data = entity.to_ros_msg()
            # For shelves:
            add_entity_req.data.furniture.shelf.cupboard_id = self.cupboard_id

            response = await self.call_service(
                AddEntity, f"{self.prefix}/add_entity", add_entity_req
            )

        pass


def main(args=None):
    bridge = CvBridge()

    rclpy.init(args=args)
    my_ros2_node = ObjectDetection()
    rclpy.spin(my_ros2_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
