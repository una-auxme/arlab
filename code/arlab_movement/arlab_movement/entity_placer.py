"""Interactive entity placer node
Node for manually placing entities (POIs) for semantic annotation of map / env,
using rviz for pose selection on map and extension of AddEntity.srv for metadata.

Maintainers:
Luca Kahlenberg <luca.kahlenberg@uni-a.de>
"""

import rclpy
from arlab_knowledge_interfaces.msg import Result
from arlab_knowledge_interfaces.srv import AddEntity
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped


class EntityPlacer(Node):
    """Node that stages rviz pose clicks and offers a service to add a new entity with staged pose to knowledge base

    We need the possibility to add entities manually to the knowledge base, to enable semantic annotation of map.
    (kitchen, laundry area) This node is the fallback if automatic annotation via CV fails.

    Workflow:
    - Set topic of 2D Goal Pose in rviz to /arlab/entity_pose using Panel Tool Properties
    - Click on displayed /map in rviz to select pose
    - Call /arlab/entity_placer/place service to add entity (pose, stamp, reference_frame are already set)
    - Display entities in rviz using topic /arlab/knowledge/visualization
    """

    def __init__(self):
        super().__init__(type(self).__name__)

        # declare parameters

        self.declare_parameter("pose_topic", "/arlab/entity_pose")
        self.declare_parameter("target_frame", "map")

        self.declare_parameter("add_entity_service", "/arlab/knowledge/add_entity")
        self.declare_parameter("place_service", "/arlab/entity_placer/place")

        # set self vars

        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.add_entity_service = self.get_parameter("add_entity_service").get_parameter_value().string_value
        self._place_service_name = self.get_parameter("place_service").get_parameter_value().string_value
        self._pending_pose = None
        self.service_timeout = 5.0

        # callback groups

        self.subscription_group = MutuallyExclusiveCallbackGroup()
        self.trigger_group = MutuallyExclusiveCallbackGroup()

        # sub to pose topic

        self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_callback,
            qos_profile=10,
            callback_group=self.subscription_group,
        )

        # create services

        self.create_service(
            AddEntity,
            self._place_service_name,
            self._place_callback,
            callback_group=self.trigger_group,
        )

        # log info

        self.get_logger().info(
            f"Staging poses from '{self.pose_topic}'.\n"
            f"Place Entity / Add to DB via '{self._place_service_name}'.\n"
            f"Entities are added in frame '{self.target_frame}')."
        )

    def _pose_callback(self, msg: PoseStamped):
        """
        Callback for /arlab/entity_pose topic, transforms pose to target_frame and stages pose for later use.
        """
        # save pose
        pose = self._to_target_frame(msg)
        if pose is None:
            return

        self._pending_pose = pose
        self.get_logger().info(f"Received and staged new pose: {pose}")

    async def _place_callback(self, request: AddEntity.Request, response: AddEntity.Response) -> AddEntity.Response:
        """
        Callback for /arlab/entity_placer/place service.
        Extends AddEntity service by using staged pose, reference_frame and stamp already set by staged pose.
        Calls AddEntity service to add entity to knowledge base and returns result.
        """
        if self._pending_pose is None:
            response.result.result_type = Result.ERROR_INVALID_INPUT
            response.result.error = "No pose staged"
            return response

        pose = self._pending_pose

        request.data.pose = pose.pose
        request.data.pose_reference_frame = pose.header.frame_id
        request.data.stamp = pose.header.stamp

        # call add_entity service

        client = self.create_client(AddEntity, self.add_entity_service)

        if not client.wait_for_service(timeout_sec=self.service_timeout):
            response.result.result_type = Result.ERROR_BUSY
            response.result.error = "service not available"
            return response

        try:
            res = await client.call_async(request)
        except Exception as e:
            response.result.result_type = Result.ERROR_DBAPI
            response.result.error = f"AddEntity sercive call failed: {e}"
            return response

        response.entityid = res.entityid
        response.result = res.result

        if res.result.result_type == Result.SUCCESS:
            self.get_logger().info(f"Entity added successfully with id: {res.entityid}")
        else:
            self.get_logger().error(f"Failed to add entity: {res.result.error}")

        return response

    def _to_target_frame(self, msg: PoseStamped) -> PoseStamped:
        """
        transforms any PoseStamped to target frame (map)
        """
        if msg.header.frame_id == self.target_frame:
            return msg
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.service_timeout),
            )
            return do_transform_pose_stamped(msg, transform)
        except Exception as e:
            self.get_logger().error(f"Failed to transform pose to target frame: {e}")
            return None


def main(args=None):
    """Entry point for node"""
    rclpy.init(args=args)
    node = EntityPlacer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
