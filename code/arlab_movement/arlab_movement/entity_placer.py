"""Interactive entity placer node
Node for manually placing entities (POIs) for semantic annotation of map / env, using rviz for pose selection on map and params for entity args.

Maintainers:
Luca Kahlenberg <luca.kahlenberg@uni-a.de>
"""
import rclpy
from arlab_knowledge_interfaces.msg import Entity, EntityType, Result
from arlab_knowledge_interfaces.srv import AddEntity
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped

class EntityPlacer(Node):
    """
    TODO doc
    """
    def __init__(self):
        super().__init__(type(self).__name__)

        # declare parameters

        self.declare_parameter("pose_topic", "/arlab/entity_pose")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("entity_type_id", 0)
        self.declare_parameter("description", "new_poi")

        self.declare_parameter("add_entity_service", "/arlab/knowledge/add_entity")
        self.declare_parameter("commit_service", "/arlab/entity_placer/commit")

        # set self vars

        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.add_entity_service = self.get_parameter("add_entity_service").get_parameter_value().string_value
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

        self._commit_service_name = self.get_parameter("commit_service").get_parameter_value().string_value

        self.create_service(
            Trigger,
            self._commit_service_name,
            self._commit_callback,
            callback_group=self.trigger_group,
        )

        # log info

        self.get_logger().info(
            f"Staging poses from '{self.pose_topic}'.\n"
            f"Commit via '{self._commit_service_name}'.\n"
            f"TODO implement tf: (Entities should be added in frame '{self.target_frame}')."
        )


    def _pose_callback(self, msg:PoseStamped):
        """
        TODO doc
        """
        # save pose
        pose = self._to_target_fram(msg)
        if pose is None:
            return

        self._pending_pose = pose
        self.get_logger().info(f"Received and staged new pose: {pose}")


    async def _commit_callback(self, request:Trigger.Request, response:Trigger.Response) -> Trigger.Response:
        """
        TODO doc
        """
        if self._pending_pose is None:
            response.success = False
            response.message = "No pose staged"
            return response

        # create entity from pose

        entity = Entity()
        entity.entity_type = EntityType(id=self.get_parameter("entity_type_id").get_parameter_value().integer_value)
        entity.description = self.get_parameter("description").get_parameter_value().string_value
        entity.pose = self._pending_pose.pose
        entity.pose_reference_frame = self._pending_pose.header.frame_id
        entity.stamp = self._pending_pose.header.stamp

        # call add_entity service

        client = self.create_client(AddEntity, self.add_entity_service)

        if not client.wait_for_service(timeout_sec=self.service_timeout):
            response.success = False
            response.message = "service not available"
            return response

        req = AddEntity.Request()
        req.data = entity
        res = await client.call_async(req)

        if res.result.result_type != Result.SUCCESS:
            response.success = False
            response.message = f"Failed to add entity: {res.result.error}"
            return response

        response.success = True
        response.message = f"Entity added successfully with id: {res.entityid}"
        self.get_logger().info(response.message)
        return response


    def _to_target_fram(self, msg:PoseStamped) -> PoseStamped:
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
    """ Entry point for node"""
    rclpy.init(args=args)
    node = EntityPlacer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
