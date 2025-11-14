"""Knowledge Visualization Node

This node visualizes knowledge entities as markers in RViz.

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from typing import List, Optional

import rclpy
from arlab_common.exceptions import emsg_with_trace
from arlab_common.markers import debug_marker_array
from arlab_common.parameters import update_attributes
from arlab_knowledge_interfaces.msg import Result
from arlab_knowledge_interfaces.srv import GetEntities, GetEntity
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from visualization_msgs.msg import Marker, MarkerArray

from arlab_knowledge.db.entities.entity import Entity

prefix = "/arlab/knowledge"
"""ROS prefix/namespace for all services
"""


class KnowledgeVisualization(Node):
    """Node that visualizes entities saved int the knowledge base as markers in RViz.

    It periodically:
        - retrieves all entities from the knowledge base
        - converts them to markers
        - publishes the markers as a MarkerArray message to RViz

    Attributes:
        update_timer_period (float): Timer period for updating markers.
        marker_publisher (Publisher): Publisher for MarkerArray messages.
        service_cb (MutuallyExclusiveCallbackGroup): Callback group for services.
        get_entities_client: Client for GetEntities service.
        get_entity_client: Client for GetEntity service.
        timer: Timer for periodic updates.
    """

    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        self.update_timer_period = (
            self.declare_parameter("update_timer_period", 0.5)
            .get_parameter_value()
            .double_value
        )

        # Setup publishers
        self.marker_publisher: Publisher = self.create_publisher(
            MarkerArray, f"{prefix}/visualization", qos_profile=1
        )

        # Setup services
        self.service_cb = MutuallyExclusiveCallbackGroup()
        self.get_entities_client = self.create_client(
            GetEntities, f"{prefix}/get_entities", callback_group=self.service_cb
        )
        self.get_entity_client = self.create_client(
            GetEntity, f"{prefix}/get_entity", callback_group=self.service_cb
        )
        for client in self.clients:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"Waiting for {client.srv_name} service...")

        self.timer = self.create_timer(self.update_timer_period, self.timer_callback)
        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Handles parameter updates.

        This callback checks if the update_timer_period parameter has changed.
        If it has, the existing timer is canceled and a new timer is created with the
        updated period

        Args:
            params (List[Parameter]): List of parameters to update.
        Returns:
            SetParametersResult: Result of the parameter update.
        """
        old_update_timer_period = self.update_timer_period
        result = update_attributes(self, params)
        if old_update_timer_period != self.update_timer_period:
            # Recreate timer
            self.timer.cancel()
            self.timer.destroy()
            self.timer = self.create_timer(
                self.update_timer_period, self.timer_callback
            )
        return result

    async def timer_callback(self):
        """Timer callback to update markers periodically."""
        try:
            await self.update_markers()
        except Exception as e:
            self.get_logger().fatal(emsg_with_trace(e), throttle_duration_sec=2)

    async def get_markers(self) -> List[Marker]:
        """Retrieve all entities from the knowledge base and convert them to markers.

        Workflow:
            1. Call GetEntities service to get all entity IDs.
            2. For each entity ID, call GetEntity service to get entity data.
            3. Convert each entity to markers and collect them.

        Returns:
            List[Marker]: List of markers representing the entities.

        Error Handling:
            - Logs an error if the service response is None or unsuccessful.
        """
        # Step 1: Get all entity IDs
        req = GetEntities.Request()
        result: Optional[
            GetEntities.Response
        ] = await self.get_entities_client.call_async(req)

        if result is None:
            self.get_logger().error("Service response was None")
            return []
        if result.result.result_type != Result.SUCCESS:
            self.get_logger().error(f"Service call unsuccessful: {result.result.error}")
            return []

        markers = []
        # Step 2: Get each entity and convert to markers
        for entity_id in result.entities:
            req = GetEntity.Request(entityid=entity_id)
            entity_rsp: Optional[
                GetEntity.Response
            ] = await self.get_entity_client.call_async(req)

            if entity_rsp is None:
                self.get_logger().error("Service response was None")
                continue
            if entity_rsp.result.result_type != Result.SUCCESS:
                self.get_logger().error(
                    f"Service call unsuccessful: {entity_rsp.result.error}"
                )
                continue
            # Step 3: Convert entity to markers
            entity = Entity.from_ros_msg(entity_rsp.data)
            markers += entity.get_all_markers()

        return markers

    async def update_markers(self):
        """Update and publish markers to RViz.
        Retrieves markers from the knowledge base and publishes them as a
        MarkerArray message.
        """
        marker_array = debug_marker_array(
            namespace="knowledge",
            markers=await self.get_markers(),
            timestamp=self.get_clock().now().to_msg(),
        )
        self.marker_publisher.publish(marker_array)
        self.get_logger().info("Published marker array")


def main(args=None):
    rclpy.init(args=args)

    knowledge_visualization_node = KnowledgeVisualization()

    rclpy.spin(knowledge_visualization_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
