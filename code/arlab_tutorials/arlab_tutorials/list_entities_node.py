"""ROS2 node that lists all entity IDs from the knowledge base.

This node uses the GetEntities service from arlab_knowledge to retrieve
all entity IDs stored in the database and then exits.

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from threading import Thread

import rclpy
from arlab_knowledge_interfaces.msg import EntityType, Result
from arlab_knowledge_interfaces.srv import GetEntities
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node


class ListEntitiesNode(Node):
    """ROS2 node for listing all entity IDs from the knowledge base.

    The node:
        1) Creates a client for the GetEntities service.
        2) Calls the service to retrieve all entity IDs.
        3) Prints the entity IDs to the console.
        4) Exits after completion.

    Attributes:
        client_get_entities: Service client for fetching entities.
        service_client_group: Callback group for services.
        prefix: Namespace prefix for KB services.
    """

    def __init__(self) -> None:
        """Initialize the node, parameters, and service clients."""
        super().__init__("list_entities_node")

        # Service clients.
        self.service_client_group = MutuallyExclusiveCallbackGroup()
        self.prefix = "/arlab/knowledge"

        self.client_get_entities = self.create_client(
            GetEntities,
            f"{self.prefix}/get_entities",
            callback_group=self.service_client_group,
        )

        # Wait for service to be available
        for client in self.clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"{client.service_name} service not available, waiting...")

        self.get_logger().info("GetEntities service is available.")

    def _get_all_entities(self) -> None:
        """Call GetEntities service to retrieve all entity IDs.

        Retrieves entities for all available types and prints their IDs.
        """
        self.get_logger().info("Fetching all entity IDs from the database...")

        # Create request
        request = GetEntities.Request()
        request.entity_type = EntityType(id=EntityType.ENTITY)  # All kinds of entity

        # Call service
        response: GetEntities.Response = self.client_get_entities.call(request)

        # Check result
        if response.result.result_type == Result.SUCCESS:
            if response.entities:
                self.get_logger().info(f"{len(response.entities)} entities found")
                self.get_logger().info(f"Entity ids: {response.entities}")
            else:
                self.get_logger().info("No entities found")
        else:
            self.get_logger().error(f"Error fetching entities: {response.result.error}")

    def run(self) -> None:
        """Run the node and execute the main task."""
        self._get_all_entities()
        self.get_logger().info("All entity IDs have been listed. Exiting.")


def main() -> None:
    """Main entry point for the list entities node."""
    rclpy.init()
    node = ListEntitiesNode()

    def spin():
        rclpy.spin(node)

    spin_thread = Thread(target=spin)
    spin_thread.start()

    node.run()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
