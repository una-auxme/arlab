from arlab_template_interfaces.srv import AddEntity
import rclpy
from rclpy.node import Node


class KnowledgeClient(Node):
    """This nodes calls a service from a timer

    It demonstrates the importance of Callback groups
    when using blocking service requests.

    Blocking in this async case means: blocks the further
    execution of the async callback.
    """

    def __init__(self):
        super().__init__(type(self).__name__)

        self.add_entity_client = self.create_client(AddEntity, "add_entity")
        while not self.add_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    async def add_entity(self, entity_id):
        self.req = AddEntity.Request()
        self.req.id = int(entity_id)

    async def send_request(self):
        try:
            result = await self.add_entity_client.call_async(self.req)
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")
        else:
            if result is None:
                self.get_logger().error("Service response was None")
                return
            self.get_logger().info(f"Result: {result}")


def main(args=None):
    rclpy.init(args=args)

    my_ros2_client = KnowledgeClient()

    rclpy.spin(my_ros2_client)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
