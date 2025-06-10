from arlab_template_interfaces.srv import AddThreeInts
import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class MyRos2Client(Node):
    """This nodes calls a service from a timer

    It demonstrates the importance of Callback groups
    when using blocking service requests.

    Blocking in this async case means: blocks the further
    execution of the async callback.
    """

    def __init__(self):
        super().__init__(type(self).__name__)
        self.cb = MutuallyExclusiveCallbackGroup()

        self.int_client = self.create_client(AddThreeInts, "add_three_ints")
        while not self.int_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.req = AddThreeInts.Request()
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])

        # Important: callbacks that block on a
        # service request (self.int_client.call_async(self.req))
        # must not be in the same callback group as the service client
        self.create_timer(0.5, self.timer_callback, self.cb)

    async def timer_callback(self):
        await self.send_request()

    async def send_request(self):
        try:
            result = await self.int_client.call_async(self.req)
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")
        else:
            if result is None:
                self.get_logger().error("Service response was None")
                return
            self.get_logger().info(
                f"Result of add_three_ints: for {self.req.a} "
                + f"+ {self.req.b} + {self.req.b} = {result.sum}"
            )


def main(args=None):
    rclpy.init(args=args)

    my_ros2_client = MyRos2Client()

    rclpy.spin(my_ros2_client)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
