#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arlab_common_interfaces.action import ManipulationAction

class test(Node):
    def __init__(self):
        super().__init__('test')
        self._client = ActionClient(self, ManipulationAction, '/manipulation_action')

    def send_test_goal(self):
        goal_msg = ManipulationAction.Goal()
        goal_msg.command.command_type = 'close'
        goal_msg.command.target_entityid = 2
        self._client.wait_for_server()
        self.get_logger().info('Sending goal...')
        future = self._client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # Zugriff auf response.message
        self.get_logger().info(f"Result message: '{result.response.message}'")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = test()
    node.send_test_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
