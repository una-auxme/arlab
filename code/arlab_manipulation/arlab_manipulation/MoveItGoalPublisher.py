#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

# open: calculate pos out of picture with bounding boxes
# open: send gripping object to gripforce service and receive force

class MoveItGoalPublisher(Node):
    def __init__(self):
        super().__init__("MoveItGoalPublisher")
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.gripforce_pub = self.create_publisher(Float64, '/grip_force', 10)
        self.create_timer(2.0, self.publish_goal)
    
    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.pose.position.x = 0.4
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.4
        goal.pose.orientation.w = 1.0

        grip_msg = Float64()
        grip_msg.data = 5.0
        self.gripforce_pub.publish(grip_msg)
        
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f"Goalpose published: Orientation(w={goal.pose.orientation.w}) | Position(x={goal.pose.position.x}, y={goal.pose.position.y}, z={goal.pose.position.z}) | "
            f"Grip force: {grip_msg.data} N"
            )

def main(args=None):
    rclpy.init(args=args)
    node = MoveItGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
