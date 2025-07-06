#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String
from arlab_common_interfaces.srv import GrippingForce

class MoveItGoalPublisher(Node):
    def __init__(self):
        super().__init__("MoveItGoalPublisher")
        
        # Publisher for pose and gripping force
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.gripforce_pub = self.create_publisher(Float64, '/grip_force', 10)

        # Subscriber for new objekt (in future from computer vision)
        self.object_sub = self.create_subscription(String, '/target_object', self.object_callback, 10)

        # Service Client for GrippingForce
        self.client = self.create_client(GrippingForce, 'GetGrippingForce')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GrippingForce servie...')

        self.req = GrippingForce.Request()

        # init force
        self.force = 5.0

        # Timer for publishing pose and gripping force
        self.publish_timer = self.create_timer(2.0, self.publish_goal)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.pose.position.x = 0.4
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.4
        goal.pose.orientation.w = 1.0

        force_msg = Float64()
        force_msg.data = self.force  
        self.gripforce_pub.publish(force_msg)
        
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f"Goalpose published: Orientation(w={goal.pose.orientation.w}) | Position(x={goal.pose.position.x}, y={goal.pose.position.y}, z={goal.pose.position.z}) | "
            f"Grip force: {force_msg.data} N"
            )

    def object_callback(self, msg):
        object_name = msg.data.strip()
        self.get_logger().info(f"New Object: '{object_name}'")

        # pause publishing
        self.publish_timer.cancel()
        self.get_logger().info(f"Pose publishing paused")

        # request gripping force
        self.req.object_name = object_name
        future = self.client.call_async(self.req)
        future.add_done_callback(lambda f: self.handle_service_response(f, object_name))

    def handle_service_response(self, future, object_name):
        try:
            response = future.result()
            self.force = response.grip_force
            self.get_logger().info(
                f"Set new gripping force for objekt: '{object_name}': {self.force:.1f} N"
            )
        except Exception as e:
            self.get_logger().error(f"service-call failed: {e}")
            self.force = 5.0

        # start timer
        self.publish_timer.reset()
        self.get_logger().info(f"Pose publishing started")

def main(args=None):
    rclpy.init(args=args)
    node = MoveItGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


