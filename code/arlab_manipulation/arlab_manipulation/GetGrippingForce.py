#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from arlab_manipulation.srv import GetGrippingForce

class GripForceService(Node):
    def __init__(self):
        super().__init__('grip_force_service')
        self.srv = self.create_service(GetGrippingForce, 'GetGrippingForce', self.handle_grip_force_request)
        self.get_logger().info("Grip force service is ready.")

        # Greifkraft in Newton (N)
        self.force_profiles = {
            "leicht": ["banane", "joghurt", "avocado", "toast", "paprika", "chips"],
            "mittel": ["müsli", "tomate", "cola", "wasser", "nudeln"],
            "fest": ["milch", "apfel", "salz", "dose", "konserve", "reis", "mehl"]
        }

        self.force_values = {
            "leicht": 5.0,
            "mittel": 10.0,
            "fest": 15.0
        }

    def handle_grip_force_request(self, request, response):
        obj = request.object_name.lower()
        force = self.determine_force(obj)
        response.grip_force = force
        self.get_logger().info(f"Object '{obj}' assigned grip force: {force} N")
        return response

    def determine_force(self, object_name):
        for category, objects in self.force_profiles.items():
            if object_name in objects:
                return self.force_values[category]
        return self.force_values["mittel"]  # fallback

def main(args=None):
    rclpy.init(args=args)
    node = GripForceService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()