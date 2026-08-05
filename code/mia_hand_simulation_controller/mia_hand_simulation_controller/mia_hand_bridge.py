import random

import rclpy
from rclpy.node import Node

from moveit.planning import MoveItPy

from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint



class Bridge(Node):

    def __init__(self):
        super().__init__("ur5e_controller")


    


        self.get_logger().info("Waiting for MoveGroup...")

        self.client = ActionClient(
            self,
            MoveGroup,
            "/move_action"
        )


        self.waiting_for_async_finish=False;
        
        self.get_logger().info("Got MoveGroup")

        self.client.wait_for_server()
        # Execute every 5 seconds
        self.timer = self.create_timer(0.1, self.move_robot)

    

   

    def move_robot(self):

        if(self.waiting_for_async_finish==True):
            return
       
        goal = MoveGroup.Goal()

        # Which MoveIt planning group to use
        goal.request.group_name = "mia_hand"

        # Joint goal
        joint_goal = Constraints()

     

        joints = self.get_joint_goal('p',100)

        for name, position in joints.items():
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            joint_goal.joint_constraints.append(constraint)

        goal.request.goal_constraints.append(joint_goal)


        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2

        self.get_logger().info(f"Sending goal {joints}")

        result=self.client.send_goal_async(goal)
        self.waiting_for_async_finish=True

        result.add_done_callback(self.goal_response_callback)



    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        # Ask for the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)


    def result_callback(self, future):

        result = future.result()

        # MoveIt error code
        error_code = result.result.error_code.val 

        if error_code == 1:
            self.get_logger().info("Motion completed successfully")
        else:
            self.get_logger().error(
                f"Motion failed. MoveIt error code: {error_code}"
            )
        
        self.waiting_for_async_finish=False


 

    def get_joint_goal(self, pinch_letter, open_percentage):
        
        '''standard values taken from, page 66: https://www.prensilia.com/wp-content/uploads/2026/05/260324_SERIAL_UserManual_MK2_v1-0.pdf'''
        grip_ids= ['c','p','l','u','d','s','t','r']

        motor_values=[
            [0,50,50,140,240,240],
            [20,170,50,110,255,50],
            [50,-235,230,210,-235,230],
            [70,100,230,70,200,230],
            [20,-180,30,20,-230,30],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0],
            [0,0,0,50,-100,50],
        ]


        motor_ranges=[
            [0,255],
            [0,255],
            [-255,255],
            [0,255],
        ]
        
        grip_id=grip_ids.index(pinch_letter)

        motor_grip_values=motor_values[grip_id]

        
        motor1=self.remap(open_percentage,motor_grip_values[0],motor_grip_values[3],0,100)
        motor2=self.remap(open_percentage,motor_grip_values[1],motor_grip_values[4],0,100)
        motor3=self.remap(open_percentage,motor_grip_values[2],motor_grip_values[5],0,100)




        self.get_logger().info(f"debug {motor_grip_values}")
        self.get_logger().info(f"debug m1 {motor1}")
        self.get_logger().info(f"debug m2 {motor2}")
        self.get_logger().info(f"debug m3 {motor3}")


        r1=self.remap(motor2,0,69,motor_ranges[0][0],motor_ranges[0][1])
        r2=self.remap(motor3,0,69,motor_ranges[1][0],motor_ranges[1][1])
        r3=0
        r4=self.remap(motor1,0,90,motor_ranges[3][0],motor_ranges[3][1])

        

        

        joints = {
            "j_index_fle": float(r1),
            "j_mrl_fle": float(r2),
            "j_thumb_fle": float(r4),
            "j_thumb_opp": float(r3),
        }

        return joints
    
    @staticmethod
    def remap(value, r1_lower,r1_upper,r2_lower,r2_upper):
        r2 =r2_upper - r2_lower
        r1 =r1_upper - r1_lower

        percentage=(value-r2_lower)/r2

        return r1*percentage + r1_lower






def main():
    rclpy.init()

    node = Bridge()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()