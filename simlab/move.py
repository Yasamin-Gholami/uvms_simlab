import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
from robot import Robot
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from uvms_interfaces.msg import Command

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_to_goal',
                          automatically_declare_parameters_from_overrides=True)

        # Get parameter values
        self.no_robot = self.get_parameter('no_robot').value
        self.no_efforts = self.get_parameter('no_efforts').value
        self.robots_prefix = self.get_parameter('robots_prefix').value
        self.record = self.get_parameter('record_data').value
        self.controllers = self.get_parameter('controllers').value
        self.get_logger().info(f"robots controllers : {self.controllers}")

        self.get_logger().info(f"robot prefixes found in task node: {self.robots_prefix}")
        self.total_no_efforts = self.no_robot * self.no_efforts
        self.get_logger().info(f"robots total number of commands : {self.total_no_efforts}")
        
        initial_pos = np.array([0.0, 0.0, 8.0, 0,0,0, 3.1, 0.7, 0.4, 2.1])


        self.robots = [Robot(self, k, 4, prefix, initial_pos, self.record) for k, prefix in enumerate(self.robots_prefix)]
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.uvms_publisher_ = self.create_publisher(Command, '/uvms_controller/uvms/commands', qos_profile)


        # Timer to control movement
        frequency = 150  # Hz
        self.timer = self.create_timer(1.0 / frequency, self.move_robot)
        self.get_logger().info("MoveTask node has been initialized.*********************************")

        # Goal position
        self.goal_x = 2.0
        self.goal_y = 2.0
        self.goal_z = 2.0
        self.goal_roll = 0.0
        self.goal_pitch = 0.0
        self.goal_yaw = 0.0

    def move_robot(self):

        command_msg = Command()
        command_msg.command_type = self.controllers
        command_msg.acceleration.data = []
        command_msg.twist.data = []
        command_msg.pose.data = []

        for robot in self.robots:
            state = robot.get_state()
            if state['status']=='active':
                sim_t = state['sim_time']
                sim_dt = state['dt']
                pose = state['pose']
                self.get_logger().info(f"MoveTask node state are {pose}")
                command_msg.acceleration.data.extend([0.0]*11)

                command_msg.twist.data.extend([0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0])

                command_msg.pose.data.extend([self.goal_x, self.goal_y, self.goal_z, self.goal_roll, self.goal_pitch, self.goal_yaw, 3.1, 0.7, 0.4, 2.1, 0.0])

                # self.get_logger().info(f"MoveTask node state are {state}")
            # self.get_logger().info("MoveTask node is running.")
            robot.write_data_to_file()
        self.uvms_publisher_.publish(command_msg)
                
                    
        # """Move the robot towards the goal using simple proportional control."""
        # if self.position is None:
        #     return

        # x, y = self.position.x, self.position.y
        # distance = math.sqrt((self.goal_x - x) ** 2 + (self.goal_y - y) ** 2)

        # if distance < 0.1:  # Stop when close to goal
        #     self.get_logger().info("Goal reached!")
        #     self.cmd_vel_pub.publish(Twist())  # Stop the robot
        #     return

        # # Compute direction
        # angle_to_goal = math.atan2(self.goal_y - y, self.goal_x - x)

        # # Send velocity command
        # twist = Twist()
        # twist.linear.x = min(0.5 * distance, 0.2)
        # twist.angular.z = 2.0 * angle_to_goal  # Simple turn control
        # self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()