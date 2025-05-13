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
        self.get_logger().info("MoveTask node has been initialized.")

        #Define multiple goal points sequentially
        self.goals = [
                    np.array([2.0, 2.0, 2.0, 0.0, 0.0, 0.0, 3.1, 0.7, 0.4, 2.1,0.0]),
                    np.array([4.0, -2.0, 3.0, 0.0, 0.0, 0.0, 3.1, 0.7, 0.4, 2.1,0.0]),
                    np.array([-1.0, 3.0, 2.0, 0.0, 0.0, 0.0, 3.1, 0.7, 0.4, 2.1,0.0])
                ]
        self.current_goal_index = 0

    # def move_robot(self):

    #     command_msg = Command()
    #     command_msg.command_type = self.controllers
    #     command_msg.acceleration.data = []
    #     command_msg.twist.data = []
    #     command_msg.pose.data = []

    #     for robot in self.robots:
    #         state = robot.get_state()
    #         if state['status']=='active':
    #             sim_t = state['sim_time']
    #             sim_dt = state['dt']
    #             pose = state['pose']
    #             # self.get_logger().info(f"MoveTask node state are {pose}")
    #             command_msg.acceleration.data.extend([0.0]*11)

    #             command_msg.twist.data.extend([0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #             current_goal=self.goals[self.current_goal_index].tolist()

    #             command_msg.pose.data.extend(current_goal)

    #             # Check if robot reached the goal
    #             distance = np.linalg.norm(np.array(pose[:3]) - np.array(current_goal[:3]))
    #             if distance < 0.1:  # Threshold to consider goal reached
    #                 self.get_logger().info(f"Goal {self.current_goal_index + 1} reached!")
    #                 self.current_goal_index += 1
    #                 self.current_goal_index = self.current_goal_index % 3

    #     self.uvms_publisher_.publish(command_msg)
 
    def move_robot(self):
        command_msg = Command()
        command_msg.command_type = self.controllers
        command_msg.acceleration.data = []
        command_msg.twist.data = []
        command_msg.pose.data = []


        # Continue moving towards the next goal
        for robot in self.robots:
            state = robot.get_state()
            if state['status'] == 'active':
                sim_t = state['sim_time']
                sim_dt = state['dt']
                pose = state['pose']
                q = state['q']
                
                # Check if all goals are reached
                if self.current_goal_index >= len(self.goals):
                    self.get_logger().info("All goals reached! Stopping movement.")
                    # Send stop command by setting zero velocities and acceleration
                    for robot in self.robots:
                        command_msg.twist.data = [0.0] * 11  # Stop the robot
                        command_msg.acceleration.data = [0.0] * 11  # Stop the robot
                        command_msg.pose.data.extend(pose + q + [0.0])
                else:
                    command_msg.acceleration.data.extend([0.0] * 11)
                    command_msg.twist.data.extend([0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0])

                    current_goal = self.goals[self.current_goal_index].tolist()
                    command_msg.pose.data.extend(current_goal)
                    robot.publish_robot_path()
                    # Check if robot reached the goal
                    distance = np.linalg.norm(np.array(pose[:3]) - np.array(current_goal[:3]))
                    if distance < 0.1:  # Threshold to consider goal reached
                        self.get_logger().info(f"Goal {self.current_goal_index + 1} reached!")
                        self.current_goal_index += 1  # Move to the next goal
            robot.write_data_to_file()
        self.uvms_publisher_.publish(command_msg)               

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()