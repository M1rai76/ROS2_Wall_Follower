#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
from nav_msgs.msg import Odometry
import math
# import tf_transformations

class MarkerNavigator(Node):
    def __init__(self):
        super().__init__('marker_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Read markers and starting pose
        # self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.set_initial_pose, 10)
        # self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.get_logger().info("Subscribed to /odom topic")

        self.start_pose, self.markers = self.load_markers('/home/rsa/colcon_ws/src/wall_follower/scripts/robot_marker_positions.csv')
        self.set_initial_pose()

    def odom_callback(self, msg):
        # Extract the robot's position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z  # Usually 0 for a 2D robot

        # Extract the robot's orientation (quaternion)
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion(orientation_q)

        # Log the current position and yaw
        self.get_logger().info(f"Position: x={x}, y={y}, z={z}, yaw={yaw}")

    def euler_from_quaternion(self, quat):
        """Converts a quaternion into Euler angles (roll, pitch, yaw)."""
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        # Roll (x-axis rotation)
        sinr_cosp = +2.0 * (w * x + y * z)
        cosr_cosp = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = +2.0 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) <= 1.0 else math.copysign(math.pi / 2, sinp)

        # Yaw (z-axis rotation)
        siny_cosp = +2.0 * (w * z + x * y)
        cosy_cosp = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # def set_initial_pose(self, msg):
    #     self.get_logger().info(f'HERE2')
    #     yaw = euler_from_quaternion(msg.pose.pose.orientation)
        
    #     self.initial_pose.pose.position.x = msg.pose.pose.position.x
    #     self.initial_pose.pose.position.y = msg.pose.pose.position.y
    #     self.initial_pose.pose.orientation.w = yaw  # orientation might be z

    #     self.get_logger().info(f'Initial pose set to x: {self.initial_pose.pose.position.x}, y: {self.initial_pose.pose.position.y}, yaw: {yaw}')
    #     # self.client.wait_for_server()
        
    #     # time.sleep(1)

    #     # Unsubscribe if only needed once
    #     # self.destroy_subscription(self.subscription)
    #     self.navigate_to_markers

    def load_markers(self, filepath):
        markers = []
        with open(filepath, 'r') as file:
            reader = csv.reader(file)
            start_x, start_y, start_heading = map(float, next(reader))
            start_pose = (start_x, start_y, start_heading)
            
            for row in reader:
                x, y, marker_type = float(row[0]), float(row[1]), row[2]
                markers.append((x, y, marker_type))
                
        return start_pose, markers

    def set_initial_pose(self):
        # Set initial pose based on the start position and heading
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.position.x = self.start_pose[0]
        initial_pose.pose.position.y = self.start_pose[1]
        initial_pose.pose.orientation.w = self.start_pose[2]

        # Publish to `/initialpose`
        self.get_logger().info(f"Setting initial pose: {self.start_pose[0]}, {self.start_pose[1]}, {self.start_pose[2]}")
        self.navigate_to_markers()
        
        # self.client.wait_for_server()
        
        # time.sleep(1)

        #self.client.send_goal_async(NavigateToPose.Goal(pose=initial_pose), feedback_callback=self.feedback_callback)
    
    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Identity quaternion for no rotation

        # Wait for the action server to be ready
        self.get_logger().info("Waiting for action server to be ready...")
        self.client.wait_for_server(timeout_sec=5.0)
        
        if not self.client.server_is_ready():
            self.get_logger().error('Action server is not ready!')
            return

        self.get_logger().info("Sending goal...")
        self._send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        # self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
    
    def navigate_to_markers(self):
        # self.set_initial_pose()  # Set the initial position on the map
        i = 0
        for x, y, marker_type in self.markers:
            if (i == 1): break
            self.get_logger().info(f"Navigating to marker {marker_type} at ({x}, {y})")
            self.send_goal(x, y)
            i += 1

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args)
    navigator = MarkerNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()