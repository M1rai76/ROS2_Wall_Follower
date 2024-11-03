#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class MarkerNavigator(Node):
    def __init__(self):
        super().__init__('marker_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Read markers and starting pose
        self.start_pose, self.markers = self.load_markers('/home/rsa/colcon_ws/src/wall_follower/scripts/robot_marker_positions.csv')

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
        self.get_logger().info("Setting initial pose")
        self.client.wait_for_server()
        
        time.sleep(1)

        self.client.send_goal_async(NavigateToPose.Goal(pose=initial_pose), feedback_callback=self.feedback_callback)

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.client.wait_for_server()
        time.sleep(1)

        self._send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
    
    def navigate_to_markers(self):
        self.set_initial_pose()  # Set the initial position on the map
        for x, y, marker_type in self.markers:
            self.get_logger().info(f"Navigating to marker {marker_type} at ({x}, {y})")
            self.send_goal(x, y)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args)

    navigator = MarkerNavigator()

    navigator.navigate_to_markers()

    rclpy.spin(navigator)

    navigator.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

