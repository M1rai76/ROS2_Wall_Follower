#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


# class MarkerNavigator(Node):
#     def init(self):
#         super().init('marker_navigator')
#         self.navigator = BasicNavigator()

#         # Load map
#         self.navigator.loadMap('map.yaml')

#         # Load marker data
#         self.markers = self.load_markers('robot_marker_positions.csv')

#     def load_markers(self, filepath):
#         markers = []
#         with open(filepath, 'r') as file:
#             reader = csv.reader(file)
#             next(reader)  # Skip the header line
#             for row in reader:
#                 x, y, marker_type = row[0], row[1], row[2]
#                 markers.append((float(x), float(y), marker_type))
#         return markers


#     def load_markers(self, filepath):
#         markers = []
#         with open(filepath, 'r') as file:
#             reader = csv.reader(file)

#             # Read initial pose from the first line
#             start_pose = next(reader)
#             x, y, yaw = float(start_pose[0]), float(start_pose[1]), float(start_pose[2])

#             # Set initial pose in Nav2
#             initial_pose = PoseStamped()
#             initial_pose.pose.position.x = x
#             initial_pose.pose.position.y = y
#             initial_pose.pose.orientation.z = yaw  # Assuming yaw is in radians

#             self.navigator.setInitialPose(initial_pose)

#             # Load marker positions
#             for row in reader:
#                 x, y, marker_type = float(row[0]), float(row[1]), row[2]
#                 markers.append((x, y, marker_type))

#         return markers

#     def navigate_to_markers(self):
#         waypoints = []
#         for x, y,  in self.markers:
#             pose = PoseStamped()
#             pose.pose.position.x = x
#             pose.pose.position.y = y
#             waypoints.append(pose)

#         self.navigator.followWaypoints(waypoints)

# def main(args=None):
#     rclpy.init(args=args)
#     marker_navigator = MarkerNavigator()
#     marker_navigator.navigate_to_markers()
#     rclpy.spin(marker_navigator)
#     marker_navigator.destroy_node()
#     rclpy.shutdown()


##############################################
############### Samyak's Code ################
##############################################


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

        self.client.send_goal_async(NavigateToPose.Goal(pose=initial_pose))

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # TBC - might not need to specify orientation
        goal_msg.pose.pose.orientation.w = 1.0

        self.client.send_goal_async(goal_msg)

    def navigate_to_markers(self):
        self.set_initial_pose()  # Set the initial position on the map
        for x, y, marker_type in self.markers:
            self.get_logger().info(f"Navigating to marker {marker_type} at ({x}, {y})")
            self.send_goal(x, y)

def main(args=None):
    rclpy.init(args=args)
    navigator = MarkerNavigator()
    navigator.navigate_to_markers()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

