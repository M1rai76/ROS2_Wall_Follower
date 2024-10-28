import rclpy
from rclpy.node import Node
import csv
from geometrymsgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class MarkerNavigator(Node):
    def init(self):
        super().init('marker_navigator')
        self.navigator = BasicNavigator()

        # Load map
        self.navigator.loadMap('map.yaml')

        # Load marker data
        self.markers = self.load_markers('robot_marker_positions.csv')

    def load_markers(self, filepath):
        markers = []
        with open(filepath, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header line
            for row in reader:
                x, y, marker_type = row[0], row[1], row[2]
                markers.append((float(x), float(y), marker_type))
        return markers


    def load_markers(self, filepath):
        markers = []
        with open(filepath, 'r') as file:
            reader = csv.reader(file)

            # Read initial pose from the first line
            start_pose = next(reader)
            x, y, yaw = float(start_pose[0]), float(start_pose[1]), float(start_pose[2])

            # Set initial pose in Nav2
            initial_pose = PoseStamped()
            initial_pose.pose.position.x = x
            initial_pose.pose.position.y = y
            initial_pose.pose.orientation.z = yaw  # Assuming yaw is in radians

            self.navigator.setInitialPose(initial_pose)

            # Load marker positions
            for row in reader:
                x, y, marker_type = float(row[0]), float(row[1]), row[2]
                markers.append((x, y, marker_type))

        return markers

    def navigate_to_markers(self):
        waypoints = []
        for x, y,  in self.markers:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            waypoints.append(pose)

        self.navigator.followWaypoints(waypoints)

def main(args=None):
    rclpy.init(args=args)
    marker_navigator = MarkerNavigator()
    marker_navigator.navigate_to_markers()
    rclpy.spin(marker_navigator)
    marker_navigator.destroy_node()
    rclpy.shutdown()