#!/usr/bin/env python3

# 1. working out distance ang angles to markers - simulation camera is perfect, real life 
# camera is cheap and have distortions , make adjustments to calculations 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import wall_follower.landmark
from wall_follower.landmark import marker_type, max_markers, Landmark
import csv

class PointTransformer(Node):

	def __init__(self):
		super().__init__('point_transformer')
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
		self.point_subscriber = self.create_subscription(PointStamped, '/marker_position', self.point_callback, 10)
		self.nav_complete_sub = self.create_subscription(Bool, 'nav_complete', self.nav_complete_callback, 10)
		self.marker_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

		self.marker_position = []
		self.marker_array = MarkerArray()			
		self.marker_array.markers = []

		for i in range(max_markers):
			self.marker_position.append(Landmark(i, self.marker_array.markers))


	def point_callback(self, msg):
		try:
			# Lookup the transform from the camera_rgb_optical_frame to the map frame
			transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
		except tf2_ros.LookupException as e:
			self.get_logger().error('Transform lookup failed: %s' % str(e))
			return

		which_marker = int(msg.point.z)
		m = marker_type[which_marker]
		msg.point.z = 0.0

		# Transform the point from camera_rgb_optical_frame to map frame
		map_point = tf2_geometry_msgs.do_transform_point(msg, transform)

		# Print the transformed point in the map frame
		# self.get_logger().info(f'Mapped {m} marker to /map frame: x={map_point.point.x}, y={map_point.point.y}, z={map_point.point.z}')

		self.marker_position[which_marker].update_position(map_point.point)
		self.marker_publisher_.publish(self.marker_array)


	def nav_complete_callback(self, msg):
		if msg.data:
			self.save_marker_positions()


	def save_marker_positions(self):
		csv_file_path = "positions.csv"
		with open(csv_file_path, mode='a', newline='') as csv_file:
			writer = csv.writer(csv_file)
			for i, landmark in enumerate(self.marker_position):
				x = landmark.top_marker.pose.position.x
				y = landmark.top_marker.pose.position.y
				writer.writerow([x, y, marker_type[i]])
		self.get_logger().info(f"Marker positions saved to {csv_file_path}")


def main(args=None):
	rclpy.init(args=args)
	node = PointTransformer()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()