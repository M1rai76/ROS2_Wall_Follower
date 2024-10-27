#!/usr/bin/env python3

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import PointStamped # type: ignore
import tf2_ros # type: ignore
import tf2_geometry_msgs # type: ignore
from visualization_msgs.msg import Marker # type: ignore
from visualization_msgs.msg import MarkerArray # type: ignore

import wall_follower.landmark
from wall_follower.landmark import marker_type, max_markers, Landmark

class PointTransformer(Node):

	def __init__(self):
		super().__init__('point_transformer')
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
		self.point_subscriber = self.create_subscription(PointStamped, '/marker_position', self.point_callback, 10) # listen to points being published by system and all the transofrms thats being publish
		self.marker_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)#

		self.marker_position = []
		self.marker_array = MarkerArray()			
		self.marker_array.markers = []

		for i in range(max_markers): # create an array of eg. 6 different array types 
			self.marker_position.append(Landmark(i, self.marker_array.markers))


	def point_callback(self, msg):
		try:
			# Lookup the transform from the camera_rgb_optical_frame to the map frame
			transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time()) # msg is the point stamp from vision system , the header tells us which 2
		except tf2_ros.LookupException as e: # co-odrinates it belongs to , it gives us the matrix when wwe clal it down in the do_transofrm 
			self.get_logger().error('Transform lookup failed: %s' % str(e))
			return

		which_marker = int(msg.point.z) 
		m = marker_type[which_marker]
		msg.point.z = 0.0

		# Transform the point from camera_rgb_optical_frame to map frame
		map_point = tf2_geometry_msgs.do_transform_point(msg, transform)# applying the transform to xyz marker

		# Print the transformed point in the map frame
#		self.get_logger().info(f'Mapped {m} marker to /map frame: x={map_point.point.x}, y={map_point.point.y}, z={map_point.point.z}')

		self.marker_position[which_marker].update_position(map_point.point)
		self.marker_publisher_.publish(self.marker_array)


def main(args=None):
	rclpy.init(args=args)
	node = PointTransformer()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

