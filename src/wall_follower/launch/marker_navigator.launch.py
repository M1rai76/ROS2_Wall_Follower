from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='marker_navigator',  # Without .py
            name='marker_navigator_node',
            output='screen',
        ),
    ])
