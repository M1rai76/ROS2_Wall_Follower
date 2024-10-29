from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pl_interface',
            executable='pl_interface',
            name='pl_interface'
        ),
        Node(
            package='pl_interface',
            executable='see_marker.py',
            name='see_marker'
        ),
        Node(
            package='pl_interface',
            executable='point_transformer.py',
            name='point_transformer',
        )
    ])
