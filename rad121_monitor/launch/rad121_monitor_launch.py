from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rad121_monitor',
            executable='rad121_monitor_node',
            name='rad121_monitor_node',
            output='screen'
        )
    ])

