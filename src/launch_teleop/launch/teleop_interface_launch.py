from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='launch_teleop',
            executable='teleop_interface',
            name='teleop_interface',
            output='screen'
        ),
        Node(
            package='launch_teleop',
            executable='teleop_node',
            name='teleop_node',
            output='screen'
        ),
        Node(
            package='launch_teleop',
            executable='teleop_Snode',
            name='teleop_Snode',
            output='screen'
        )
    ])
