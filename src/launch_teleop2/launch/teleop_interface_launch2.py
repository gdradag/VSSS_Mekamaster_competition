from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='launch_teleop2',
            executable='teleop_interface2',
            name='teleop_interface2',
            output='screen'
        ),
        Node(
            package='launch_teleop2',
            executable='teleop_node2',
            name='teleop_node2',
            output='screen'
        ),
        Node(
            package='launch_teleop2',
            executable='teleop_Snode2',
            name='teleop_Snode2',
            output='screen'
        )
    ])
