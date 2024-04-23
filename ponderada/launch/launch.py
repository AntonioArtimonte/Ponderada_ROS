from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        Node(
            package='draw_circle',
            executable='draw_circle',
            name='circle',
            prefix = 'gnome-terminal --',
            output='screen'
        )
    ])
