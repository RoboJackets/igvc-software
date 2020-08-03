from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            node_executable='joy_node',
            node_name='joy_node',
            output='screen',
            parameters = [
                {"dev", "/dev/igvc_joystick"}
            ]
        ),
        Node(
            package='igvc_platform',
            node_executable='joystick_driver',
            node_name='joystick_driver',
            output='screen'
        )
    ])