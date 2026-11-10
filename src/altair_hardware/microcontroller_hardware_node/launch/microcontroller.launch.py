from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='microcontroller_hardware_node',
            executable='microcontroller_hardware_node',
            name='microcontroller_hardware_node',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baudrate': 115200}
            ]
        )
    ])