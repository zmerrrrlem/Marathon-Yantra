from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Modify the default port
    port_arg = DeclareLaunchArgument('port', default_value='5005')

    port = LaunchConfiguration('port')

    return LaunchDescription([
        port_arg,

        Node(
            name='vision_receiver_node',
            package='vision_bridge',
            executable='vision_bridge_receiver',
            output='screen',
            arguments=[port]
        )
    ])

