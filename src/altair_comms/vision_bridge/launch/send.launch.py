from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Modify the ip to the motion board static IP
    ip_arg = DeclareLaunchArgument('ip', default_value='127.0.0.1')
    port_arg = DeclareLaunchArgument('port', default_value='5005')

    ip = LaunchConfiguration('ip')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        ip_arg,
        port_arg,

        Node(
            package='vision_bridge',
            executable='vision_bridge_sender',
            name='vision_sender_node',
            output='screen',
            arguments=[ip, port]
        )
    ])
