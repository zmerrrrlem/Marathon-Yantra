import launch
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch_ros.substitutions
import os


def generate_launch_description():
    controller_params = os.path.join(
        get_package_share_directory('altair_teleop'), 'config/gamepad_config.yaml'
    )
    
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'deadzone': 0.1, 'autorepeat_rate': 10.0}]
    )

    joy_to_twist_node = launch_ros.actions.Node(
        package='altair_teleop',
        executable='altair_teleop',
        name='altair_teleop',
        parameters=[
            {'type': LaunchConfiguration('type')},
            controller_params
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'type', default_value='custom', description='Sets the controller type e.g. custom, xbox'
        ),
        joy_node,
        joy_to_twist_node
    ])
