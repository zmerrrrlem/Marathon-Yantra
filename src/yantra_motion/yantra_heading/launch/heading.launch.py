"""Start heading_controller between the operator/mission command and quintic_walk."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('yantra_heading'), 'config', 'heading.yaml')
    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='yantra_heading',
            executable='heading_controller',
            name='heading_controller',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
    ])
