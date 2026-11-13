#!/usr/bin/env python3
"""Launch file for Altair odometry fuser using robot_localization EKF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # EKF config file path
    ekf_config = PathJoinSubstitution([
        FindPackageShare('altair_odometry_fuser'),
        'config',
        'ekf.yaml'
    ])

    # EKF node - fuses walk engine odometry with IMU
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
    ])
