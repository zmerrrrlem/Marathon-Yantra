import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('yantravira_urdf1')
    gz_pkg = get_package_share_directory('ros_gz_sim')
    urdf = os.path.join(pkg, 'urdf', 'yantravira_urdf1.urdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf', 'on_exit_shutdown': 'true'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf]),
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_yantravira',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'yantravira_urdf1',
            '-z', '0.30',
        ],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([gazebo, clock_bridge, robot_state_publisher, spawn_robot])
