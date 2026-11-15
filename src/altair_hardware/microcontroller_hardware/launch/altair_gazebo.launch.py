# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # DEPRECATED: Altair model is deprecated; gazebo now loads Yantravira (yantravira_urdf1).
    import os as _os

    def _workspace_root(start_path):
        cur = _os.path.abspath(start_path)
        for _ in range(8):
            if _os.path.exists(_os.path.join(cur, 'configure.py')):
                return cur
            nxt = _os.path.dirname(cur)
            if nxt == cur:
                break
            cur = nxt
        return cur

    altair_lab_root = _workspace_root(__file__)
    yantra_description_dir = _os.path.join(altair_lab_root, 'src', 'yantravira_urdf1')
    yantra_urdf_path = _os.path.join(yantra_description_dir, 'urdf', 'yantravira_urdf1.urdf')
    with open(yantra_urdf_path, 'r') as f:
        robot_description_content = f.read()
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('yantravira_urdf1'),
            'config',
            'gazebo_sim',
            'controller.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'yantra', '-allow_renaming', 'true',
                   '-x', '0', '-y', '0', '-z', '0.8'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers,
            ],
    )
    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'imu_sensor_broadcaster',
            '--param-file',
            robot_controllers,
            ],
    )

    return LaunchDescription([
        # Make yantravira_urdf1 meshes resolvable in Gazebo
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[altair_lab_root, ':', _os.path.join(altair_lab_root, 'src')]
        ),
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
        node_robot_state_publisher,
        gz_spawn_entity,
        imu_sensor_broadcaster_spawner,
        joint_state_broadcaster_spawner,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
