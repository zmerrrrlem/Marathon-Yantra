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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions


def generate_launch_description():

    

    # Launch Arguments
    cubic_walk_params = PathJoinSubstitution(
        [
            FindPackageShare('b_splines_walk'),
            'config',
            'walking.yaml',
        ]
    )
    controller_param = PathJoinSubstitution(
        [
            FindPackageShare('b_splines_walk'),
            'config',
            'controller.yaml',
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_param],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    cubic_walk_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["b_splines_walk", "--param-file", cubic_walk_params],
    )

    microcontroller_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["microcontroller_broadcaster"],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"],
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        cubic_walk_spawner,
        imu_sensor_broadcaster_spawner
    ])
