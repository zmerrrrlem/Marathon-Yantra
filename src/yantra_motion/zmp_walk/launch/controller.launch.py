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

import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from xacro import process_file
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot = "yantra"
    robot_name = ""
    robot_namespace_arg = DeclareLaunchArgument(
        "robot",
        default_value=robot_name,
        description="Namespace for the robot",
    )
    namespace = LaunchConfiguration("robot")

    yantra_description = get_package_share_directory('yantra_description')
    robot_xacro_path = os.path.join(yantra_description, 'urdf', 'yantra_compose.urdf.xacro')
    robot_description = process_file(robot_xacro_path, mappings={
        "real_mode": "false",
        "dxl_unavailable": "true",
        "imu_unavailable": "true",
        "mock_mode": "true",
        "sim_mode": "false",
        "tuning_mode": "true",

        "dynamixel_port": "/dev/u2d2",
        "dynamixel_baudrate": "1000000",
        "stm32_port": "/dev/ttyUSB0",
        "stm32_baudrate": "115200",
    }).toxml()

    # Launch Arguments
    zmp_walk_params = PathJoinSubstitution(
        [
            FindPackageShare('zmp_walk'),
            'config',
            'walking.yaml',
        ]
    )
    controller_param = PathJoinSubstitution(
        [
            FindPackageShare('zmp_walk'),
            'config',
            'controller.yaml',
        ]
    )

    robot_state_publisher = Node(
        namespace=namespace,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description}],
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
    zmp_walk_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zmp_walk", "--param-file", zmp_walk_params],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"],
    )

    return LaunchDescription([
        robot_namespace_arg,
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        zmp_walk_spawner,
        imu_sensor_broadcaster_spawner
    ])
