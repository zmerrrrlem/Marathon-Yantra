import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from xacro import process_file
import os

def generate_launch_description():
    gazebo_default = False
    gazebo_robot_name_default = 'altair'
    offset_file_path_default = get_package_share_directory('altair_manager') + '/config/altair_offset.yaml'
    robot_file_path_default = get_package_share_directory('altair_manager') + '/config/altair.robot'
    init_file_path_default = get_package_share_directory('altair_manager') + '/config/dxl_init_altair.yaml'
    altair_description = get_package_share_directory('altair_description') + '/urdf/altair.urdf.xacro'
    robot_description = process_file(altair_description, mappings={'use_sim': 'false'}).toxml()
    device_name_default = '/dev/ttyUSB0'
    microcontroller_port = '/dev/ttyACM0'

    return LaunchDescription([
        # Node(
        #     package="robot_state_publisher",
        #     executable="robot_state_publisher",
        #     name="robot_state_publisher",
        #     output="both",
        #     parameters=[{
        #         'robot_description': robot_description,
        #         'use_sim_time': True
        #     }],
        # ),
        Node(
            package='altair_manager',
            executable='altair_manager',
            output='screen',
            parameters=[{
                'angle_unit': 30.0,
                'gazebo': gazebo_default,
                'gazebo_robot_name': gazebo_robot_name_default,
                'offset_file_path': offset_file_path_default,
                'robot_file_path': robot_file_path_default,
                'init_file_path': init_file_path_default,
                'device_name': device_name_default
            }]
        ),
        Node(
            package='microcontroller_hardware_node',
            executable='microcontroller_hardware_node',
            name='microcontroller_hardware_node',
            parameters=[
                {'port': microcontroller_port},
                {'baudrate': 115200}
            ]
        )
    ])
