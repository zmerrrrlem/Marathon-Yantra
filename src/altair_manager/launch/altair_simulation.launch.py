from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from xacro import process_file
import os
import subprocess

def launch_gazebo_control_in_terminal(context, *args, **kwargs):
    """Launch gazebo.control.py in a new terminal, using any available terminal emulator."""
    control_script_path = os.path.join(
        get_package_share_directory('altair_manager'), 'launch', 'gazebo.control.py'
    )
    robot_name = kwargs.get('robot_name', 'altair')

    # Commands to try in order
    terminal_commands = [
        ["gnome-terminal", "--", "bash", "-c", f"python3 {control_script_path} {robot_name}; exec bash"],
        ["xfce4-terminal", "--command", f"bash -c 'python3 {control_script_path} {robot_name}; exec bash'"],
        ["konsole", "-e", f"bash -c 'python3 {control_script_path} {robot_name}; exec bash'"],
        ["xterm", "-hold", "-e", f"python3 {control_script_path} {robot_name}"],
        ["lxterminal", "-e", f"bash -c 'python3 {control_script_path} {robot_name}; exec bash'"]
    ]

    for cmd in terminal_commands:
        try:
            subprocess.Popen(cmd)
            print(f"Launching using: {cmd[0]}")
            return
        except FileNotFoundError:
            continue

    raise RuntimeError(
        "No terminal emulator found (gnome-terminal, xfce4-terminal, konsole, xterm, lxterminal). "
        "Install one or run the script directly:\n"
        f"python3 {control_script_path} {robot_name}"
    )

def generate_launch_description():
    simulation_default = True
    simulation_robot_name_default = 'robotis_op3'
    offset_file_path_default = get_package_share_directory('altair_manager') + '/config/altair_offset.yaml'
    robot_file_path_default = get_package_share_directory('altair_manager') + '/config/altair.robot'
    init_file_path_default = get_package_share_directory('altair_manager') + '/config/dxl_init_altair.yaml'
    device_name_default = '/dev/ttyUSB0'

    altair_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('altair_gazebo_ros2'),
                'launch',
                'sim.launch.py'
            )
        )
    )

    # Delay altair_manager by 5 seconds
    altair_manager_delayed = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='altair_manager',
                executable='altair_manager',
                # name='altair_manager',
                output='screen',
                parameters=[{
                    'simulation': simulation_default,
                    'simulation_robot_name': simulation_robot_name_default,
                    'offset_file_path': offset_file_path_default,
                    'robot_file_path': robot_file_path_default,
                    'init_file_path': init_file_path_default,
                    'device_name': device_name_default
                }]
            )
        ]
    )

    altair_description = get_package_share_directory('altair_description')
    robot_xacro_path = os.path.join(altair_description, 'urdf', 'altair.urdf.xacro')
    robot_description = process_file(robot_xacro_path, mappings={'use_sim': 'true'}).toxml()
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
    )

    return LaunchDescription([
        altair_gazebo_launch,
        altair_manager_delayed,

        robot_state_publisher,

        OpaqueFunction(function=launch_gazebo_control_in_terminal),
    ])
