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

def launch_gazebo_control_in_terminal(context, *args, **kwargs):
    """Function to launch gazebo.control.py in a new terminal."""
    control_script_path = os.path.join(
        get_package_share_directory('altair_description'), 'launch', 'gazebo.control.py'
    )
    subprocess.Popen([
        "gnome-terminal", "--", "python3", control_script_path
    ])

def generate_launch_description():
    # ======================== ENVIRONMENT SETUP ========================
    env_vars = [
        # Required for Gazebo-ROS 2 communication
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=[FindPackageShare('ros_gz_sim'), '/lib']
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[FindPackageShare('altair_description'), '/urdf']
        ),
    ]

    # ======================== ROBOT DESCRIPTION ========================
    # Declare launch argument for robot namespace
    robot_namespace_arg = DeclareLaunchArgument(
        "robot",
        default_value="",
        description='Namespace for the robot'
    )

    # Get the value of the robot namespace
    namespace = LaunchConfiguration("robot")
    altair_description = get_package_share_directory('altair_description')
    robot_xacro_path = os.path.join(altair_description, 'urdf', 'altair_compose.urdf.xacro')
    robot_description = process_file(robot_xacro_path, mappings={'sim_mode': 'true'}).toxml()

    # ======================== GAZEBO SETUP ========================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-v 4 -r empty.sdf',  # Verbose logging with empty world
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ======================== TIME SYNCHRONIZATION ========================
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # ======================== ROBOT SPAWNING ========================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-topic', [namespace, '/robot_description'],
            '-name', 'altair',
            '-z', '0.35',
            '-allow_renaming', 'true',
            '--ros-args --remap', 'sim_mode:=true'  # Pass xacro arguments directly
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ======================== CONTROLLERS ========================
    controller_params = PathJoinSubstitution(
        [FindPackageShare('quintic_walk'), 'config', 'gazebo_sim', 'controller.yaml']
    )

    quintic_walk_params = PathJoinSubstitution(
        [FindPackageShare('quintic_walk'), 'config', 'gazebo_sim', 'walking.yaml']
    )

    robot_state_publisher = Node(
        namespace=namespace,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
    )

    controller_manager = Node(
        namespace=namespace,
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[
            {'use_sim_time': True},
            controller_params
        ],
        output='screen'
    )

    joint_state_broadcaster = Node(
        namespace=namespace,
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    imu_broadcaster = Node(
        namespace=namespace,
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster'],
        output='screen'
    )

    quintic_walk_controller = Node(
        namespace=namespace,
        package='controller_manager',
        executable='spawner',
        arguments=['quintic_walk', '--param-file', quintic_walk_params],
        output='screen'
    )

    return LaunchDescription([
        robot_namespace_arg,
        *env_vars,
        
        # Core simulation
        gazebo_launch,
        clock_bridge,
        
        # Robot
        spawn_robot,
        
        # Control stack
        imu_broadcaster,
        joint_state_broadcaster,
        quintic_walk_controller,

        # Launch gazebo.control.py in a new terminal
        OpaqueFunction(function=launch_gazebo_control_in_terminal),
    ])