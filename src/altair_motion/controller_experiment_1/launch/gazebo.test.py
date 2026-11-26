import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from xacro import process_file

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
            '-file', robot_xacro_path,  # Use direct file path instead of topic
            '-name', 'altair',
            '-z', '0.38',
            '-allow_renaming', 'true',
            '-xacro-args', 'sim_mode:=true'  # Pass xacro arguments directly
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ======================== CONTROLLERS ========================
    controller_params = PathJoinSubstitution(
        [FindPackageShare('quintic_walk'), 'config', 'controller.yaml']
    )

    quintic_walk_params = PathJoinSubstitution(
        [FindPackageShare('quintic_walk'), 'config', 'walking.yaml']
    )

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

    controller_manager = Node(
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
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    controller_experiment_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["controller_experiment_1", "--param-file", controller_params],
    )

    quintic_walk_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['quintic_walk', '--param-file', quintic_walk_params],
        output='screen'
    )

    return LaunchDescription([
        *env_vars,
        
        # Core simulation
        gazebo_launch,
        clock_bridge,
        
        # Robot
        spawn_robot,
        
        # Control stack
        robot_state_publisher,
        # controller_manager,
        joint_state_broadcaster,
        controller_experiment_1,
        # quintic_walk_controller,
    ])