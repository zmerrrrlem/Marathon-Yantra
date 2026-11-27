import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
            value=[FindPackageShare('yantra_description'), '/urdf']
        ),
    ]

    # ======================== ROBOT DESCRIPTION ========================
    yantra_description = get_package_share_directory('yantra_description')
    world_path = os.path.join(yantra_description, 'worlds', 'empty.sdf')
    robot_xacro_path = os.path.join(yantra_description, 'urdf', 'yantra_compose.urdf.xacro')
    robot_description = process_file(robot_xacro_path, mappings={'sim_mode': 'true'}).toxml()

    # ======================== GAZEBO SETUP ========================
    gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
      ),
      launch_arguments={
        'gz_args': f'-v 4 -r {world_path}',  # Verbose logging with specified world
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
            '-topic', 'robot_description',
            '-name', 'yantra',
            '-z', '0.35',
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

    # NOTE: no separate ros2_control_node here. The gz_ros2_control plugin (gazebo/yantra.gazebo.xacro)
    # already runs /controller_manager inside Gazebo. A second /controller_manager with the same name
    # made spawner service calls time out ("Failed to acquire lock", configure_controller timeouts).

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    imu_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster'],
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
        robot_state_publisher,

        # Control stack, started one after another instead of all at once:
        # joint_state_broadcaster -> imu_sensor_broadcaster + controller_experiment_1 -> quintic_walk
        joint_state_broadcaster,
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[imu_broadcaster, controller_experiment_1])),
        RegisterEventHandler(OnProcessExit(
            target_action=imu_broadcaster,
            on_exit=[quintic_walk_controller])),
    ])