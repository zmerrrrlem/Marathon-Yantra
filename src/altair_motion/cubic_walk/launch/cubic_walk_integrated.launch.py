import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from xacro import process_file
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def launch_gazebo_control_in_terminal(context, *args, **kwargs):
    control_script_path = os.path.join(
        get_package_share_directory('altair_description'), 'launch', 'gazebo.control.py'
    )
    subprocess.Popen([
        "gnome-terminal", "--", "python3", control_script_path
    ])

def generate_launch_description():
    env_vars = [
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=[FindPackageShare('ros_gz_sim'), '/lib']),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[FindPackageShare('altair_description'), '/urdf']),
    ]

    robot_namespace_arg = DeclareLaunchArgument("robot", default_value="", description='Namespace for the robot')
    namespace = LaunchConfiguration("robot")
    altair_description = get_package_share_directory('altair_description')
    robot_xacro_path = os.path.join(altair_description, 'urdf', 'altair_compose.urdf.xacro')
    robot_description = process_file(robot_xacro_path, mappings={'sim_mode': 'true'}).toxml()

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-v 4 -r empty.sdf', 'on_exit_shutdown': 'true'}.items()
    )

    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')
    camera_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
        arguments=['/gazebo/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'], output='screen')
    lidar_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='lidar_bridge',
        arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'], output='screen')

    spawn_robot = Node(package='ros_gz_sim', executable='create', name='spawn_robot',
        arguments=['-topic', [namespace, '/robot_description'], '-name', 'altair', '-z', '0.35', '-allow_renaming', 'true'],
        parameters=[{'use_sim_time': True}], output='screen')

    controller_params = PathJoinSubstitution([FindPackageShare('cubic_walk'), 'config', 'controller.yaml'])
    cubic_walk_params = PathJoinSubstitution([FindPackageShare('cubic_walk'), 'config', 'walking.yaml'])

    robot_state_publisher = Node(namespace=namespace, package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="both",
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    joint_state_broadcaster = Node(namespace=namespace, package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controller_params, '--controller-manager-timeout', '60'], output='screen')
    imu_broadcaster = Node(namespace=namespace, package='controller_manager', executable='spawner',
        arguments=['imu_sensor_broadcaster', '--param-file', controller_params, '--controller-manager-timeout', '60'], output='screen')
    cubic_walk_controller = Node(namespace=namespace, package='controller_manager', executable='spawner',
        arguments=['cubic_walk', '--param-file', controller_params, '--param-file', cubic_walk_params, '--controller-manager-timeout', '60'], output='screen')

    # Compatibility utilities from altair_walk (ported for cubic_walk)
    heading_hold = TimerAction(period=5.0, actions=[Node(
        package='altair_walk', executable='heading_hold', name='heading_hold', output='screen',
        parameters=[{'use_sim_time': True}],
    )])

    fall_detector = TimerAction(period=5.0, actions=[Node(
        package='altair_walk', executable='fall_detector', name='fall_detector', output='screen',
        parameters=[{'use_sim_time': True}],
    )])

    joint_command_bridge = TimerAction(period=5.0, actions=[Node(
        package='altair_walk', executable='joint_command_bridge', name='joint_command_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
    )])

    distance_tracker = TimerAction(period=5.0, actions=[Node(
        package='altair_walk', executable='distance_tracker', name='distance_tracker', output='screen',
        parameters=[{
            'target_distance_m': 400.0,
            'reset_threshold_m': 5.0,
            'publish_rate_hz': 10.0,
            'use_sim_time': True,
        }],
    )])

    robot_mode_controller = TimerAction(period=5.0, actions=[Node(
        package='altair_walk', executable='robot_mode_controller', name='robot_mode_controller', output='screen',
        parameters=[{'use_sim_time': True, 'default_mode': 0}],
    )])

    return LaunchDescription([
        robot_namespace_arg,
        *env_vars,
        gazebo_launch,
        clock_bridge,
        camera_bridge,
        lidar_bridge,
        robot_state_publisher,
        spawn_robot,
        TimerAction(period=10.0, actions=[imu_broadcaster, joint_state_broadcaster, cubic_walk_controller]),
        heading_hold,
        fall_detector,
        joint_command_bridge,
        distance_tracker,
        robot_mode_controller,
        OpaqueFunction(function=launch_gazebo_control_in_terminal),
    ])