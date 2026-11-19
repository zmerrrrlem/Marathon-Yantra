import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    import os as _os

    # Resolve workspace root (where configure.py lives)
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

    # DEPRECATED: Altair model is deprecated. Gazebo now loads Yantravira (yantravira_urdf1).
    yantra_description_dir = _os.path.join(altair_lab_root, 'src', 'yantravira_urdf1')
    yantra_urdf_path = _os.path.join(yantra_description_dir, 'urdf', 'yantravira_urdf1.urdf')
    with open(yantra_urdf_path, 'r') as f:
        robot_description = f.read()

    env_vars = [
    AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
        altair_lab_root + ':' + _os.path.join(altair_lab_root, 'src')),
    ]

    robot_namespace_arg = DeclareLaunchArgument("robot", default_value="", description='Namespace for the robot')
    world_arg = DeclareLaunchArgument("world", default_value="empty.sdf", description="Gazebo world SDF file to load")
    namespace = LaunchConfiguration("robot")
    world = LaunchConfiguration("world")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-v 4 -r ', world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    camera_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
        arguments=['/gazebo/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'])
    lidar_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='lidar_bridge',
        arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'])

    spawn_robot = Node(package='ros_gz_sim', executable='create', name='spawn_robot',
        arguments=['-topic', [namespace, '/robot_description'], '-name', 'yantra', '-z', '1.5', '-R', '0', '-P', '0', '-Y', '0', '-allow_renaming', 'true'],
        parameters=[{'use_sim_time': True}], output='screen')

    controller_params = PathJoinSubstitution([FindPackageShare('cubic_walk'), 'config', 'controller.yaml'])
    cubic_walk_params = PathJoinSubstitution([FindPackageShare('cubic_walk'), 'config', 'walking.yaml'])

    robot_state_publisher = Node(namespace=namespace, package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="both",
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    joint_state_broadcaster = Node(namespace=namespace, package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controller_params, '--controller-manager-timeout', '60'], output='screen')
    cubic_walk_controller = Node(namespace=namespace, package='controller_manager', executable='spawner',
        arguments=['cubic_walk', '--param-file', controller_params, '--param-file', cubic_walk_params, '--controller-manager-timeout', '60'], output='screen')

    # Compatibility utilities from altair_walk (DEPRECATED - to be replaced by cubic_walk-native equivalents)
    # TODO: Replace these with cubic_walk-native equivalents
    # heading_hold removed - was publishing noise-level angular.z causing spurious walking
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

    # Odometry fuser - fuses walk engine odometry with IMU data
    odometry_fuser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('altair_odometry_fuser'),
                'launch',
                'odometry_fuser.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    return LaunchDescription([
        robot_namespace_arg,
        world_arg,
        *env_vars,
        gazebo_launch,
        clock_bridge,
        camera_bridge,
        lidar_bridge,
        robot_state_publisher,
        spawn_robot,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[cubic_walk_controller])),
        TimerAction(period=12.0, actions=[odometry_fuser_launch]),
        fall_detector,
        joint_command_bridge,
        distance_tracker,
        robot_mode_controller,
    ])