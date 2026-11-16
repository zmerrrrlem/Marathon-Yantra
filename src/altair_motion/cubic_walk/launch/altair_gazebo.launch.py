import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_gazebo_control_in_terminal(context, *args, **kwargs):
    """Function to launch gazebo.control.py in a new terminal."""
    control_script_path = os.path.join(
        get_package_share_directory('altair_description'), 'launch', 'gazebo.control.py'
    )
    subprocess.Popen([
        "gnome-terminal", "--", "python3", control_script_path
    ])


def generate_launch_description():
    # ======================== DEPRECATION NOTICE ========================
    # The Altair model is deprecated. Gazebo now loads Yantravira (yantravira_urdf1).
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
        robot_description = f.read()

    # ======================== ENVIRONMENT SETUP ========================
    env_vars = [
        # Required for Gazebo-ROS 2 communication
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=[FindPackageShare('ros_gz_sim'), '/lib']
        ),
        # Make yantravira_urdf1 meshes resolvable in Gazebo
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[altair_lab_root, ':', _os.path.join(altair_lab_root, 'src')]
        ),
    ]

    # ======================== LAUNCH ARGUMENTS ========================
    robot_namespace_arg = DeclareLaunchArgument(
        "robot",
        default_value="",
        description='Namespace for the robot'
    )
    namespace = LaunchConfiguration("robot")

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

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=['/gazebo/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen'
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen'
    )

    # ======================== ROBOT SPAWNING ========================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-topic', [namespace, '/robot_description'],
            '-name', 'yantra',
            '-z', '0.8',
            '-R', '0', '-P', '0', '-Y', '0',
            '-allow_renaming', 'true',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ======================== CONTROLLERS ========================
    controller_params = PathJoinSubstitution(
        [FindPackageShare('cubic_walk'), 'config', 'controller.yaml']
    )

    cubic_walk_params = PathJoinSubstitution(
        [FindPackageShare('cubic_walk'), 'config', 'walking.yaml']
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

    joint_state_broadcaster = Node(
        namespace=namespace,
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controller_params, '--controller-manager-timeout', '60'],
        output='screen'
    )

    imu_broadcaster = Node(
        namespace=namespace,
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--param-file', controller_params, '--controller-manager-timeout', '60'],
        output='screen'
    )

    cubic_walk_controller = Node(
        namespace=namespace,
        package='controller_manager',
        executable='spawner',
        arguments=['cubic_walk', '--param-file', controller_params, '--param-file', cubic_walk_params, '--controller-manager-timeout', '60'],
        output='screen'
    )

    return LaunchDescription([
        robot_namespace_arg,
        *env_vars,

        # Core simulation
        gazebo_launch,
        clock_bridge,
        camera_bridge,
        lidar_bridge,

        # Robot description
        robot_state_publisher,

        # Robot (Yantravira)
        spawn_robot,

        # Control stack (hosted by Gazebo gz_ros2_control plugin), delayed for Gazebo readiness
        TimerAction(period=10.0, actions=[imu_broadcaster, joint_state_broadcaster, cubic_walk_controller]),

        # Launch gazebo.control.py in a new terminal
        OpaqueFunction(function=launch_gazebo_control_in_terminal),
    ])
