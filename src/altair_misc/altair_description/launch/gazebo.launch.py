import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ======================== DEPRECATION NOTICE ========================
    # The Altair model is deprecated. This launch now loads Yantravira (yantravira_urdf1).
    deprecation_notice = LogInfo(
        msg="[altair_description/gazebo.launch.py] DEPRECATED: Altair model is deprecated. "
            "Loading Yantravira (yantravira_urdf1) instead."
    )

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

    # ======================== GAZEBO SETUP ========================
    gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
      ),
      launch_arguments={
        'gz_args': '-v 4 -r empty.sdf',
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
            '-z', '0.8',
            '-R', '0', '-P', '0', '-Y', '0',
            '-allow_renaming', 'true',
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

    quintic_walk_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['quintic_walk', '--param-file', quintic_walk_params],
        output='screen'
    )

    return LaunchDescription([
        deprecation_notice,
        *env_vars,

        # Core simulation
        gazebo_launch,
        clock_bridge,

        # Robot description
        robot_state_publisher,

        # Robot (Yantravira)
        spawn_robot,

        # Control stack
        imu_broadcaster,
        joint_state_broadcaster,
        quintic_walk_controller,
    ])
