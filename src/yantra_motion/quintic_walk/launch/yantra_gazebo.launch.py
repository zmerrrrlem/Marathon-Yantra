import os
import shutil
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
    """Launch gazebo.control.py in a new terminal, only when a display exists."""
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("[yantra_gazebo] No display detected; skipping gazebo.control.py terminal.")
        return
    if shutil.which("gnome-terminal") is None:
        print("[yantra_gazebo] gnome-terminal not found; skipping gazebo.control.py terminal.")
        return
    control_script_path = os.path.join(
        get_package_share_directory('yantra_description'), 'launch', 'gazebo.control.py'
    )
    subprocess.Popen([
        "gnome-terminal", "--", "python3", control_script_path
    ])

def generate_launch_description():
    # ======================== ENVIRONMENT SETUP ========================
    # Gazebo resolves package:// mesh URIs as model://<package>/..., so the
    # parent of the installed package share directory must be on the path.
    yantra_description = get_package_share_directory('yantra_description')
    gazebo_resource_path = os.pathsep.join(filter(None, [
        os.path.dirname(yantra_description),
        os.path.join(yantra_description, 'urdf'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    env_vars = [
        # Required for Gazebo-ROS 2 communication
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=[FindPackageShare('ros_gz_sim'), '/lib']
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gazebo_resource_path
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
    robot_xacro_path = os.path.join(yantra_description, 'urdf', 'yantra_compose.urdf.xacro')
    robot_description = process_file(robot_xacro_path, mappings={'sim_mode': 'true'}).toxml()

    # ======================== GAZEBO SETUP ========================
    # Headless machines cannot start the Gazebo GUI; run the server only.
    display_available = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    gz_args = "-v 4 -r empty.sdf" if display_available else "-v 4 -s -r empty.sdf"

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_args,  # Verbose logging with empty world
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
            '-name', 'yantra',
            '-z', '0.35',
            '-allow_renaming', 'true',
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

    # Gazebo hosts the controller manager via the gz_ros2_control URDF plugin.
    # Give it time to come up before asking spawners to attach to it.
    delayed_control_stack = TimerAction(
        period=10.0,
        actions=[imu_broadcaster, joint_state_broadcaster, quintic_walk_controller],
    )

    return LaunchDescription([
        robot_namespace_arg,
        *env_vars,

        # Core simulation
        gazebo_launch,
        clock_bridge,

        # Robot
        robot_state_publisher,
        spawn_robot,

        # Control stack (delayed until Gazebo's controller manager is ready)
        delayed_control_stack,

        # Launch gazebo.control.py in a new terminal when a display is available
        OpaqueFunction(function=launch_gazebo_control_in_terminal),
    ])