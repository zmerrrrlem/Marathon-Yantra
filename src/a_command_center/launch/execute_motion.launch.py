
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    robot_name = ""            # keep the variable for configure.py's regex, but run in global namespace
    walk_package = "quintic_walk"

    # STM32 port and baudrate launch arguments
    stm32_port_arg = DeclareLaunchArgument('stm32_port', default_value='/dev/stm32')
    stm32_baudrate_arg = DeclareLaunchArgument('stm32_baudrate', default_value='115200')

    ros2_controller_param = PathJoinSubstitution(
        [
            FindPackageShare('command_center'),
            'config',
            'controller.yaml',
        ]
    )

    microcontroller_hardware_node = Node(
            package='microcontroller_hardware_node',
            executable='microcontroller_hardware_node',
            name='microcontroller_hardware_node',
            parameters=[{
                'port': LaunchConfiguration('stm32_port'),
                'baudrate': ParameterValue(LaunchConfiguration('stm32_baudrate'), value_type=int),
            }],
        )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controller_param],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )
    
    # First launch file - move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('yantra_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'publish_robot_state': 'true',
        }.items()
    )
    
    # Second launch file - quintic_walk controller (delayed)
    quintic_walk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('quintic_walk'),
                'launch',
                'controller.launch.py'
            ])
        )
    )
    param_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('param_manager'),
                'launch',
                'param_manager.launch.py'
            ])
        ),
        launch_arguments={
            'walk_package': walk_package
        }.items()
    ) 

    head_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('head_controller'),
                'launch',
                'yantra.launch.py'
            ])
        )
    )

    # Delay the controller launch to give move_group time to start
    delayed_quintic_walk_launch = TimerAction(
        period=10.0,  # Wait 10 seconds before launching controller
        actions=[quintic_walk_launch]
    )
    
    
    has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))

    if has_display:
        auxiliary_node = ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c',
                 'source install/setup.bash && '
                 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash'],
            output='screen'
        )
    
    return LaunchDescription([
        GroupAction([
            # PushRosNamespace(robot_name),   # removed: run in the global namespace
            stm32_port_arg,
            stm32_baudrate_arg,
            auxiliary_node,
            param_manager_launch,
            microcontroller_hardware_node,
            move_group_launch,
            control_node,
            delayed_quintic_walk_launch,
            # head_controller_launch,
        ])
    ])