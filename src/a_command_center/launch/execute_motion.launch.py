
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node



def generate_launch_description():

    robot_name = "alta"
    walk_package = "quintic_walk"

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
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 115200}
            ]
        )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controller_param],
        output="both",
    )
    
    # First launch file - move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('altair_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        )
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
                'altair.launch.py'
            ])
        )
    )

    # Delay the controller launch to give move_group time to start
    delayed_quintic_walk_launch = TimerAction(
        period=5.0,  # Wait 5 seconds before launching controller
        actions=[quintic_walk_launch]
    )
    
    
    # Launch param_manager and rosbridge server in the same terminal
    ws_setup = os.path.expanduser('~/altair_ws_fix/install/setup.bash')
    auxiliary_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c',
             f'source {ws_setup} && '
             'ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash'],
        output='screen'
    )
    
    return LaunchDescription([
        GroupAction([
            PushRosNamespace(robot_name),
            auxiliary_node,
            param_manager_launch,
            microcontroller_hardware_node,
            move_group_launch,
            control_node,
            delayed_quintic_walk_launch,
            # head_controller_launch,
        ])
    ])