from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    cmd_mux = Node(
        package='altair_turn',
        executable='cmd_mux',
        name='cmd_mux',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'turn_priority': True,
            'mission_timeout_s': 0.5,
            'turn_timeout_s': 0.5,
            'publish_rate_hz': 20.0,
        }],
    )

    turn_supervisor = Node(
        package='altair_turn',
        executable='turn_supervisor',
        name='turn_supervisor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'linear_speed': 0.0,
            'capture_band_deg': 8.0,
            'settle_tolerance_deg': 2.0,
            'settle_ticks': 10,
            'timeout_s': 60.0,
            'in_place': True,
            'yaw_sign': 1.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation time'
        ),
        cmd_mux,
        turn_supervisor,
    ])
