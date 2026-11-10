from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    start_heading = LaunchConfiguration('start_heading', default='true')

    # Fall detector: publishes /is_falling from /subcon/imu and /subcon/gyr_data
    fall_detector = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='altair_walk',
                executable='fall_detector',
                name='fall_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'accel_threshold_mps2': 15.0,
                    'gyro_threshold_rads': 6.0,
                    'confirm_count': 3,
                    'recover_count': 10,
                    'publish_rate_hz': 50.0,
                }],
            )
        ]
    )

    # Walking node (delayed to let Gazebo start from altair_simulation.launch.py)
    altair_walk_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='altair_walk',
                executable='altair_walk_node',
                name='altair_walk_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    # IMU relay: /robotis_op3/imu -> /subcon/{imu,euler_data,gyr_data} + gain_control
    imu_relay = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='altair_walk',
                executable='imu_relay',
                name='sim_imu_relay',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'pitch_p': 0.4,
                    'pitch_d': 0.05,
                    'pitch_i': 0.3,
                    'pitch_sign': 1.0,
                    'gyro_sign': 1.0,
                    'yaw_p': 0.1,
                    'yaw_d': 0.02,
                    'yaw_sign': 1.0,
                }],
            )
        ]
    )

    # Enable altair_walk_module via service call after walking node starts
    enable_walk_module = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='altair_walk',
                executable='enable_walk_module',
                name='enable_walk_module',
                output='screen',
            )
        ]
    )

    # Optional closed-loop heading controller (off by default)
    heading_hold = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='altair_walk',
                executable='heading_hold',
                name='heading_hold',
                output='screen',
                condition=PythonExpression(["'", start_heading, "' == 'true' and '", crawl_mode, "' == 'false'"]),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'kp': 0.1,
                    'max_angular': 0.02,
                    'euler_filter_tau': 0.5,
                    'walk_debounce': 15,
                    'target_yaw': 0.0,
                }],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'start_heading', default_value='true',
            description='Start heading_hold'),
        fall_detector,
        altair_walk_node,
        imu_relay,
        enable_walk_module,
        heading_hold,
    ])
