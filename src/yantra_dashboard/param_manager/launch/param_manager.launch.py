from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'walk_package',
            default_value='quintic_walk',
            description='The walking package to use (e.g., quintic_walk, cubic_walk, b_splines_walk)'
        ),
        Node(
            package='param_manager',
            executable='param_manager',
            name='param_manager',
            output='screen',
            parameters=[{
                'walk_package': LaunchConfiguration('walk_package'),
                'yaml_file': 'config/walking.yaml'
            }]
        ),
    ])