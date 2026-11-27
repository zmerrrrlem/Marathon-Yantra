from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package's share directory
    package_share_directory = get_package_share_directory('quintic_walk')

    # Construct the path to the parameter file
    param_file_path = os.path.join(package_share_directory, 'config', 'walking.yaml')
    print(param_file_path)

    # Ensure the file exists (optional check)
    if not os.path.exists(param_file_path):
        raise FileNotFoundError(f"Parameter file not found: {param_file_path}")

    return LaunchDescription([
        Node(
            package='quintic_walk',             # Replace with your package name
            executable='quintic_walk',             # Replace with your executable name
            output='screen',                  # Output type (e.g., 'screen' or 'log')
            parameters=[param_file_path],     # Path to parameter file
        )
    ])
