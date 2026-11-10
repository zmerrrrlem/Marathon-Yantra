import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # find op3 packages
    altair_description_path = os.path.join(get_package_share_directory('altair_description'))
    xacro_file = os.path.join(altair_description_path, 'urdf', 'altair.urdf.xacro')

    altair_gazebo_path = os.path.join(get_package_share_directory('altair_gazebo_ros2'))

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(altair_gazebo_path, 'worlds'), ':' +
            str(Path(altair_description_path).parent.resolve())
            ]
        )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='empty',
                          description='Gz sim World'),
           ]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r']
                    )
                ]
             )

    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0', 
                   '-y', '0.0',
                   '-z', '0.35',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'altair',
                   '-allow_renaming', 'true'],
    )

    altair_gz_bridge_pkg_path = FindPackageShare('altair_gz_bridge')
    altair_joint_list_path = PathJoinSubstitution([altair_gz_bridge_pkg_path, 'config', 'altair_joint_list.yaml'])
    
    altair_gz_bridge = Node(
        package='altair_gz_bridge',
        executable='altair_gz_bridge_node',
        output='screen',
        parameters=[{'joint_list_path': altair_joint_list_path}]
    )

    bridge_params = os.path.join(altair_gazebo_path,'config','bridge_parameters.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    controller_params = os.path.join(altair_gazebo_path, 'config', 'altair_gazebo_config.yaml')
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    imu_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager', '-p', 'sensor_name:=imu_sensor'],
        output='screen'
    )
    
    forward_position_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["altair_position_command", '--controller-manager', '/controller_manager'],
        )

    return LaunchDescription([
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        ros_gz_bridge,
        altair_gz_bridge,
        joint_state_broadcaster,
        imu_broadcaster,
        forward_position_controller,
    ])
