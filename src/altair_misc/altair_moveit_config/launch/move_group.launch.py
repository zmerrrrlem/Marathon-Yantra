import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit



def generate_launch_description():

    # Define xacro mappings for the robot description file
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
    }

    robot = "altair"
    robot_name = ""
    robot_namespace_arg = DeclareLaunchArgument(
        "robot",
        default_value=robot_name,
        description="Namespace for the robot",
    )
    namespace = LaunchConfiguration("robot")
    altair_description_path = get_package_share_directory("altair_description")
    xacro_path = f"{altair_description_path}/urdf/{robot}_compose.urdf.xacro"

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            robot, package_name="altair_moveit_config"
        )
        .robot_description_semantic(file_path=f"config/{robot}.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description(file_path=xacro_path, mappings={
            "real_mode": "false",
            "dxl_unavailable": "true",
            "imu_unavailable": "true",
            "mock_mode": "true",
            "sim_mode": "false",
            "tuning_mode": "true",

            "dynamixel_port": "/dev/u2d2",
            "dynamixel_baudrate": "1000000",
            "stm32_port": "/dev/ttyUSB0",
            "stm32_baudrate": "115200",
        })
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        # .moveit_cpp(
        #     file_path="config/moveit_cpp.yaml"
        # )
        .to_moveit_configs()
    )
    moveit_config.move_group_capabilities = {"capabilities": "move_group/MoveGroupKinematicsService"}

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        # namespace=namespace,
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz for visualization
    # Get the path to the RViz configuration file
    # rviz_config_arg = DeclareLaunchArgument(
    #     "rviz_config",
    #     default_value="moveit.rviz",
    #     description="RViz configuration file",
    # )
    # rviz_base = LaunchConfiguration("rviz_config")
    # rviz_config = PathJoinSubstitution(
    #     [FindPackageShare("moveit2_tutorials"), "launch", rviz_base]
    # )

    # Launch RViz
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         moveit_config.planning_pipelines,
    #         moveit_config.joint_limits,
    #     ],
    # )

    # Static TF
    static_tf = Node(
        # namespace=namespace,
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "base", "--child-frame-id", "body_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        # namespace=namespace,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using mock hardware for trajectory execution
    ros2_controllers_path = os.path.join(
        get_package_share_directory("altair_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        namespace=namespace,
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        namespace=namespace,
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    quintic_walk_spawner = Node(
        namespace=namespace,
        package="controller_manager",
        executable="spawner",
        arguments=["quintic_walk", "-c", "/controller_manager"],
    )
    
    zmp_walk_spawner = Node(
        namespace=namespace,
        package="controller_manager",
        executable="spawner",
        arguments=["zmp_walk", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            robot_namespace_arg,
            # static_tf,
            robot_state_publisher,
            run_move_group_node,
            # ros2_control_node,
            # joint_state_broadcaster_spawner,
            # quintic_walk_spawner,
            # RegisterEventHandler(
            #     OnProcessExit(
            #         target_action=joint_state_broadcaster_spawner,
            #         on_exit=[quintic_walk_spawner]
            #     )
            # ),
        ]
    )