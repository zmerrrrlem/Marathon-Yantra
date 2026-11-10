import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).lower()
    video_device = LaunchConfiguration('video_device').perform(context)
    with_motion = LaunchConfiguration('with_motion').perform(context).lower() == 'true'

    bringup_dir = get_package_share_directory('altair_bringup')
    deeplearning_dir = get_package_share_directory('altair_vision_deeplearning')

    default_model = os.path.join(deeplearning_dir, 'models', 'FL3_openvino_int8')
    if not os.path.exists(default_model):
        default_model = os.path.expanduser('~/altair_ws_fix/models/FL3_openvino_int8')

    actions = [
        LogInfo(msg=f"=== Launching Altair Competition: [{mode.upper()}] ===")
    ]

    # Select vision launch based on mode (delegated to altair_vision_deeplearning)
    if mode in ['penalty_kick', 'penalty']:
        vision_launch = os.path.join(deeplearning_dir, 'launch', 'penalty_kick_vision.launch.py')
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vision_launch),
                launch_arguments={
                    'video_device': video_device,
                    'model_path': default_model,
                }.items()
            )
        )
    elif mode in ['united_soccer', 'soccer']:
        vision_launch = os.path.join(deeplearning_dir, 'launch', 'united_soccer_vision.launch.py')
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vision_launch),
                launch_arguments={
                    'video_device': video_device,
                    'model_path': default_model,
                }.items()
            )
        )
    else:
        # Generic / Sprint / Other
        actions.append(
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='camera',
                output='screen',
                parameters=[{
                    'video_device': video_device,
                    'image_width': 640,
                    'image_height': 480,
                    'framerate': 30.0,
                    'pixel_format': 'yuyv',
                    'io_method': 'mmap',
                    'camera_name': 'usb_cam',
                    'frame_id': 'camera_link',
                }],
                remappings=[('image_raw', '/camera/image_raw')]
            )
        )

    # Optional motion stack
    if with_motion:
        actions.append(LogInfo(msg="Bringing up Altair motion controllers..."))
        cmd_center_dir = get_package_share_directory('a_command_center')
        motion_launch = os.path.join(cmd_center_dir, 'launch', 'execute_motion.launch.py')
        if os.path.exists(motion_launch):
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(motion_launch)
                )
            )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='penalty_kick',
            description='Competition mode: [penalty_kick | united_soccer | sprint | obstacle_challenge]'
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video2',
            description='Camera device path (default: /dev/video2 for Microsoft LifeCam)'
        ),
        DeclareLaunchArgument(
            'with_motion',
            default_value='false',
            description='Whether to also launch motion controllers (execute_motion)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
