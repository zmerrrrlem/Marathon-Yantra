import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('altair_vision_deeplearning')
    default_model = os.path.join(pkg_share, 'models', 'FL3_openvino_int8')
    if not os.path.exists(default_model):
        default_model = os.path.expanduser('~/altair_ws_fix/models/FL3_openvino_int8')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video2',
            description='Path to video device (e.g. /dev/video2 for Microsoft LifeCam)'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Camera width'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Camera height'
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='30.0',
            description='Camera FPS'
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model,
            description='Path to FL3 OpenVINO model directory'
        ),
        DeclareLaunchArgument(
            'ball_conf',
            default_value='0.50',
            description='Confidence threshold for ball in penalty kick'
        ),
        DeclareLaunchArgument(
            'goal_conf',
            default_value='0.45',
            description='Confidence threshold for goal in penalty kick'
        ),

        # Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'framerate': LaunchConfiguration('framerate'),
                'pixel_format': 'yuyv',
                'io_method': 'mmap',
                'camera_name': 'usb_cam',
                'frame_id': 'camera_link',
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
            ]
        ),

        # OpenVINO Detector Node for Penalty Kick
        Node(
            package='altair_vision_deeplearning',
            executable='openvino_detector',
            name='openvino_detector',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'camera_topic': '/camera/image_raw',
                'mode': 'penalty_kick',
                'conf_threshold': 0.35,
                'ball_conf': LaunchConfiguration('ball_conf'),
                'goal_conf': LaunchConfiguration('goal_conf'),
                'publish_annotated': True,
            }]
        ),
    ])
