# launch/ball_detect.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # Get the path to the config file relative to the usb_cam package
    config_path = PathJoinSubstitution([
        FindPackageShare('usb_cam'),
        'config',
        'camera_info.yaml'
    ])

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'camera_name': 'usb_cam',
                'camera_info_url': ['file://', config_path],
                'io_method': 'mmap',
            }],
        ),
        Node(
            package='ball_detect',
            executable='ball_det',
            name='ball_detector',
            output='screen'
        )
        # Node(
        #     package='soccer_ipm',
        #     executable='ipm',
        #     name='soccer_ipm',
        #     output='screen',
        #     parameters=[
        #         {'balls.ball_diameter': 0.153},
        #         {'output_frame': 'base_footprint'},
        #         {'robots.footpoint_out_of_image_threshold': 0.8},
        #         {'robots.object_default_dimensions.x': 0.2},
        #         {'robots.object_default_dimensions.y': 0.2},
        #         {'robots.object_default_dimensions.z': 1.0},
        #         {'use_distortion': False}
        #     ]
        # )
    ])
