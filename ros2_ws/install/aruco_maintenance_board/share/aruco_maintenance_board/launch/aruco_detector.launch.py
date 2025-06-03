#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image_raw',
            description='Camera topic name'
        ),
        
        Node(
            package='aruco_maintenance_board',
            executable='aruco_detector',
            name='aruco_detector_node',
            output='screen',
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic')
            }],
            remappings=[
                ('/camera/image_raw', LaunchConfiguration('camera_topic'))
            ]
        )
    ])