import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    color = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{
                'image_size': [640,480],
                'camera_frame_id': 'camera_link_optical'
                }]
        )

    BW= Node(
            package='pi_rover',
            executable='image_publisher',
        )

    return LaunchDescription([

    BW        
        
    ])