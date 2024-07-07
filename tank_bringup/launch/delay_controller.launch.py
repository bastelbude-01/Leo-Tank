from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():


    controller_interface = Node(
        package="tank_controller",
        executable="turret_controller.py"
    )


    return LaunchDescription([
           controller_interface


    ])
