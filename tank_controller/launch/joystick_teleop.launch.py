from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory(
            "tank_controller"), "config", "joyconfig.yaml")]

    )

    joy_drive = Node(
        package="joy_teleop",
        executable="joy_teleop",
        #name="joy_drive",
        parameters=[os.path.join(get_package_share_directory(
            "tank_controller"), "config", "joydrive.yaml")]
    )

    joy_turret = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop_",
        parameters=[os.path.join(get_package_share_directory(
            "tank_controller"), "config", "joyturret.yaml")]
    )

    return LaunchDescription([
        joy_node,
        #joy_turret,
        joy_drive
        
    ])
