from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tank_controller"),
            "launch",
            "joystick_teleop.launch.py"
        )
    )

    return LaunchDescription([
        joystick
    ])
