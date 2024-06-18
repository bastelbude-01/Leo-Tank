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

    rviz = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tank_description"),
            "launch",
            "display_rviz.launch.py"
        )
    )   

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tank_description"),  # package
            "launch",                                         # ordner
            "gazebo.launch.py"                                # file
        )
    )


    return LaunchDescription([
        joystick,
        rviz,
        gazebo
    ])
