from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tank_description"),  # package
            "launch",                                         # ordner
            "gazebo.launch.py"                                # file
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tank_controller"),  # package
            "launch",                                        # ordner
            "controller.launch.py"                           # file
        ),
        launch_arguments={
            "use_simple_controller": "false",
            "use_python": "False"
        }.items()
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tank_controller"),
            "launch",
            "joystick_teleop.launch.py"
        )
    )

    return LaunchDescription([
        gazebo,
        controller,
        joystick


    ])
