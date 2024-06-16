from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("chassis_firmware"),  
            "launch",                                           
            "hardware_interface.launch.py"                     
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("tank_controller"),  
            "launch",                                             
            "controller.launch.py"                                
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
        hardware_interface,
        controller,
        joystick


    ])