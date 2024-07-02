from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    

    hardware_interface_chassis = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("chassis_firmware"),  
            "launch",                                           
            "hardware_interface.launch.py"                     
        )
    )

    hardware_interface_turret = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("turret_firmware"),  
            "launch",                                           
            "hardware_interface.launch.py"                     
        )
    )

    hardware_interface_pipe = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("pipe_firmware"),  
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


    return LaunchDescription([
        controller,
        hardware_interface_chassis,
        hardware_interface_turret#,
        #hardware_interface_pipe
        


    ])
