from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


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
        TimerAction(period=5.0, actions=[hardware_interface_chassis]),
        TimerAction(period=5.0, actions=[hardware_interface_turret])#,
        #TimerAction(period=5.0, actions=[hardware_interface_pipe])
        


    ])
