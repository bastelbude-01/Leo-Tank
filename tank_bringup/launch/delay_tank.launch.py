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
            "turret_controller.launch.py"                                
        )
    )
    

    return LaunchDescription([
        controller,
        TimerAction(period=1.0, actions=[hardware_interface_chassis]),
        TimerAction(period=2.5, actions=[hardware_interface_turret])#,
        #TimerAction(period=3.0, actions=[hardware_interface_pipe])
        


    ])
