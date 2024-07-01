from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    

    

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

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller])

    hardware_interface_chassis = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("chassis_firmware"),  
            "launch",                                           
            "hardware_interface.launch.py"                     
        )
    )

    delayed_chassis_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller,
            on_start=[hardware_interface_chassis],
        )
    )

    hardware_interface_turret = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("turret_firmware"),  
            "launch",                                           
            "hardware_interface.launch.py"                     
        )
    )

    delayed_turret_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller,
            on_start=[hardware_interface_turret],
        )
    )

    hardware_interface_pipe = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("pipe_firmware"),  
            "launch",                                           
            "hardware_interface.launch.py"                     
        )
    )

    delayed_pipe_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller,
            on_start=[hardware_interface_pipe],
        )
    )


    return LaunchDescription([
        controller,
        delayed_controller_manager,
        delayed_chassis_spawner#,
        #delayed_turret_spawner


    ])
