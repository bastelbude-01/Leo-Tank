from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    speed_controller =  Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "chassis_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            )
    
    turret_controller =  Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "turret_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            )
    
    turm_control = Node(
                package="tank_controller",
                executable="turret_controller.py"
            )
    
    pipe_controller =  Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "pipe_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            )


    return LaunchDescription([
        joint_state_broadcaster_spawner,
        speed_controller,
        #turm_control,
        turret_controller#,
        #pipe_controller
    ])
