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
                    "tank_speed_controller",
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
        turret_controller#,
        #pipe_controller
    ])
