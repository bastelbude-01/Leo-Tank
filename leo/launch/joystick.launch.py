import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_params1 = os.path.join(get_package_share_directory("leo"), "config", "joystick.yaml")
    joy_params2 = os.path.join(get_package_share_directory("leo"), "config", "joystickTurret.yaml")
    pkg_path = os.path.join(get_package_share_directory('leo'))


    joystick = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params1]
        )
    
    teleop_node1 = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node1',
            parameters=[joy_params1],
            remappings=[("/cmd_vel", "/leo_driver/cmd_vel_unstamped")]
        )

    teleop_node2 = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node2',
            parameters=[joy_params2],
            remappings=[("/cmd_vel", "/leo_turret")]
        )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_path,"rviz","display_leo.rviz" )],
    )


    return LaunchDescription([

        joystick,
        teleop_node1,
        teleop_node2,
        rviz2
        
    ])