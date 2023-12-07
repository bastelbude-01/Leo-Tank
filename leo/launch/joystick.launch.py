import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory("leo"), "config", "joystick.yaml")

    joystick = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params]
        )
    
    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
            remappings=[("/cmd_vel", "/leo_driver/cmd_vel_unstamped")]
        )


    return LaunchDescription([

        joystick,
        teleop_node
        
    ])