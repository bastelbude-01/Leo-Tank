import os
import launch
import launch.actions
import launch.substitutions
import launch.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_params1 = os.path.join(get_package_share_directory("leo"), "config", "joystick.yaml")
    
    joystick = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params1]
        )
    
    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node1',
            parameters=[joy_params1],
            #remappings=[("/cmd_vel", "/mk3_twist")]
        )




    return LaunchDescription([

        #launch.actions.ExecuteProcess(
        #    cmd=['ros2', 'run', 'micro_ros_agent','micro_ros_agent','udp4','--port', '8868', '--dev', '192.168.178.88'],
        #    output='screen',
        #),

        joystick,
        teleop_node
        
    ])