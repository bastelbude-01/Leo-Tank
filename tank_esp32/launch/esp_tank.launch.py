import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    return LaunchDescription(
        [
            
            ExecuteProcess(
                cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial',
                      '--dev', '/dev/ttyUSB0'],
                output='screen'
            ),
            
            Node(
                package="tank_esp32",
                executable="tank_joy.py",
            ),
            Node(
                package="joy",
                executable="joy_node",
                output="screen",
            ),            
        ]
    )
