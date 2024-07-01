from launch import LaunchDescription
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("tank_description"),
                    "urdf",
                    "leopard.urdf.xacro",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(get_package_share_directory("tank_controller"),
                         "config",
                         "tank_controllers.yaml",
                         ),
        ],
    )

    return LaunchDescription([
        controller_manager,
    ])
