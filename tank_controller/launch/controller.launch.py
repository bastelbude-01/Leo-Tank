from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="True"
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.035"
    )

    wheel_seperation_arg = DeclareLaunchArgument(
        "wheel_seperation",
        default_value="0.163"
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="False"
    )

    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_seperation = LaunchConfiguration("wheel_seperation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "chassis_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)

    )

    turret_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "turret_position_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)

    )

    turm_control = Node(
                package="tank_controller",
                executable="turret_controller.py"
            ),

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[

            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "tank_speed_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            ),

            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "turret_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            ),

            Node(
                package="tank_controller",
                executable="tank_speed_controller.py",
                parameters=[{"wheel_radius": wheel_radius},
                            {"wheel_seperation": wheel_seperation}],
                condition=IfCondition(use_python)
            ),

            Node(
                package="tank_controller",
                executable="turret_speed_controller.py",
                condition=IfCondition(use_python)
            ),

            Node(
                package="tank_controller",
                executable="simple_controller",
                parameters=[{"wheel_radius": wheel_radius},
                            {"wheel_seperation": wheel_seperation}],
                condition=UnlessCondition(use_python)
            )

        ]
    )

    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_seperation_arg,
        use_simple_controller_arg,
        wheel_controller_spawner,
        joint_state_broadcaster_spawner,
        simple_controller,
        #turm_control,
        turret_controller_spawner

    ])
