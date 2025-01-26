from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.1",
    )
    wheel_separation_length_arg = DeclareLaunchArgument(
        "wheel_separation_length",
        default_value="0.35",
    )
    wheel_separation_width_arg = DeclareLaunchArgument(
        "wheel_separation_width",
        default_value="0.34",
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation_length = LaunchConfiguration("wheel_separation_length")
    wheel_separation_width = LaunchConfiguration("wheel_separation_width")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    mecanum_controller = GroupAction(
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["mecanum_controller", 
                        "--controller-manager", 
                        "/controller_manager"
                ]
            ),
            Node(
                package="zeta_controller",
                executable="mecanum_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius,
                    "wheel_separation_length": wheel_separation_length,
                    "wheel_separation_width": wheel_separation_width,
                    "use_sim_time": use_sim_time}],
            ),
        ]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            wheel_radius_arg,
            wheel_separation_length_arg,
            wheel_separation_width_arg,
            joint_state_broadcaster_spawner,
            mecanum_controller,
        ]
    )