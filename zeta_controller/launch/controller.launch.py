from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zeta_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
    )


    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
        ]
    )