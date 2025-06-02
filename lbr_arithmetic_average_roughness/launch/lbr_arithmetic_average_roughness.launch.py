from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lbr_arithmetic_average_roughness",
            executable="lbr_arithmetic_average_roughness_node",
            name="lbr_arithmetic_average_roughness",
            output="screen",
        )
    ])
