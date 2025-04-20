from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    terrain_analysis = Node(
        package="terrain_analysis",
        executable="terrain_analysis",
        name="terrain_analysis",
        output="screen"
    )

    return LaunchDescription(terrain_analysis)
