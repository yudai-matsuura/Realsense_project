from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lbr_terrain_analysis",
            executable="lbr_terrain_analysis_node",
            name="lbr_terrain_analysis",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        )
    ])
