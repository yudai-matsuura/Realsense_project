from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('lbr_terrain_analysis'),
        'config',
        'terrain_analysis.rviz'
    )
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
            arguments=['-d', rviz_config_path],
        )
    ])
