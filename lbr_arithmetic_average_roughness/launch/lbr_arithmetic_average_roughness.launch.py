from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('lbr_arithmetic_average_roughness'),
        'config',
        'arithmetic_average_roughness.rviz'
    )
    return LaunchDescription([
        Node(
            package="lbr_arithmetic_average_roughness",
            executable="lbr_arithmetic_average_roughness_node",
            name="lbr_arithmetic_average_roughness",
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
