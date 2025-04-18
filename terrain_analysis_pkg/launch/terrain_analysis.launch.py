from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    terrain_roughness = Node(
        package = 'terrain_analysis',
        executable = 'terrain_roughness',
        name = 'terrain_roughness',
        output = 'screen'
    )

    return LaunchDescription([
        terrain_roughness
    ])