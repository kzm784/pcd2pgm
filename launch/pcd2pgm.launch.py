from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pcd2pgm_config = os.path.join(
        get_package_share_directory('pcd2pgm'),
        'config',
        'pcd2pgm.yaml'
    )

    return LaunchDescription([
        Node(
            package='pcd2pgm',
            executable='pcd2pgm',
            name='pcd2pgm_node',
            output='screen',
            parameters=[pcd2pgm_config]
        ),
    ])