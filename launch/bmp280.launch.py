from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bmp280'),
        'config',
        'bmp280_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='bmp280',
            executable='bmp280_node',
            name='bmp280_node',
            parameters=[os.path.join(
                os.path.dirname(__file__), '../config/bmp280_params.yaml')],
            output='screen'
        )
    ])
