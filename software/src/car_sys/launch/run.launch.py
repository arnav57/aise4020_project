# run.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('car_sys')
    config = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='car_sys',
            executable='car_lcd',
            name='car_lcd',
            parameters=[config],
            output='screen'
        ),
        # adding a new sensor = adding a Node() block here
    ])