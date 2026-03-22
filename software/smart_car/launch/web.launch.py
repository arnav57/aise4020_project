"""
full.launch.py — launches the complete AISE 4020 car stack.

Node startup order doesn't strictly matter in ROS2 (all use topics),
but grouping them by tier makes the intent clear.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # --- Sensing tier ---
        Node(
            package='smart_car',
            executable='vision_node',
            name='vision_node',
            output='screen',
        ),
        Node(
            package='smart_car',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
        ),

        # --- Processing tier ---
        Node(
            package='smart_car',
            executable='pid_node',
            name='pid_node',
            output='screen',
            parameters=[{
                'kp':    0.005,
                'ki':    0.000,
                'kd':    0.050,
                'k_cam': 0.0075,
                'speed': 0.15,
            }],
        ),

        # --- Arbitration ---
        Node(
            package='smart_car',
            executable='arbiter_node',
            name='arbiter_node',
            output='screen',
        ),

        # --- Output tier ---
        Node(
            package='smart_car',
            executable='drive_node',
            name='drive_node',
            output='screen',
        ),

        Node(
            package='smart_car',
            executable='arduino_node',
            name='arduino_node',
            output='screen',
        ),

        # --- UI ---
        Node(
            package='smart_car',
            executable='web_node',
            name='web_node',
            output='screen',
        ),
    ])
