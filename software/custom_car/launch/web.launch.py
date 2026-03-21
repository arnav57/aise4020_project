from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. The Hardware Interface (The Muscles)
        Node(
            package='custom_car',
            executable='drive_node',
            name='drive_node',
            output='screen'
        ),
        
        # 2. The Vision Pipeline (The Eyes)
        Node(
            package='custom_car',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),
        
        # 3. The Web Interface (The Cockpit)
        Node(
            package='custom_car',
            executable='web_node',
            name='web_node',
            output='screen'
        ),

        # 4. PID Node (steering control)

        Node(
            package='custom_car',
            executable='pid_node',
            name='pid_node',
            output='screen'
        ),
    ])