import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        package_name = 'diffbot_control'

        rviz = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'sim_config.rviz')],
                output='screen'
        )

        teleop_keyboard = Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                output='screen',
                prefix = 'xterm -e',
                parameters=[{'stamped': True}],
                remappings=[('/cmd_vel', '/diff_controller/cmd_vel')]
                        
        )
        return LaunchDescription([
                rviz,
                teleop_keyboard
        ])