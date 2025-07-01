import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
        package_name = 'diffbot_control' 

        # Robot state publisher
        robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','robot_state_publisher.launch.py'
                )]),
                launch_arguments={'use_sim_time': 'false'}.items())
        
        rviz = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'sim_config.rviz')],
                output='screen'
        )

        # ros2 controler manager and controllers
        controller_params_file = os.path.join(
                get_package_share_directory(package_name), 
                'config',
                'ros2_controllers.yaml')
        
        controller_manager = Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[controller_params_file]
        )

        diff_controller_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_controller']
        )

        joint_broadcaster_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_broadcaster'],
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
                robot_state_publisher,
                controller_manager,
                diff_controller_spawner,
                joint_broadcaster_spawner,
                rviz,
                teleop_keyboard
        ])