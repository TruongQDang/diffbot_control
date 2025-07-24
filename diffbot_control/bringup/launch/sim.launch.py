import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
        # Package name
        package_name = 'diffbot_control'

        # Process URDF
        xacro_file = os.path.join(
                get_package_share_directory(package_name),
                'urdf', 
                'robot.urdf.xacro'
        )
        robot_description = Command(['xacro ', xacro_file, ' sim_mode:=true'])
        
        # Robot state publisher
        robot_config = {'use_sim_time': True, 'robot_description': robot_description}
        robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name="robot_state_publisher",
                output='screen',
                parameters=[robot_config],
        ) 

        # Gazebo Sim
        world = os.path.join(
                get_package_share_directory(package_name),
                'world',
                'slam_world.world')    
        gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': ['-r ', world]}.items())

        # Spawn robot
        spawn = Node(
                package='ros_gz_sim', 
                executable='create',
                parameters=[{
                        'name': 'my_robot_sim',
                        'topic': 'robot_description',
                        'z': 0.5,}],
                        output='screen')
        
        # ROS-Gz bridge
        bridge_params = os.path.join(
                get_package_share_directory(package_name),
                'config',
                'gz_bridge.yaml')
        bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                        '--ros-args',
                        '-p',
                        f'config_file:={bridge_params}'],
                output='screen')

        # ROS2 control
        diff_drive_controller_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller']
        )

        joint_state_broadcaster_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
        )
        
        # Rviz
        rviz = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'sim_config.rviz')],
                output='screen'
        )

        # Teleop keyboard
        teleop_keyboard = Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                output='screen',
                prefix = 'xterm -e',
                parameters=[{'stamped': True}],
                remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')]
                        
        )

        twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
        twist_mux = Node(
                package="twist_mux",
                executable="twist_mux",
                parameters=[twist_mux_params, {'use_sim_time': True}],
                remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

        twist_stamper = Node(
                package='twist_stamper',
                executable='twist_stamper',
                parameters=[{'use_sim_time': 'true'}],
                remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
                        ('/cmd_vel_out','/diff_cont/cmd_vel')]
        )

        return LaunchDescription([
                robot_state_publisher,
                spawn,
                bridge,
                gazebo,
                diff_drive_controller_spawner,
                joint_state_broadcaster_spawner,
                rviz,
                teleop_keyboard
        ])