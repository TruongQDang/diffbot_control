import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
        # Package name
        package_name = 'diffbot_control'

        # Process URDF
        xacro_file = os.path.join(
                get_package_share_directory(package_name),
                'urdf', 
                'robot.urdf.xacro'
        )
        robot_description = Command(['xacro ', xacro_file])
        
        # Robot state publisher
        robot_config = {'robot_description': robot_description}
        robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name="robot_state_publisher",
                output='both',
                parameters=[robot_config],
        )

        # ROS2 control
        controllers_config = PathJoinSubstitution(
        [
                FindPackageShare(package_name),
                "config",
                "ros2_controllers.yaml"
        ]
        )
        controller_manager = Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_config, controllers_config],
                output="both"
        )
        joint_state_broadcaster_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )
        diff_drive_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        )

        # Delay start of robot_controller after `joint_state_broadcaster`
        delay_diff_controller = RegisterEventHandler(
                event_handler=OnProcessExit(
                        target_action=joint_state_broadcaster_spawner,
                        on_exit=[diff_drive_controller_spawner],
                )
        )

        # # MySLAM
        # myslam = IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([os.path.join(
        #                 get_package_share_directory('myslam'),'launch','online_async_launch.py'
        #         )]), launch_arguments={'use_sim_time': 'false'}.items()
        # )

        # # Micro-ROS-Agent
        # micro_ros_agent = ExecuteProcess(
        #         cmd=[
        #                 'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
        #                 'serial', '--dev', '/dev/ttyACM0'
        #         ],
        #         output='screen'
        # )

        # # RPLIDAR
        # rplidar = IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([os.path.join(
        #                 get_package_share_directory('slllidar_ros2'),'launch','sllidar_c1_launch.py'
        #         )]), launch_arguments={'frame_id': 'lidar_frame'}.items()
        # )


        return LaunchDescription([
                controller_manager,
                robot_state_publisher,
                joint_state_broadcaster_spawner,
                delay_diff_controller,
                # myslam,
                # micro_ros_agent,
                # rplidar
        ])