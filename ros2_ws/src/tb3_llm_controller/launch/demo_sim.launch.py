"""Jazzy / Gazebo Sim 8 launch using installed ROBOTIS worlds and models."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('turtlebot3_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('render_engine', default_value='ogre'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-v2', '--render-engine',
                            LaunchConfiguration('render_engine'),
                            os.path.join(share, 'worlds', 'turtlebot3_world.world')], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': '-2.0', 'y_pose': '-0.5'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
    ])
