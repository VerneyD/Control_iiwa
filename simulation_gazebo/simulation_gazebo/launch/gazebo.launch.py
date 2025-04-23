import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_world', default_value='worlds/empty.world', description='Gazebo world file'),
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo_server',
            output='screen',
            parameters=[{'world_name': LaunchConfiguration('gazebo_world')}]
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gazebo_client',
            output='screen'
        ),
    ])
