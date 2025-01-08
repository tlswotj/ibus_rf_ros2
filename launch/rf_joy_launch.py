import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # rf_publisher_node 실행
        Node(
            package='rf_joy',
            executable='rf_publisher_node',
            name='rf_publisher_node',
            output='screen'
        ),
        
        # rf_to_joy_node 실행
        Node(
            package='rf_joy',
            executable='rf_to_joy_node',
            name='rf_to_joy_node',
            output='screen'
        ),
    ])
