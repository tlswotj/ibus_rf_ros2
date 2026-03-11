from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share = get_package_share_directory("rf_joy")
    default_serial_config = os.path.join(package_share, "config", "rf_publisher.yaml")
    default_joy_config = os.path.join(package_share, "config", "rf_to_joy.yaml")

    serial_config = LaunchConfiguration("serial_config")
    joy_config = LaunchConfiguration("joy_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_config",
            default_value=default_serial_config,
            description="Path to the rf_publisher parameter file",
        ),
        DeclareLaunchArgument(
            "joy_config",
            default_value=default_joy_config,
            description="Path to the rf_to_joy parameter file",
        ),
        Node(
            package="rf_joy",
            executable="rf_publisher_node",
            output="screen",
            parameters=[serial_config],
        ),
        Node(
            package="rf_joy",
            executable="rf_to_joy_node",
            output="screen",
            parameters=[joy_config],
        ),
    ])
