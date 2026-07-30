# Copyright 2026 gongbang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the i-BUS receiver node together with the Joy converter node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Bring up rf_publisher_node and rf_to_joy_node with their parameter files."""
    package_share = get_package_share_directory('rf_joy')
    default_serial_config = os.path.join(package_share, 'config', 'rf_publisher.yaml')
    default_joy_config = os.path.join(package_share, 'config', 'rf_to_joy.yaml')

    serial_config = LaunchConfiguration('serial_config')
    joy_config = LaunchConfiguration('joy_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_config',
            default_value=default_serial_config,
            description='Path to the rf_publisher parameter file',
        ),
        DeclareLaunchArgument(
            'joy_config',
            default_value=default_joy_config,
            description='Path to the rf_to_joy parameter file',
        ),
        Node(
            package='rf_joy',
            executable='rf_publisher_node',
            output='screen',
            parameters=[serial_config],
        ),
        Node(
            package='rf_joy',
            executable='rf_to_joy_node',
            output='screen',
            parameters=[joy_config],
        ),
    ])
