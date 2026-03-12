"""
Launch file for the Aeye ROS 2 LiDAR driver.

Usage examples:

  # Use the default YAML config (shipped with the package)
  ros2 launch aeye_ros2_driver aeye_driver.launch.py

  # Use a custom config file
  ros2 launch aeye_ros2_driver aeye_driver.launch.py \
      config_file:=/path/to/my_robot.yaml

  # Dual-sensor setup with per-sensor configs
  ros2 launch aeye_ros2_driver aeye_driver.launch.py \
      config_file:=config/front_lidar.yaml &
  ros2 launch aeye_ros2_driver aeye_driver.launch.py \
      config_file:=config/rear_lidar.yaml &

Parameter precedence (highest wins):
  1. YAML config file values
  2. Node-declared defaults (fallback if YAML omits a field)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Resolve the default config file shipped with the package
    pkg_share = get_package_share_directory('aeye_ros2_driver')
    default_config = os.path.join(pkg_share, 'config', 'default_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to YAML parameter file'
    )

    driver_node = Node(
        package='aeye_ros2_driver',
        executable='aeye_driver_node',
        name='aeye_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
        ],
    )

    return LaunchDescription([config_arg, driver_node])
