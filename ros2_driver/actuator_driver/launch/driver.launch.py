"""ros2 launch actuator_driver driver.launch.py url:=tcp://host.docker.internal:29536"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("actuator_driver"), "config", "driver.yaml")
    url = LaunchConfiguration("url")
    mode = LaunchConfiguration("command_mode")
    return LaunchDescription([
        DeclareLaunchArgument("url", default_value="tcp://localhost:29536",
                              description="SLCAN link to the actuators"),
        DeclareLaunchArgument("command_mode", default_value="velocity",
                              description="velocity, position or current"),
        Node(
            package="actuator_driver",
            executable="driver",
            name="actuator_driver",
            output="screen",
            parameters=[config, {"url": url, "command_mode": mode}],
        ),
    ])
