from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_dir = get_package_share_directory("ros2_ars5_ethernet_cpp")
    config_file = os.path.join(package_dir, "config", "ars5.yaml")

    return LaunchDescription([
        Node(
            package="ros2_ars5_ethernet_cpp",
            executable="ars5_node",
            name="ars5_node",
            output="screen",
            parameters=[config_file],
        )
    ])
