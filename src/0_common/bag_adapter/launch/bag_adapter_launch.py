from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():


    node=Node(
        package='bag_adapter',
        name='bag_adapter_node',
        executable='bag_adapter_node',
    )

    return LaunchDescription([
        node
    ])