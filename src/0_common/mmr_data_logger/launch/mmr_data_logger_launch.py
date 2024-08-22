from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config_node = os.path.join(
        get_package_share_directory('mmr_data_logger'),
        'config',
        'mmr_data_logger_conf.yaml'
    )

    node=Node(
        package='mmr_data_logger',
        name='mmr_data_logger_node',
        executable='mmr_data_logger_node',
        parameters=[
            config_node,
        ]
    )

    return LaunchDescription([
        node
    ])