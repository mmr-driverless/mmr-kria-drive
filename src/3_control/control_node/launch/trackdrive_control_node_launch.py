from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
  config = os.path.join(get_package_share_directory('control_node'), 'config', 'control_node_trackdrive.yaml')

  os.environ['ROS_LOG_DIR'] = '/home/root/control_log/'

  node = Node(
    name="control_node",
    package="control_node",
    executable="control_node",
    parameters=[config]
  )

  return LaunchDescription([ node ])