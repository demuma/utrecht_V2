import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='remote',
            executable='remote',
            name='remote',
            output='screen'
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mqtt_client'),
                    'launch',
                    'standalone.launch.xml'
                )
            ),
            launch_arguments={'params_file': '/root/ros2_ws/src/bridge.yaml'}.items()
        ),
  ])
