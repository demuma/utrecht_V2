from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    emit_shutdown_action = Shutdown(reason='Shutting down')

    return LaunchDescription([
        Node(
            package='robotic_arm',
            executable='driver',
            name='robotic_arm',
            output='screen',
            on_exit=[emit_shutdown_action],
        ),
        Node(
            package='robotic_arm',
            executable='telemetry',
            name='telemetry',
            output='screen',
            on_exit=[emit_shutdown_action],
        ),
        # This must be included last
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
