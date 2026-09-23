import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_waypoints = os.path.join(
        get_package_share_directory('nexva_web'), 'config', 'waypoints.yaml')

    waypoints = LaunchConfiguration('waypoints')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('waypoints', default_value=default_waypoints,
                              description='Waypoint YAML file'),
        DeclareLaunchArgument('host', default_value='0.0.0.0',
                              description='Bind address for the web UI'),
        DeclareLaunchArgument('port', default_value='8080',
                              description='Port for the web UI'),
        Node(
            package='nexva_web',
            executable='web_bridge',
            name='nexva_web_bridge',
            output='screen',
            arguments=['-f', waypoints, '--host', host, '--port', port],
        ),
    ])
