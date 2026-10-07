"""Zone cleaning, on top of an ALREADY RUNNING Nav2 stack.

Starts only the zone planner/executor. Bring up the robot and Nav2 as usual;
nothing about the navigation tuning is touched.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('nexva_coverage')

    params = LaunchConfiguration('params')
    zones = LaunchConfiguration('zones')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params', default_value=os.path.join(share, 'config',
                                                 'zone_coverage.yaml'),
            description='Coverage tuning YAML'),
        DeclareLaunchArgument(
            'zones', default_value=os.path.join(share, 'config', 'zones.yaml'),
            description='Zone definitions (polygons in map coordinates)'),
        Node(
            package='nexva_coverage',
            executable='zone_coverage',
            name='zone_coverage',
            output='screen',
            parameters=[params, {'zones_file': zones}],
        ),
    ])
