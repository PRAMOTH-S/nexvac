"""Start the coverage planner against an ALREADY RUNNING Nav2 stack.

This launch deliberately starts nothing else. Bring up the robot and Nav2 the
way you normally do; this only adds the planner that feeds them goals, so the
navigation tuning stays exactly as it is.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('nexva_coverage'), 'config', 'coverage.yaml')

    params = LaunchConfiguration('params')

    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params,
                              description='Coverage tuning YAML'),
        # The estimator must come up with the planner: without it the planner's
        # gap-fill, waypoint skipping and coverage_target are inert, and it
        # says so in a startup warning.
        Node(
            package='nexva_coverage',
            executable='coverage_estimator',
            name='coverage_estimator',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nexva_coverage',
            executable='coverage_planner',
            name='coverage_planner',
            output='screen',
            parameters=[params],
        ),
    ])
