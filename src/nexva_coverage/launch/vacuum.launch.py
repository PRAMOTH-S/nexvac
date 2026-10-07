"""Everything above Nav2, in one command.

The reason this file exists: the zone planner, the coverage estimator and the
web bridge are three separate nodes, and every time one of them was started by
hand it eventually ended up missing - which looks exactly like "the website is
broken" from the outside. Starting them together removes that whole class of
problem.

What this does NOT start: the robot bringup and Nav2. Bring those up the way
you already do. Nothing here changes the navigation tuning.

  ros2 launch nexva_bringup bringup.launch.py        # robot + lidar + odom
  ros2 launch nexva_navigation navigation.launch.py map:=<your map>
  ros2 launch nexva_coverage vacuum.launch.py        # <- this file
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cov_share = get_package_share_directory('nexva_coverage')
    web_share = get_package_share_directory('nexva_web')

    params = LaunchConfiguration('params')
    zones = LaunchConfiguration('zones')
    waypoints = LaunchConfiguration('waypoints')
    port = LaunchConfiguration('port')
    kill_teleop = LaunchConfiguration('kill_teleop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=os.path.join(cov_share, 'config',
                                       'zone_coverage.yaml'),
            description='Coverage tuning (swath, overlap, retries)'),
        DeclareLaunchArgument(
            'zones',
            default_value=os.path.join(cov_share, 'config', 'zones.yaml'),
            description='Saved zone polygons'),
        DeclareLaunchArgument(
            'waypoints',
            default_value=os.path.join(web_share, 'config', 'waypoints.yaml'),
            description='Named waypoints for point-to-point goals'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument(
            'kill_teleop', default_value='true',
            description='Stop stray teleop_twist_keyboard nodes, which '
                        'publish /cmd_vel and fight Nav2 for the base'),

        # teleop_twist_keyboard left running from an earlier session competes
        # with Nav2 for /cmd_vel. Clear it before anything else starts.
        ExecuteProcess(
            condition=IfCondition(kill_teleop),
            cmd=['bash', '-c',
                 'pkill -f "[t]eleop_twist_keyboard" 2>/dev/null; exit 0'],
            name='clear_teleop', output='screen'),

        # Tracks where the robot has actually been. Without it the planner's
        # coverage target and gap-fill do nothing.
        Node(package='nexva_coverage', executable='coverage_estimator',
             name='coverage_estimator', output='screen', parameters=[params]),

        # Plans a drawn zone and drives it through the existing Nav2 stack.
        Node(package='nexva_coverage', executable='zone_coverage',
             name='zone_coverage', output='screen',
             parameters=[params, {'zones_file': zones}]),

        # The operator UI: map, draw a zone, waypoints, teleop, stop.
        Node(package='nexva_web', executable='web_bridge',
             name='nexva_web_bridge', output='screen',
             arguments=['-f', waypoints, '--host', '0.0.0.0', '--port', port]),
    ])
