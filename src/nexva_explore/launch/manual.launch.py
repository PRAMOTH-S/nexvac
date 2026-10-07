"""
Map by hand: slam_toolbox + the on-demand map saver, nothing that drives.

Drive from the web joystick (or any cmd_vel publisher) and the map builds in
memory as you go. NOTHING is written to disk until you ask: the web UI's
"Save map" button (one message on /save_map) makes `map_saver` write
~/nexva_maps/<map_source>/<map_name>.{pgm,yaml} and record it in
~/nexva_maps/map.md. There is no autosave - press Save when you are done.
Expects bringup to already be running - this starts no hardware.

Usage:
  ros2 launch nexva_explore manual.launch.py map_name:=kitchen
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    slam_dir = get_package_share_directory('nexva_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_source = LaunchConfiguration('map_source')
    map_dir = LaunchConfiguration('map_dir')
    map_name = LaunchConfiguration('map_name')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use the simulation clock (never on the real robot)',
    )
    declare_map_source = DeclareLaunchArgument(
        'map_source', default_value='hardware',
        description="Where the map came from: 'hardware' or 'sim'. Each has "
                    'its own folder and its own map.md entry',
    )
    declare_map_dir = DeclareLaunchArgument(
        'map_dir', default_value=PathJoinSubstitution(
            [os.path.expanduser('~/nexva_maps'), map_source]),
        description='Directory the SLAM map is saved into (on demand)',
    )
    declare_map_name = DeclareLaunchArgument(
        'map_name', default_value='manual',
        description='Base filename (no extension) for the saved map',
    )

    # slam_toolbox with nexva_slam's tuning. Its launch defaults use_sim_time
    # to true, so it has to be passed explicitly.
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'slam.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # The ONLY thing that writes the map here: the web UI's Save map button.
    # Nothing is written until a /save_map request arrives (autosave: False;
    # there is no autosaver in this launch).
    map_saver_node = Node(
        package='nexva_explore',
        executable='map_saver',
        name='map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_dir': map_dir,
            'map_name': map_name,
            'map_source': map_source,
            'autosave': False,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_source)
    ld.add_action(declare_map_dir)
    ld.add_action(declare_map_name)
    ld.add_action(slam_cmd)
    ld.add_action(map_saver_node)
    return ld
