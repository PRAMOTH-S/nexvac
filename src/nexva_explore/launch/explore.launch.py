"""
Autonomous exploration: SLAM + on-demand map saver + pose store + explorer.

Brings up slam_toolbox (nexva_slam's tuning) to build the map, and the
frontier explorer to drive the robot through the whole reachable area on its
own. Expects bringup to already be running - this starts no hardware.

NO AUTOSAVE. `map_autosaver` is deliberately NOT started here any more: the
map is no longer written to disk on every SLAM update. It is saved on demand,
by one message on /save_map (the web UI's "Save map" button, or the explorer
itself when exploration finishes), handled by `map_saver`, which writes the
.pgm/.yaml, map.md, the slam_toolbox pose graph and the robot's pose together.
`pose_store` keeps ~/nexva_maps/<map_name>.pose.yaml current until that save.

The explorer's tuning lives in config/explore.yaml; the launch only adds the
things that change per run (the map name, the clock).

Usage:
  ros2 launch nexva_explore explore.launch.py map_name:=kitchen
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# NO START DELAY. The explorer used to be held back by
# TimerAction(period=8.0) so SLAM could publish a first map and map->odom.
# Eight seconds was a guess: too short on a cold Pi 5 (the explorer starts
# driving with no pose, robot_pose() returns None and the first plan is
# nonsense) and wasted time when SLAM was ready in two. The explorer now
# gates itself on the real condition instead - a FRESH map -> base_footprint
# transform plus a /map - holding station with a zero Twist until then. See
# `gate_localization` in frontier_explorer.py. It is started immediately so
# that it is up, publishing its mode and its zero Twist, and visible to the
# web page from the first second of the mission.


def generate_launch_description():
    slam_dir = get_package_share_directory('nexva_slam')
    explore_dir = get_package_share_directory('nexva_explore')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_source = LaunchConfiguration('map_source')
    map_dir = LaunchConfiguration('map_dir')
    map_name = LaunchConfiguration('map_name')
    params_file = LaunchConfiguration('params_file')

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
        description='Directory a saved map is written into (on demand)',
    )
    declare_map_name = DeclareLaunchArgument(
        'map_name', default_value='explore',
        description='Default name for a saved map (the save request may override)',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(explore_dir, 'config', 'explore.yaml'),
        description='frontier_explorer parameters for explore mode',
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

    # Saves /map + pose + pose graph when asked (/save_map). Writes nothing
    # on its own.
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

    # Keeps <map_name>.pose.yaml current (throttled, atomic) until a save pins it.
    pose_store_node = Node(
        package='nexva_explore',
        executable='pose_store',
        name='pose_store',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_name': map_name,
            'map_source': map_source,
        }],
    )

    # Topics are un-namespaced on this robot, so no tf remaps are needed:
    # 'tf' relative to the root namespace IS /tf.
    frontier_explorer_node = Node(
        package='nexva_explore',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'start_mode': 'explore',
                'map_name': map_name,
            },
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_source)
    ld.add_action(declare_map_dir)
    ld.add_action(declare_map_name)
    ld.add_action(declare_params_file)
    ld.add_action(slam_cmd)
    ld.add_action(map_saver_node)
    ld.add_action(pose_store_node)
    ld.add_action(frontier_explorer_node)
    return ld
