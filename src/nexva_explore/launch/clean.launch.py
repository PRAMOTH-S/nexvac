"""
Clean a map that was saved on an earlier run - no exploring.

Nav2 with AMCL localises against the saved map (nexva_navigation's
navigation.launch.py, so the tuned nav2_params.yaml is reused and the web
UI's waypoint and zone features stay alive), the autosaver keeps a copy of
/map under ~/nexva_maps/<map_source>/<map_name>.*, and the cleaner
(frontier_explorer started in `clean` mode) sweeps the floor. Expects bringup
to already be running - this starts no hardware.

The map may be one the robot autosaved (~/nexva_maps/hardware/<name>.yaml) or
one shipped in nexva_navigation/maps. Only the former is ever overwritten by
the autosaver, and before it is, a pristine `.as_mapped` copy is kept.

POSE SEEDING. When the map was saved by an explore run it has a pose file
(~/nexva_maps/<map_name>.pose.yaml: the robot's 2D pose + heading when the map
was saved). `initial_pose_seeder` hands that to AMCL on /initialpose so the
robot starts localised instead of guessing, and the cleaner starts once the
seeder has finished. It lives in this launch (not in robot_clean_saved.sh)
because only here are the map yaml and the launch ordering known: the seeder
must run after AMCL exists and before the cleaner plans from map->base. If the
file is missing, stale or for another map the seeder says so and does nothing -
the cleaner then starts on the same 9 s timer as before. seed_pose:=false
turns the whole thing off.

Usage:
  ros2 launch nexva_explore clean.launch.py map:=/home/me/nexva_maps/hardware/kitchen.yaml
"""

import os
import shutil
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from nexva_explore import map_registry

# Give map_server and AMCL time to activate and settle on a pose before the
# cleaner asks for map->base_footprint. Same figure as the sim.
CLEANER_DELAY = 9.0

BACKUP_MARKER = '.as_mapped'


def requested(key, env_var):
    """
    A launch argument's value, before launch arguments exist.

    The backup below has to happen while the launch description is being
    built, so `map:=` and `map_source:=` are read off the command line
    directly, and failing that from the environment the robot_*.sh scripts
    export. Empty when neither says.
    """
    prefix = key + ':='

    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]

    return os.environ.get(env_var) or ''


def keep_as_mapped(map_yaml, map_dir):
    """
    Keep one pristine copy of the map the run started from.

    The autosaver writes /map back to <map_dir>/<map_name>.* while cleaning.
    When that is the same file the map was loaded from, the map the
    exploration run built is progressively replaced by a re-rendering of
    itself - measured in the sim, about a quarter of the occupied cells gone
    over one sweep, compounding every run. Updating the map while cleaning is
    wanted; destroying the original is not, and it is not recoverable once
    done.

    Only done for maps inside map_dir. A map from nexva_navigation/maps is
    never written to by the autosaver, so it needs no backup - and a backup
    file must not land in the source tree.
    """
    if not map_yaml or not os.path.isfile(map_yaml):
        return

    yaml_dir = os.path.dirname(os.path.abspath(map_yaml))

    if os.path.abspath(yaml_dir) != os.path.abspath(map_dir):
        return

    base = os.path.splitext(map_yaml)[0]
    backup = base + BACKUP_MARKER

    if os.path.exists(backup + '.pgm'):
        return

    try:
        shutil.copy2(base + '.pgm', backup + '.pgm')
        shutil.copy2(map_yaml, backup + '.yaml')
        print(f'[clean] kept the as-mapped copy: {backup}.pgm')
    except OSError as exc:
        print(f'[clean] could not back up the map: {exc}')


def generate_launch_description():
    nav_dir = get_package_share_directory('nexva_navigation')
    explore_dir = get_package_share_directory('nexva_explore')

    # Resolved at build time, for the backup and the defaults. The launch
    # arguments below carry the same values for the nodes.
    source = requested('map_source', 'MAP_SOURCE') or map_registry.DEFAULT_SOURCE
    default_map_dir = os.path.join(map_registry.maps_root(), source)
    default_map = requested('map', 'MAP')

    if not default_map:
        # Nothing said: the last map this source saved, as the sim did.
        default_map = map_registry.saved_map_file(source=source) or ''

        if default_map:
            print(f'[clean] map.md: cleaning the last {source} map, {default_map}')
        else:
            print('[clean] no map given and none recorded in '
                  f'{map_registry.registry_path()} - pass map:=/path/to/map.yaml')

    default_map_name = (os.environ.get('MAP_NAME')
                        or (os.path.splitext(os.path.basename(default_map))[0]
                            if default_map else 'clean'))

    map_dir_now = requested('map_dir', 'MAP_DIR') or default_map_dir
    keep_as_mapped(default_map, map_dir_now)

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    map_source = LaunchConfiguration('map_source')
    map_dir = LaunchConfiguration('map_dir')
    map_name = LaunchConfiguration('map_name')
    params_file = LaunchConfiguration('params_file')
    seed_pose = LaunchConfiguration('seed_pose')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use the simulation clock (never on the real robot)',
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Saved map .yaml to clean; defaults to the one named in map.md',
    )
    declare_map_source = DeclareLaunchArgument(
        'map_source', default_value=source,
        description="Which map.md entry this run updates: 'hardware' or 'sim'",
    )
    declare_map_dir = DeclareLaunchArgument(
        'map_dir', default_value=PathJoinSubstitution(
            [map_registry.maps_root(), map_source]),
        description='Directory to save the updated map into while cleaning',
    )
    declare_map_name = DeclareLaunchArgument(
        'map_name', default_value=default_map_name,
        description='Base filename for the updated map',
    )
    declare_seed_pose = DeclareLaunchArgument(
        'seed_pose', default_value='true',
        description='Seed AMCL from the map\'s saved pose file when it is valid',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(explore_dir, 'config', 'clean.yaml'),
        description='frontier_explorer parameters for clean mode',
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(nav_dir, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters (AMCL, costmaps, planner, controller)',
    )

    # Nav2 + AMCL against the saved map - exactly what robotnav.sh has always
    # started, so the web UI's goals, waypoints and zones keep working while
    # the cleaner runs.
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'slam': 'False',
        }.items(),
    )

    # Writes /map back out as it changes, and refreshes map.md. Under Nav2
    # the map_server publishes /map once, so this is one save that records
    # the cleaned map under its name; slam_toolbox's serialise service is
    # absent here and the autosaver says so once, harmlessly.
    map_autosaver_node = Node(
        package='nexva_explore',
        executable='map_autosaver',
        name='map_autosaver',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_dir': map_dir,
            'map_name': map_name,
            'map_source': map_source,
        }],
    )

    # Saves /map (+ pose + graph if available) on a /save_map request.
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

    # Hands AMCL the pose the map was saved at, then exits. Guarded inside.
    seeder_node = Node(
        package='nexva_explore',
        executable='initial_pose_seeder',
        name='initial_pose_seeder',
        output='screen',
        condition=IfCondition(seed_pose),
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_name': map_name,
            'map_yaml': map_yaml,
            'map_source': map_source,
            'min_runtime': CLEANER_DELAY,
        }],
    )

    # The explorer, started in clean mode. Named frontier_explorer as in the
    # sim so its topics (~/command, robot_status, coverage_blocks) are the
    # same whichever mission started it.
    def make_cleaner():
        return Node(
            package='nexva_explore',
            executable='auto_clean',
            name='frontier_explorer',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'start_mode': 'clean',
                    'map_name': map_name,
                },
            ],
        )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_map_source)
    ld.add_action(declare_map_dir)
    ld.add_action(declare_map_name)
    ld.add_action(declare_seed_pose)
    ld.add_action(declare_params_file)
    ld.add_action(declare_nav2_params_file)
    ld.add_action(nav_cmd)
    ld.add_action(map_autosaver_node)
    ld.add_action(map_saver_node)
    ld.add_action(seeder_node)
    # Seeding on: the cleaner follows the seeder (which never exits before
    # CLEANER_DELAY, so the timing matches today's). Off: the plain timer.
    ld.add_action(TimerAction(period=CLEANER_DELAY,
                              actions=[make_cleaner()],
                              condition=UnlessCondition(seed_pose)))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=seeder_node,
                      on_exit=[TimerAction(period=1.0,
                                           actions=[make_cleaner()])]),
        condition=IfCondition(seed_pose)))
    return ld
