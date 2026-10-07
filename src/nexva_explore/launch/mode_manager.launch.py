"""
The mode manager: the one process that decides which mission is running.

It starts nothing by itself unless `initial_mode` says so. robotnav.sh passes
`initial_mode:=navigate initial_map:=<map.yaml>` so the robot comes up
navigating exactly as before, and the web UI switches modes from there via
/set_robot_mode.

Usage:
  ros2 launch nexva_explore mode_manager.launch.py
  ros2 launch nexva_explore mode_manager.launch.py initial_mode:=navigate initial_map:=/abs/map.yaml
  ros2 launch nexva_explore mode_manager.launch.py initial_mode:=manual
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source = LaunchConfiguration('source')
    workspace = LaunchConfiguration('workspace')
    initial_mode = LaunchConfiguration('initial_mode')
    initial_map = LaunchConfiguration('initial_map')

    declare_source = DeclareLaunchArgument(
        'source', default_value='hardware',
        description="Which script table and map folder to use: 'hardware'",
    )
    declare_workspace = DeclareLaunchArgument(
        'workspace', default_value='',
        description='Workspace root holding launch/realbot/*.sh; empty means '
                    'NEXVA_WS, or walk up from the package to find build.sh',
    )
    declare_initial_mode = DeclareLaunchArgument(
        'initial_mode', default_value='',
        description='A mode to request once at startup (navigate, explore, '
                    'clean, manual); empty starts idle',
    )
    declare_initial_map = DeclareLaunchArgument(
        'initial_map', default_value='',
        description='Map for the initial mode: a saved map name, or an '
                    'absolute path to a map .yaml',
    )

    # Stopping a mission means waiting for Nav2 or slam_toolbox to shut down,
    # which takes longer than launch's 5 s default before it escalates to
    # SIGTERM and then SIGKILL. Give the manager room to finish the job.
    declare_sigterm_timeout = DeclareLaunchArgument(
        'sigterm_timeout', default_value='20',
        description='Seconds after SIGINT before launch sends SIGTERM',
    )
    declare_sigkill_timeout = DeclareLaunchArgument(
        'sigkill_timeout', default_value='10',
        description='Seconds after SIGTERM before launch sends SIGKILL',
    )

    mode_manager_node = Node(
        package='nexva_explore',
        executable='mode_manager',
        name='mode_manager',
        output='screen',
        parameters=[{
            'source': source,
            'workspace': workspace,
            'initial_mode': initial_mode,
            'initial_map': initial_map,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_source)
    ld.add_action(declare_workspace)
    ld.add_action(declare_initial_mode)
    ld.add_action(declare_initial_map)
    ld.add_action(declare_sigterm_timeout)
    ld.add_action(declare_sigkill_timeout)
    ld.add_action(mode_manager_node)
    return ld
