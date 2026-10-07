"""
Find and stop ROS processes a previous run left behind.

This is the "overlapping map" bug. robot.sh used to clean up with
`pkill -9 -f python`, which killed the `ros2 launch` parents - they are
Python - but not slam_toolbox or Nav2, which are C++. Those orphans kept
running, and the next run started a SECOND slam_toolbox beside the first:
two nodes publishing /map and map->odom, each with its own idea of where the
robot is, and a saved map that was both of their maps pasted on top of each
other at different angles.

Matching is by WHAT A PROCESS IS RUNNING - its executable, or the script its
interpreter was given - never by a word appearing anywhere on its command
line. A substring match would also hit an editor with explore.launch.py open,
or any shell whose command happens to mention slam_toolbox.

Two scopes:

    missions  slam_toolbox, Nav2, the explorer and map nodes, mission launch
              files and launch/realbot scripts. What mode_manager sweeps
              before starting a mission; bringup is left alone.
    all       every ROS node from /opt/ros or this workspace, and every
              `ros2 launch`. What robot.sh sweeps at startup. Still spares the
              web bridge (it reconnects by itself), RViz/rqt and the daemon.

The calling process and all of its ancestors are always excluded.

    python3 -m nexva_explore.stale_procs --scope all --kill
"""

import argparse
import os
import signal
import sys
import time

MISSION_BINARIES = (
    'async_slam_toolbox_node',
    'sync_slam_toolbox_node',
)
MISSION_NODES = (
    'frontier_explorer',
    'auto_clean',
    'map_autosaver',
    'map_updater',
)
MISSION_LAUNCHES = (
    'explore.launch.py',
    'clean.launch.py',
    'manual.launch.py',
    'navigation.launch.py',
    'slam.launch.py',
)

# Never swept, in either scope: the web UI (it reconnects to whatever comes
# up next) and the operator's own viewers.
SPARED = (
    'web_bridge',
    'web.launch.py',
    'rviz2',
)

SHELLS = ('bash', 'sh', 'dash')


def _argv(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as handle:
            raw = handle.read()
    except OSError:
        return None

    if not raw:
        return None                         # kernel thread or zombie

    return [part.decode('utf-8', 'replace') for part in raw.split(b'\0') if part]


def _ancestors(pid):
    """The pid, its parent, its parent's parent... up to init."""
    chain = set()

    while pid > 1 and pid not in chain:
        chain.add(pid)
        try:
            with open(f'/proc/{pid}/stat') as handle:
                # comm can contain spaces and parens; ppid follows the last ')'
                pid = int(handle.read().rsplit(')', 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break

    return chain


def _program(argv):
    """What is actually being run: the script for an interpreter, else argv[0]."""
    exe = argv[0]

    if os.path.basename(exe).startswith('python') and len(argv) > 1:
        # `python3 -m pkg` or `python3 -c ...` run nothing on disk we can name.
        if not argv[1].startswith('-'):
            return argv[1]

    return exe


def _is_ros2_launch(argv):
    prog = _program(argv)
    return (os.path.basename(prog) == 'ros2'
            and 'launch' in argv[1:4])


def is_mission(argv):
    prog = _program(argv)
    base = os.path.basename(prog)

    if base in MISSION_BINARIES:
        return True

    if prog.startswith('/opt/ros/') and '/lib/nav2_' in prog:
        return True

    if base.startswith('component_container'):
        return True

    if base in MISSION_NODES and '/nexva_explore/' in prog:
        return True

    if _is_ros2_launch(argv) and any(arg.endswith(MISSION_LAUNCHES) for arg in argv):
        return True

    if (os.path.basename(argv[0]) in SHELLS and len(argv) > 1
            and '/launch/realbot/robot_' in argv[1] and argv[1].endswith('.sh')):
        return True

    return False


def is_ros(argv, workspace):
    """Any ROS process from /opt/ros or this workspace, or any ros2 launch."""
    if is_mission(argv):
        return True

    prog = _program(argv)

    if prog.startswith('/opt/ros/') and '/lib/' in prog:
        return True

    if workspace and prog.startswith(os.path.join(workspace, 'install') + os.sep):
        return True

    if _is_ros2_launch(argv):
        return True

    # A robot.sh from an earlier session still running in another terminal.
    if (workspace and os.path.basename(argv[0]) in SHELLS and len(argv) > 1
            and os.path.basename(argv[1]) == 'robot.sh'):
        return True

    return False


def find(scope='missions', workspace=None, exclude=()):
    """{pid: argv} of this user's processes that match `scope`."""
    uid = os.getuid()
    skip = set(exclude) | _ancestors(os.getpid())
    found = {}

    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue

        pid = int(entry)

        if pid in skip:
            continue

        try:
            if os.stat(f'/proc/{pid}').st_uid != uid:
                continue
        except OSError:
            continue

        argv = _argv(pid)

        if not argv:
            continue

        if (any(os.path.basename(arg) in SPARED for arg in argv)
                or os.path.basename(_program(argv)).startswith('rqt')):
            continue

        if scope == 'missions':
            matched = is_mission(argv)
        else:
            matched = is_ros(argv, workspace)

        if matched:
            found[pid] = argv

    return found


def alive(pid):
    """Running, and not just a zombie waiting to be reaped."""
    try:
        with open(f'/proc/{pid}/stat') as handle:
            return handle.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except (OSError, IndexError):
        return False


def stop(pids, grace=5.0):
    """SIGINT, `grace` seconds to exit, then SIGKILL. Returns those killed."""
    for pid in pids:
        try:
            os.kill(pid, signal.SIGINT)
        except OSError:
            pass

    deadline = time.time() + grace

    while time.time() < deadline and any(alive(pid) for pid in pids):
        time.sleep(0.2)

    killed = []

    for pid in pids:
        if alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
                killed.append(pid)
            except OSError:
                pass

    # Reap any that were our own children, so they do not linger as zombies.
    for pid in pids:
        try:
            os.waitpid(pid, os.WNOHANG)
        except OSError:
            pass

    return killed


def main(argv=None):
    parser = argparse.ArgumentParser(prog='stale_procs')
    parser.add_argument('--scope', choices=('missions', 'all'), default='missions')
    parser.add_argument('--workspace', default=os.environ.get('NEXVA_WS', ''))
    parser.add_argument('--kill', action='store_true')
    parser.add_argument('--grace', type=float, default=5.0)
    parser.add_argument('--prefix', default='[stale]')
    args = parser.parse_args(argv)

    found = find(args.scope, args.workspace)

    if not found:
        return 0

    for pid, cmd in sorted(found.items()):
        print(f'{args.prefix}   pid {pid}: {" ".join(cmd)[:120]}')

    if args.kill:
        killed = stop(list(found), args.grace)
        print(f'{args.prefix}   stopped {len(found)}'
              + (f' ({len(killed)} needed SIGKILL)' if killed else ''))

    return 0


if __name__ == '__main__':
    sys.exit(main())
