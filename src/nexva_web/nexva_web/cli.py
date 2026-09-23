"""Command-line waypoint control - the whole ROS path with no web layer.

    ros2 run nexva_web waypoint_cli list
    ros2 run nexva_web waypoint_cli check
    ros2 run nexva_web waypoint_cli init "Home"
    ros2 run nexva_web waypoint_cli goto "3D Printing"
    ros2 run nexva_web waypoint_cli tour "Home" "3D Printing"

Use this to prove the waypoint file, the map check and the initial-pose
sequence against the real robot before trusting any of it from a browser.
"""

import argparse
import os
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

from . import mapcheck
from . import waypoints as wp_mod
from .nav_client import NavClient


def default_waypoint_file():
    env = os.environ.get('NEXVA_WAYPOINTS')
    if env:
        return env
    return os.path.join(
        get_package_share_directory('nexva_web'), 'config', 'waypoints.yaml')


def find_map_yaml(map_name):
    """Locate <map_name>.yaml without needing map_server to be running."""
    candidates = []
    env = os.environ.get('NEXVA_MAP_DIR')
    if env:
        candidates.append(env)
    try:
        candidates.append(os.path.join(
            get_package_share_directory('nexva_navigation'), 'maps'))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.abspath(os.path.join(
        here, '..', '..', 'nexva_navigation', 'maps')))
    for d in candidates:
        path = os.path.join(d, map_name + '.yaml')
        if os.path.isfile(path):
            return path
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(prog='waypoint_cli')
    ap.add_argument('-f', '--file', default=None, help='waypoint YAML file')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('list', help='list waypoints in the file')
    sub.add_parser('check', help='verify the loaded map matches the file')
    p = sub.add_parser(
        'validate', help='check every waypoint sits in reachable free space')
    p.add_argument('--map-yaml', default=None,
                   help='map to check against (default: ask map_server)')
    p = sub.add_parser('init', help='seed AMCL from a waypoint')
    p.add_argument('name')
    p = sub.add_parser('goto', help='navigate to a waypoint')
    p.add_argument('name')
    p = sub.add_parser('tour', help='visit several waypoints in order')
    p.add_argument('names', nargs='+')
    p.add_argument('--loops', type=int, default=0)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    path = args.file or default_waypoint_file()
    try:
        wps = wp_mod.load(path)
    except wp_mod.WaypointError as exc:
        print('error: %s' % exc, file=sys.stderr)
        return 1

    if args.cmd == 'list':
        print('%s  (map: %s, frame: %s)' % (path, wps.map_name, wps.frame_id))
        for w in wps.waypoints:
            print('  %-20s x %8.3f  y %8.3f  qz %7.4f  qw %7.4f'
                  % (w.name, w.x, w.y, w.qz, w.qw))
        return 0

    # Offline: checks the map file directly, so it works with nothing running.
    if args.cmd == 'validate':
        map_yaml = args.map_yaml or find_map_yaml(wps.map_name)
        if not map_yaml:
            print('error: could not find %s.yaml - pass --map-yaml'
                  % wps.map_name, file=sys.stderr)
            return 2
        try:
            grid = mapcheck.OccupancyMap(map_yaml)
        except mapcheck.MapError as exc:
            print('error: %s' % exc, file=sys.stderr)
            return 2
        x0, x1, y0, y1 = grid.extent
        print('map %s: %dx%d @ %.3f m  ->  x %.2f..%.2f  y %.2f..%.2f\n'
              % (grid.name, grid.width, grid.height, grid.resolution,
                 x0, x1, y0, y1))
        results = [mapcheck.check_waypoint(grid, w) for w in wps.waypoints]
        for r in results:
            near = ('%.2f m at %+.0f deg' % r['nearest']
                    if r['nearest'] else 'none within 1.5 m')
            print('%-14s x %7.3f  y %7.3f  yaw %+6.1f  %-8s  nearest %s'
                  % (r['name'], r['x'], r['y'], r['yaw'], r['cell'], near))
            for problem in r['problems']:
                print('               PROBLEM: %s' % problem)
        bad = [r for r in results if not r['ok']]
        print('\n%d of %d waypoints OK' % (len(results) - len(bad), len(results)))
        if bad:
            print('unreachable: %s' % ', '.join(r['name'] for r in bad))
        return 0 if not bad else 2

    rclpy.init()
    node = NavClient(wps)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    done = threading.Event()
    result = {}

    def on_event(evt):
        if evt['type'] == 'feedback':
            if 'distance_remaining' in evt:
                print('\r  %.2f m to go, eta %.0fs, recoveries %d        '
                      % (evt['distance_remaining'], evt['eta'],
                         evt['recoveries']), end='', flush=True)
            else:
                print('\r  waypoint %d/%d: %s            '
                      % (evt['waypoint_index'] + 1, evt['total'],
                         evt['waypoint_name']), end='', flush=True)
        elif evt['type'] == 'result':
            result.update(evt)
            done.set()

    node.on_event = on_event
    rc = 0
    try:
        ok, detail = node.check_map()
        if ok is True:
            print('map OK: %s' % detail)
        elif ok is False:
            print('MAP MISMATCH: %s' % detail, file=sys.stderr)
            if args.cmd in ('goto', 'tour'):
                return 2
        else:
            print('warning: could not verify map (%s)' % detail, file=sys.stderr)

        if args.cmd == 'check':
            return 0 if ok else 2

        if args.cmd == 'init':
            print('Seeding AMCL at %r.' % args.name)
            print('This asserts the robot is PHYSICALLY at that spot right now.')
            if input('Is it? [y/N] ').strip().lower() not in ('y', 'yes'):
                print('aborted')
                return 1
            if node.set_initial_pose(args.name):
                print('AMCL converged.')
            else:
                print('AMCL has not converged yet - check /amcl_pose.',
                      file=sys.stderr)
                rc = 1
            return rc

        if args.cmd == 'goto':
            node.goto(args.name)
        else:
            node.tour(args.names, loops=args.loops)

        print('navigating... (Ctrl-C to stop the robot)')
        while not done.wait(timeout=0.2):
            pass
        print('\n%s: %s %s' % (result.get('destination'), result.get('status'),
                               result.get('detail', '')))
        rc = 0 if result.get('status') == 'SUCCEEDED' else 1

    except wp_mod.WaypointError as exc:
        print('error: %s' % exc, file=sys.stderr)
        rc = 1
    except KeyboardInterrupt:
        print('\ninterrupted - stopping robot')
        node.estop()
        time.sleep(0.5)
        rc = 130
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        spin.join(timeout=2.0)
    return rc


if __name__ == '__main__':
    sys.exit(main())
