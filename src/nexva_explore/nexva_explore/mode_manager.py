"""
The one thing that decides what the robot is doing.

The web UI and the terminal are both operator interfaces, and if each launched
its own scripts they would sooner or later launch two at once - two SLAM
instances fighting over `map -> odom`, two cleaners publishing `cmd_vel` at
each other, and a robot that behaves like neither. So neither of them launches
anything. They both send a request here, and this node owns the answer:

    robotnav.sh (initial_mode)  ─┐
                              ├──>  mode_manager  ──>  launch/realbot/robot_*.sh
    web UI (set_robot_mode)  ─┘         │
                                        └─ /robot_mode  (latched)

WHY A PROCESS SUPERVISOR AND NOT ONE BIG NODE

The missions need DIFFERENT localisation stacks underneath. Exploring and
manual mapping run slam_toolbox on a growing map; navigating and cleaning run
Nav2 with AMCL against a fixed one. A single long-lived node cannot swap the
thing that owns `map -> odom` out from under itself, so each mission is a
script that brings up its own stack on top of the always-running bringup, and
switching modes means stopping one script and starting another.

WHAT IT GUARANTEES

- One mission at a time. Switching stops the previous one and WAITS for it to
  be gone before starting the next; a half-dead SLAM still holds its topics.
- No duplicates. A request for the mode already running is answered, not
  re-launched.
- Failures are reported. A mission that dies inside its first few seconds is
  noticed and said out loud, rather than leaving the web UI claiming the
  robot is exploring while nothing is running.
- STOP always works, and always stops the wheels itself: after the mission
  is gone it publishes a burst of zero Twists on /cmd_vel rather than trusting
  the thing it just killed to have done so on the way out.

WHAT IT WILL NOT DO

Run arbitrary commands. The web UI is on the network, so the set of things it
can ask for is a fixed table in this file - `navigate`, `explore`, `clean`,
`manual`, `stop` - and the only free-form value that ever reaches a
filesystem path is a map name, which is validated by `map_library.valid_name`
before it is used and passed as an environment variable rather than
interpolated into a command line. There is no route from a WebSocket message
to a shell.

CONTRACT

    set_robot_mode  std_msgs/String  {"mode": "clean", "map": "kitchen", "new": false}
                                     (a bare mode name is accepted too)
    robot_mode      std_msgs/String  {"mode", "map", "source", "detail",
                    (latched)         "error", "running", "since", "modes"}
"""

import json
import os
import signal
import subprocess
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from nexva_explore import map_library

# Latched, so an interface that connects later still learns the current mode
# instead of showing "unknown" until the next change.
STATE_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

IDLE = 'idle'

# THE ALLOWED ACTIONS. Nothing outside this table can be started, whoever asks.
#
# `map` says what the mode needs: 'new' names a map that is about to be made,
# 'existing' picks one already saved, 'either' accepts both, None takes none.
# The script paths are relative to the workspace root and are looked up by
# `source`; only `hardware` has scripts in this workspace.
MODES = {
    'navigate': {
        'map': 'existing',
        'hardware': 'launch/realbot/robot_navigate.sh',
        'label': 'navigating on a saved map',
    },
    'explore': {
        'map': 'new',
        'hardware': 'launch/realbot/robot_explore.sh',
        'label': 'exploring and building a map',
    },
    'clean': {
        'map': 'existing',
        'hardware': 'launch/realbot/robot_clean_saved.sh',
        'label': 'cleaning a saved map',
    },
    'manual': {
        'map': 'either',
        'hardware': 'launch/realbot/robot_manual.sh',
        'label': 'driving by hand',
    },
}

# A mission that exits sooner than this did not start; it failed. Long enough
# for Nav2 to still be coming up, short enough that the operator finds out
# from the web UI rather than from the robot sitting still.
STARTUP_GRACE = 25.0

# The zero-velocity burst sent after a mission is stopped. Several messages,
# not one: the first can be lost while the last cmd_vel subscriber is still
# re-matching after the mission's nodes went away.
STOP_BURST = 8
STOP_BURST_GAP = 0.05

# Files whose presence marks the workspace root, for finding the mission
# scripts when nothing says where the workspace is.
WORKSPACE_MARKERS = ('build.sh', 'src')


def find_workspace():
    """
    Locate the workspace root.

    NEXVA_WS wins. Failing that, walk up from this file: with
    --symlink-install the installed module IS the source file, so the share
    directory is no guide, but both the source tree and the install tree sit
    directly under the workspace, so walking up finds it either way.
    """
    explicit = os.environ.get('NEXVA_WS')

    if explicit:
        return explicit

    for start in (os.path.abspath(__file__), os.path.realpath(__file__)):
        here = os.path.dirname(start)

        while True:
            if all(os.path.exists(os.path.join(here, marker))
                   for marker in WORKSPACE_MARKERS):
                return here

            parent = os.path.dirname(here)

            if parent == here:
                break

            here = parent

    return os.getcwd()


class ModeManager(Node):
    """Starts, stops and reports the robot's current mission."""

    def __init__(self):
        super().__init__('mode_manager')

        # 'hardware' on the robot. Selects the script table and the map
        # directory; nothing else in this node differs.
        self.declare_parameter('source', 'hardware')

        # The workspace root, so the scripts can be found from wherever this
        # was launched. Empty means work it out.
        self.declare_parameter('workspace', '')

        # How long to wait for a mission to die before insisting.
        self.declare_parameter('stop_timeout', 12.0)

        # A mode to request once, shortly after startup. robotnav.sh passes
        # `navigate` so "bringup, then Nav2" keeps working exactly as before,
        # with the web UI free to switch modes afterwards. Empty starts idle.
        self.declare_parameter('initial_mode', '')

        # The map for that first request: a saved map's name, or - because
        # this is a local launch parameter and not something off the network -
        # an absolute path to a map .yaml, which is used as given.
        self.declare_parameter('initial_map', '')

        self.source = self.get_parameter('source').value
        self.workspace = (self.get_parameter('workspace').value
                          or find_workspace())
        self.stop_timeout = self.get_parameter('stop_timeout').value
        self.initial_mode = (self.get_parameter('initial_mode').value
                             or '').strip()
        self.initial_map = (self.get_parameter('initial_map').value
                            or '').strip()

        self.mode = IDLE
        self.map_name = ''
        self.since = time.time()
        self.detail = 'nothing running'
        self.process = None
        self.started_at = 0.0
        self.last_error = ''

        self.state_pub = self.create_publisher(
            String, 'robot_mode', STATE_QOS)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            String, 'set_robot_mode', self.on_request, 10)

        # Noticing a mission that died on its own - a launch failure, or the
        # operator pressing Ctrl-C in the mission's own terminal.
        self.create_timer(1.0, self.watch)

        self.publish_state()

        self.get_logger().info(
            f'Mode manager ready for {self.source} in {self.workspace}. '
            f'Modes: {", ".join(sorted(MODES))}, stop. '
            f'Maps in {self.maps_dir()}')

        self.initial_timer = None

        if self.initial_mode:
            # A short delay so the latched state publisher and the bringup
            # check in the script both have something to talk to.
            self.initial_timer = self.create_timer(2.0, self.start_initial)

    def maps_dir(self):
        try:
            return map_library.source_dir(self.source)
        except AttributeError:
            return os.path.join(os.path.expanduser('~'), 'nexva_maps',
                                self.source)

    # ------------------------------------------------------------------
    # Requests
    # ------------------------------------------------------------------

    def on_request(self, msg):
        """
        Handle one request from the web UI or a terminal.

        The payload is JSON: {"mode": "clean", "map": "kitchen"}. A bare mode
        name is accepted too, so `ros2 topic pub ... "{data: stop}"` works
        from anywhere without quoting a JSON document.
        """
        request = self.parse(msg.data)

        if request is None:
            self.refuse(f'could not read the request: {msg.data!r}')
            return

        self.handle_request(request)

    def start_initial(self):
        """The one request robotnav.sh makes on the operator's behalf."""
        if self.initial_timer is not None:
            self.initial_timer.cancel()
            self.initial_timer = None

        mode = self.initial_mode
        chosen = self.initial_map

        self.get_logger().info(
            f'initial mode: {mode}' + (f' on "{chosen}"' if chosen else ''))

        if mode in MODES and os.sep in chosen:
            # A path, from the launch parameter only. Handed straight to the
            # mission; nothing from the network can reach this branch.
            if not os.path.isfile(chosen):
                self.refuse(f'initial map {chosen} does not exist')
                return

            name = os.path.splitext(os.path.basename(chosen))[0]
            self.start_mission(mode, name, map_path=chosen)
            return

        self.handle_request({'mode': mode, 'map': chosen})

    # NOT `handle`: rclpy.Node exposes a `handle` property that
    # Node.__init__ uses as a context manager (`with self.handle:`),
    # so a method of that name shadows it and the node cannot be
    # constructed at all - TypeError before __init__ even returns.
    def handle_request(self, request):
        mode = request.get('mode', '')
        name = request.get('map', '')

        if mode in ('stop', IDLE):
            self.stop_mission('asked to stop')
            return

        if mode not in MODES:
            self.refuse(
                f'"{mode}" is not a mode. One of: '
                f'{", ".join(sorted(MODES))}, stop')
            return

        name, problem = self.check_map(mode, name, request.get('new', False))

        if problem:
            self.refuse(problem)
            return

        if self.mode == mode and self.map_name == name and self.alive():
            # Already doing exactly this. Answer, do not relaunch: restarting
            # a healthy mission because a button was pressed twice is its own
            # failure.
            self.detail = f'already {MODES[mode]["label"]}'
            self.publish_state()
            return

        self.start_mission(mode, name)

    @staticmethod
    def parse(payload):
        """Read a request, as JSON or as a bare mode name."""
        text = (payload or '').strip()

        if not text:
            return None

        if text.startswith('{'):
            try:
                request = json.loads(text)
            except ValueError:
                return None

            return request if isinstance(request, dict) else None

        return {'mode': text}

    def check_map(self, mode, name, wants_new):
        """
        Validate the map a mode was asked to use.

        Returns `(name, problem)`. This is the only place a value from the
        network becomes part of a path, so it is also the only place that has
        to be careful about one.
        """
        needs = MODES[mode]['map']

        if needs is None:
            return '', None

        cleaned, problem = map_library.valid_name(name)

        if problem:
            if needs == 'either' and not (name or '').strip():
                # Manual driving with no map at all is a legitimate choice.
                return '', None

            return '', f'map name: {problem}'

        if needs == 'existing' or (needs == 'either' and not wants_new):
            found = map_library.find(cleaned, self.source)

            if found is None:
                available = [item['name']
                             for item in map_library.list_maps(self.source)]

                return '', (
                    f'no saved {self.source} map called "{cleaned}". '
                    f'Available: {", ".join(available) if available else "none"}')

            return cleaned, None

        # A new map. Refuse to overwrite one that exists unless told to.
        if map_library.exists(cleaned, self.source) and not wants_new:
            return '', (
                f'a map called "{cleaned}" already exists. Choose another '
                f'name, or confirm replacing it')

        return cleaned, None

    # ------------------------------------------------------------------
    # Running missions
    # ------------------------------------------------------------------

    def start_mission(self, mode, name, map_path=None):
        """Stop whatever is running, then start the requested mission."""
        if self.alive():
            self.stop_mission(f'switching to {mode}', announce=False)

        relative = MODES[mode].get(self.source)

        if not relative:
            self.refuse(f'{mode} has no script for source "{self.source}"')
            return

        script = os.path.join(self.workspace, relative)

        if not os.path.isfile(script):
            self.refuse(f'{script} is missing - cannot start {mode}')
            return

        # The map name reaches the mission as an ENVIRONMENT VARIABLE, never
        # as part of a command line. There is no shell in this call, so there
        # is nothing for a crafted name to escape into - and it has been
        # through `valid_name` besides.
        environment = dict(os.environ)
        environment['NEXVA_WS'] = self.workspace
        environment['MAP_SOURCE'] = self.source

        if name:
            environment['MAP_NAME'] = name

        if map_path:
            environment['MAP'] = map_path
        elif name and MODES[mode]['map'] == 'existing':
            # Cleaning and navigating take a PATH, not a name - the scripts
            # read `MAP` and only fall back to resolving MAP_NAME themselves
            # when it is empty. Handing them the resolved path is what makes
            # "clean THIS map" mean this map, rather than the last one saved.
            chosen = map_library.find(name, self.source)

            if chosen is not None:
                environment['MAP'] = chosen['yaml']

        try:
            self.process = subprocess.Popen(
                # Through bash, not exec'd directly: the scripts reach the Pi
                # by rsync/copy, which can drop the executable bit, and then
                # every mode switch failed with "[Errno 13] Permission
                # denied". bash only needs to READ the file.
                ['bash', script],
                cwd=self.workspace,
                env=environment,
                # Its own process group, so stopping the mission takes the
                # whole tree - a mission script starts ros2 launch, which
                # starts a dozen nodes, and killing only the script leaves
                # every one of them holding its topics.
                start_new_session=True,
            )
        except OSError as exc:
            self.refuse(f'could not start {mode}: {exc}')
            return

        self.mode = mode
        self.map_name = name
        self.since = time.time()
        self.started_at = time.time()
        self.last_error = ''
        self.detail = MODES[mode]['label'] + (f' "{name}"' if name else '')

        self.get_logger().info(
            f'{mode}: {self.detail} (pid {self.process.pid})')
        self.publish_state()

    def stop_mission(self, why, announce=True):
        """
        Stop the running mission, wait for it to actually be gone, and then
        stop the wheels.

        Waiting matters: a half-dead SLAM still owns `map -> odom` and a
        half-dead cleaner still publishes `cmd_vel`, so starting the next
        mode on top of one is how two missions end up fighting.
        """
        process = self.process
        self.process = None

        if process is not None and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except OSError:
                pass

            deadline = time.time() + self.stop_timeout

            while process.poll() is None and time.time() < deadline:
                time.sleep(0.2)

            if process.poll() is None:
                self.get_logger().warn(
                    'the mission did not stop when asked - insisting')

                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except OSError:
                    pass

                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass

        # The wheels, regardless of what the mission did on the way out. The
        # ESP32 has its own 500 ms watchdog, but a mission killed mid-command
        # should not get to lean on it.
        self.halt_wheels()

        self.mode = IDLE
        self.map_name = ''
        self.since = time.time()
        self.detail = why

        if announce:
            self.get_logger().info(f'stopped: {why}')
            self.publish_state()

    def halt_wheels(self):
        """Publish a burst of zero velocity on cmd_vel."""
        stop = Twist()

        try:
            for _ in range(STOP_BURST):
                self.cmd_pub.publish(stop)
                time.sleep(STOP_BURST_GAP)
        except Exception as exc:                            # noqa: BLE001
            # Only reachable when the ROS context is already gone (shutdown
            # racing a stop). The mission is dead by now either way.
            self.get_logger().warn(f'could not publish the stop burst: {exc}')

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def watch(self):
        """Notice a mission that ended without being asked to."""
        if self.mode == IDLE or self.process is None:
            return

        code = self.process.poll()

        if code is None:
            return

        running_for = time.time() - self.started_at
        self.process = None

        if running_for < STARTUP_GRACE and code != 0:
            self.last_error = (
                f'{self.mode} failed to start (exit {code} after '
                f'{running_for:.0f}s) - check the mission output')
            self.get_logger().error(self.last_error)
            self.detail = self.last_error
        else:
            self.detail = f'{self.mode} finished (exit {code})'
            self.get_logger().info(self.detail)

        # It went away on its own, so nothing is publishing cmd_vel now - but
        # whatever it last published may still be standing.
        self.halt_wheels()

        self.mode = IDLE
        self.map_name = ''
        self.since = time.time()
        self.publish_state()

    # ------------------------------------------------------------------

    def refuse(self, why):
        """Say no, and say why, without disturbing what is already running."""
        self.last_error = why
        self.get_logger().warn(why)
        self.publish_state()

    def state(self):
        """Everything an interface needs to draw itself."""
        return {
            'mode': self.mode,
            'map': self.map_name,
            'source': self.source,
            'detail': self.detail,
            'error': self.last_error,
            'running': self.alive(),
            'since': self.since,
            'modes': sorted(MODES),
        }

    def publish_state(self):
        message = String()
        message.data = json.dumps(self.state())

        try:
            self.state_pub.publish(message)
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().warn(f'could not publish robot_mode: {exc}')

    def destroy_node(self):
        # Never leave a mission running behind the manager: an orphan mission
        # is one nothing can stop any more.
        self.stop_mission('manager shutting down', announce=False)
        super().destroy_node()


def main(args=None):
    # Signal handling is taken over from rclpy on purpose. rclpy's own handler
    # shuts the context down BEFORE spin() returns, so by the time
    # destroy_node() runs there is no valid context left and the stop burst
    # fails with "publisher's context is invalid" - the one moment the wheels
    # most need it. Here the flag only breaks the loop, and the mission is
    # stopped below while ROS is still fully up.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ModeManager()

    stopping = threading.Event()

    def request_stop(signum, frame):
        stopping.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Context is still valid here, so the kill + zero Twist burst in
        # stop_mission actually reaches the base.
        try:
            node.stop_mission('manager shutting down', announce=False)
        except Exception as exc:                                # noqa: BLE001
            print('mode_manager: stop on shutdown failed: %s' % exc)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
