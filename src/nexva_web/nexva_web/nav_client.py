"""ROS side of the waypoint bridge: localization, goal dispatch, stop.

Everything the web layer needs from ROS lives here, so the bridge never touches
rclpy directly. The node is spun by a MultiThreadedExecutor on its own thread;
public methods below are called from the asyncio thread and are safe to call
there because they only publish, start actions, or read state under a lock.

State changes are pushed out through `on_event`, a plain callable the bridge
installs. It is invoked from ROS callback threads, so the bridge is responsible
for hopping back onto its event loop.
"""

import collections
import json
import math
import os
import threading
import time

import numpy as np
import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    Point32, PolygonStamped, PoseStamped, PoseWithCovarianceStamped, Twist,
    Vector3,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import (
    Bool as BoolMsg, Float32 as Float32Msg, String as StringMsg,
)
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from rcl_interfaces.msg import Log as RosoutLog
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from . import mapcheck
from . import waypoints as wp_mod
from .waypoints import WaypointError

# Above this the particle cloud is too spread out to trust for navigation.
LOCALIZED_COV_MAX = 0.5
# map -> base_footprint older than this is a dead publisher, not a pose.
# tf2 hands back the latest transform it ever saw for Time(), however old.
TF_FRESH_S = 2.0

# The zone backend (zone_coverage) listens here; see send_zone.
ZONE_TOPIC = '/zone_coverage/zone'
ZONE_CMD_TOPIC = '/zone_coverage/command'

# Teleop. The publish rate matters: driving /cmd_vel much above this reboots
# the ESP32 (the micro-ROS link cannot keep up and the board resets), so it is
# capped here rather than left to however fast the browser sends events.
TELEOP_HZ = 20.0
# If the browser stops sending - released, tab closed, wifi dropped - the robot
# must stop by itself well before the firmware's own 500 ms timeout.
TELEOP_DEADMAN = 0.4
# Kept under the vx_max/wz_max in nav2_params.yaml.
TELEOP_VX_MAX = 0.26
TELEOP_WZ_MAX = 1.5

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}

# Motor limits, published to the ESP32 on /pid_limits. These bounds mirror the
# ones in microros_code.ino exactly - the firmware clamps whatever it is sent,
# so anything outside these would silently come back as something else and the
# page would show a number the robot is not using.
PWM_ABS_MAX = 255
PWM_ABS_MIN = 0
MAX_PWM_LOWER_BOUND = 40
SPEED_ABS_MAX = 0.50
SPEED_ABS_MIN = 0.01

# The firmware's power-on values. The ESP32 reverts to these on every reboot,
# which is why the limits are republished periodically rather than once.
PID_LIMIT_DEFAULTS = {'min_pwm': 150, 'max_pwm': 255, 'max_speed': 0.30}

# Slow enough to be free, fast enough that a reboot mid-run does not leave the
# base on factory limits for long.
PID_LIMITS_REPUBLISH_S = 5.0

# Autonomous missions (package nexva_explore). mode_manager supervises the
# mission processes and answers on /robot_mode; frontier_explorer only exists
# while an explore or clean mission is running.
ROBOT_MODES = ('navigate', 'explore', 'clean', 'manual', 'stop')
EXPLORER_COMMANDS = ('explore', 'clean', 'pause', 'resume', 'stop')
# Explorer states in which its process is up and holds the base.
EXPLORER_ALIVE = ('explore', 'clean', 'recovering', 'paused')
# ...and the subset in which it is actually publishing velocities.
EXPLORER_DRIVING = ('explore', 'clean', 'recovering')
# status_json comes at 1 Hz. Longer than this without one, and with its
# publisher gone, the explorer process has exited - possibly without ever
# publishing 'stopped', since mode_manager kills it with a signal.
EXPLORER_STALE_S = 5.0
# A browser cannot usefully draw more of the planned route than this.
EXPLORE_PATH_MAX_POINTS = 120

# --- /rosout mirror -------------------------------------------------------
# How much of the ROS log is kept in memory for the diagnostics page. A page
# opened after something went wrong must still be able to see it: a log that
# starts at "whatever arrives next" is useless for the only job it has. A few
# hundred lines of (level, node, message) is tens of kB and bounded.
ROSOUT_RING = 400
# DEBUG is dropped. rcl only publishes what a node has enabled, but one node
# left on debug publishes thousands of lines a minute, which would flush the
# ring and flood every open WebSocket before anyone could read it. INFO and
# above is what a diagnostics page is for.
ROSOUT_MIN_LEVEL = 20                       # rcl_interfaces/msg/Log.INFO
ROSOUT_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR',
                 50: 'FATAL'}
# A single log line nobody wants wrapped across a whole screen.
ROSOUT_MSG_MAX = 1000

# --- how long a button press may wait before the person gets an answer ------
# Every public method below runs on a worker thread, never on the event loop,
# but somebody is still staring at a button that "did nothing". Half a second
# is worth spending on a discovery race (a node that started a moment ago);
# five is not worth spending on a stack that simply is not running.
DISCOVERY_WAIT_S = 0.4

# A queue longer than this is a typo or a runaway script, not a route.
MAX_QUEUE_POINTS = 50

# --- IMU readout (nexva_sensor) ----------------------------------------------
# The driver runs at tens of Hz; a page needs two. Nothing newer than this is
# trusted: past it the card says there is no data rather than showing numbers
# that stopped moving.
IMU_FRESH_S = 2.0
IMU_EMIT_S = 0.5

# Live speed for the page: measured (/odom) next to commanded (/cmd_vel, the
# final command in every mode - Nav2's collision monitor output, the explorer,
# or teleop). Sent at SPEED_EMIT_HZ, not per message; /odom runs at 20 Hz.
SPEED_EMIT_HZ = 4.0
# Older than this and the numbers are stale: say so instead of showing them.
ODOM_FRESH_S = 1.0
# A zero command is assumed after this long without one, because nothing
# republishes a stop: the last /cmd_vel stays "current" forever otherwise.
CMD_FRESH_S = 0.6
# The collision monitor publishes only when its action CHANGES, so the last
# message is the current state. Names are CollisionMonitorState's constants.
GUARD_ACTIONS = {0: '', 1: 'stop', 2: 'slowdown', 3: 'approach', 4: 'limit'}


def yaw_from_quat(z, w):
    return math.degrees(2.0 * math.atan2(z, w))


def rpy_from_quat(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) in degrees."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _finite(v):
    """A float, or None when it is NaN/inf. JSON has no NaN: one in a message
    makes the browser's JSON.parse throw and costs the whole frame."""
    v = float(v)
    return v if math.isfinite(v) else None


def _pose_msg(frame_id, stamp, x, y, yaw):
    """PoseStamped at (x, y) facing `yaw` radians in `frame_id`."""
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = stamp
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps


def latched_qos():
    """Transient-local, keep-last-1: what map_server, slam_toolbox and the
    explorer's state topics use, so a late subscriber still gets the last."""
    return QoSProfile(depth=1,
                      history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def encode_occupancy_rle(data):
    """OccupancyGrid cells -> the page's 3-symbol run-length string.

    Each cell becomes u (unknown, <0), o (occupied, >=65) or f (free), and
    equal neighbours collapse to '<count><char>'. Same encoding as the sim's
    dashboard; a room-sized map packs to a few kB, and no image library is
    needed on the Pi. numpy does the classification and the run boundaries so
    a 4-million-cell map does not mean a 4-million-iteration Python loop.
    """
    arr = np.asarray(data, dtype=np.int16).ravel()
    if arr.size == 0:
        return ''
    cls = np.full(arr.shape, ord('f'), dtype=np.uint8)
    cls[arr < 0] = ord('u')
    cls[arr >= 65] = ord('o')
    change = np.flatnonzero(cls[1:] != cls[:-1]) + 1
    starts = np.concatenate(([0], change))
    ends = np.concatenate((change, [cls.size]))
    lengths = (ends - starts).tolist()
    chars = cls[starts].tobytes().decode('ascii')
    return ''.join('%d%s' % (n, ch) for n, ch in zip(lengths, chars))


def reduce_marker(marker):
    """One /coverage_blocks CUBE -> {x, y, size, s[, f]}.

    The state is read from the colour the explorer painted, never recomputed,
    so the page cannot disagree with the robot about what is cleaned:
    green = cleaned (g), blue = being worked (b), grey = given up (x), and the
    red ramp r=0.9 g=0.15+0.6*fraction b=0.1 = still to do (r), where the
    fraction says how much of the square has been swept so far.
    """
    c = marker.color
    if c.g > 0.6 and c.r < 0.3:
        s, f = 'g', None
    elif c.b > 0.6:
        s, f = 'b', None
    elif c.r > 0.4 and c.g > 0.4 and c.b > 0.4:
        s, f = 'x', None
    else:
        s = 'r'
        f = max(0.0, min(1.0, (float(c.g) - 0.15) / 0.6))
    d = {
        'x': round(float(marker.pose.position.x), 3),
        'y': round(float(marker.pose.position.y), 3),
        'size': round(float(marker.scale.x), 3),
        's': s,
    }
    if f is not None:
        d['f'] = round(f, 2)
    return d


class NavClient(Node):

    def __init__(self, waypoint_set):
        super().__init__('nexva_web_bridge')
        self.waypoints = waypoint_set
        self.on_event = None

        self._lock = threading.Lock()
        self._pose = None            # (x, y, yaw_deg)
        self._cov_xx = float('inf')
        self._cov_yy = float('inf')
        self._goal_handle = None
        self._active = None          # name of the destination, or None
        self._last_result = None
        self._map_ok = None          # None = not checked yet
        # map_server's yaml path as of the last good probe, and the parsed
        # grid for it. set_pose_at/goto_pose check against THESE instead of
        # re-asking map_server: during an explore mission there is no
        # map_server and every re-ask was a 5 s stall before any feedback.
        self._map_path = None
        self._grid = None            # (path, mtime, OccupancyMap)
        self._teleop = None          # (vx, wz, expiry) or None

        cb = ReentrantCallbackGroup()

        # AMCL's liveness signal is the map -> odom broadcast, not /amcl_pose.
        # A stationary robot never moves far enough to trigger a filter update
        # (update_min_d), so /amcl_pose simply stops arriving - treating that
        # as "lost localization" wrongly blocks every goal.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb,
            QoSProfile(depth=10,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.VOLATILE),
            callback_group=cb)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.nav_to_pose = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self.follow_waypoints = ActionClient(
            self, FollowWaypoints, 'follow_waypoints', callback_group=cb)

        self.map_params = self.create_client(
            GetParameters, '/map_server/get_parameters', callback_group=cb)

        # --- zone cleaning ---------------------------------------------
        # The rectangle drawn on the page goes out as a polygon in MAP metres;
        # zone_coverage plans it and drives it through this same Nav2 stack.
        self.zone_pub = self.create_publisher(
            PolygonStamped, '/zone_coverage/zone', 10)
        self.zone_cmd_pub = self.create_publisher(
            StringMsg, '/zone_coverage/command', 10)
        self.zone_state = 'unknown'
        self.zone_progress = 0.0
        # The planned sweep, kept so the page can draw it. Latched QoS: the
        # plan is published once when planning finishes, so a volatile
        # subscriber that connects afterwards would never see it.
        self.zone_plan = []
        self.create_subscription(
            NavPath, '/zone_coverage/plan', self._zone_plan,
            QoSProfile(depth=1,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=cb)
        self.create_subscription(
            StringMsg, '/zone_coverage/state',
            lambda m: self._zone_state(m.data), 10, callback_group=cb)
        self.create_subscription(
            Float32Msg, '/zone_coverage/progress',
            lambda m: self._zone_progress(m.data), 10, callback_group=cb)

        # --- motor limits (ESP32 PID) ----------------------------------
        # /pid_limits is a plain volatile topic into micro-ROS, and the ESP32
        # resets to its compiled defaults whenever it reboots - which the
        # bringup does deliberately on every start. Publishing once would mean
        # the page showing limits the base silently is not using, so the
        # current set is repeated on a timer instead.
        self.pid_limits_pub = self.create_publisher(
            Vector3, '/pid_limits', 10)
        self.pid_limits = dict(PID_LIMIT_DEFAULTS)
        self._pid_limits_sent = False
        self.create_timer(PID_LIMITS_REPUBLISH_S, self._pid_limits_tick,
                          callback_group=cb)

        self.create_timer(1.0 / TELEOP_HZ, self._teleop_tick,
                          callback_group=cb)

        # --- autonomous missions (nexva_explore) --------------------------
        # mode_manager takes a JSON request on /set_robot_mode and reports on
        # latched /robot_mode. frontier_explorer, alive only during explore
        # and clean, takes pause/resume/stop and reports its state, a 1 Hz
        # status dict, the coverage squares and its planned route.
        self.set_mode_pub = self.create_publisher(
            StringMsg, '/set_robot_mode', 10)
        self.explorer_cmd_pub = self.create_publisher(
            StringMsg, '/frontier_explorer/command', 10)
        # Save map: the page sends a name, map_saver (nexva_explore) writes
        # map + pose + pose graph and answers with JSON on /save_map_result.
        # Volatile topics; the result is broadcast as a 'save_result' event.
        self.save_map_pub = self.create_publisher(StringMsg, '/save_map', 10)
        self.create_subscription(
            StringMsg, '/save_map_result', self._save_result_cb, 10,
            callback_group=cb)
        self.robot_mode = {}             # last /robot_mode, parsed
        self.explorer_mode = None        # explore|clean|paused|... or None
        self.explorer_status = {}        # last status_json, parsed
        self._explorer_seen_at = 0.0     # time of the last status_json
        self._explorer_pause_sent = 0.0  # teleop takeover rate limit
        self.coverage_blocks = []        # [{x, y, size, s[, f]}, ...]
        self.explore_path = []           # [(x, y), ...] thinned
        self.create_subscription(
            StringMsg, '/robot_mode', self._robot_mode_cb, latched_qos(),
            callback_group=cb)
        self.create_subscription(
            StringMsg, '/frontier_explorer/mode', self._explorer_mode_cb,
            latched_qos(), callback_group=cb)
        self.create_subscription(
            StringMsg, '/frontier_explorer/status_json',
            self._explorer_status_cb, 10, callback_group=cb)
        self.create_subscription(
            MarkerArray, '/coverage_blocks', self._coverage_cb,
            latched_qos(), callback_group=cb)
        self.create_subscription(
            NavPath, '/explore_path', self._explore_path_cb, 10,
            callback_group=cb)

        # The map itself. Whoever is publishing it - map_server while
        # navigating, slam_toolbox while exploring - it is latched, and the
        # latest one is kept so /api/live_map can serve it while it grows.
        # Encoding is done on request and cached per message, not per
        # callback: slam_toolbox republishes far more often than a page asks.
        self._map_msg = None
        self._map_seq = 0
        self._map_encoded = None
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, latched_qos(),
            callback_group=cb)

        # --- the ROS log, mirrored for the diagnostics page ---------------
        # /rosout carries every node's log output. rcl publishes it RELIABLE,
        # TRANSIENT_LOCAL with a 10 s lifespan, so a transient-local
        # subscriber is handed the last few seconds from each node that is
        # still alive - which is why the page has something to show the
        # instant it opens. Depth is kept small on purpose: a depth of 1000
        # (what rcl offers) times every node in the graph would arrive as one
        # burst of tens of thousands of messages at startup.
        self._log_ring = collections.deque(maxlen=ROSOUT_RING)
        self._log_seq = 0
        self.create_subscription(
            RosoutLog, '/rosout', self._rosout_cb,
            QoSProfile(depth=25,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=cb)

        # --- IMU + stall guard (nexva_sensor), for the diagnostics page ----
        # The driver publishes /imu BEST_EFFORT depth 1, so that is what we
        # subscribe with. Only the newest message is kept (a reference, no
        # copy), and it is turned into an event at IMU_EMIT_S, not per sample.
        # stall_guard's flag and status line are latched.
        self._imu = None
        self._imu_at = 0.0               # time.monotonic() of the newest one
        self._imu_emit_at = 0.0
        self._imu_count = 0
        self._imu_stale_sent = False
        self._imu_calib = ''
        self._stalled = None             # latched Bool, None = never heard
        self._stall_status = ''
        self.create_subscription(
            Imu, '/imu', self._imu_cb,
            QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       durability=QoSDurabilityPolicy.VOLATILE),
            callback_group=cb)
        self.create_subscription(
            StringMsg, '/imu/status', self._imu_calib_cb, 10,
            callback_group=cb)
        self.create_subscription(
            BoolMsg, '/motion_check/moved', self._stalled_cb,
            latched_qos(), callback_group=cb)
        self.create_subscription(
            StringMsg, '/motion_check/status', self._stall_status_cb,
            latched_qos(), callback_group=cb)

        # --- live speed + obstacle guard, for the Status card -------------
        # Depth 1, best effort: only the newest sample matters, and best
        # effort matches any publisher. Callbacks just store; the timer emits.
        self._odom = None                # (v, w, monotonic time)
        self._cmd = None                 # (v, w, monotonic time)
        self._guard = ''                 # '' | stop | slowdown | approach | limit
        self._guard_polygon = ''
        newest = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, '/odom', self._odom_cb, newest,
                                 callback_group=cb)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, newest,
                                 callback_group=cb)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state', self._guard_cb,
            10, callback_group=cb)
        self.create_timer(1.0 / SPEED_EMIT_HZ, self._speed_tick,
                          callback_group=cb)

        # Several things the UI cares about change without any event to hang a
        # broadcast on: action servers finishing discovery, AMCL converging,
        # Nav2 being restarted underneath us. Poll and push when it differs.
        self._last_state_sig = None
        self.create_timer(1.0, self._state_tick, callback_group=cb)

        self.get_logger().info(
            'waypoint bridge up: %d waypoints for map %r'
            % (len(self.waypoints.waypoints), self.waypoints.map_name))

    # ---------------------------------------------------------------- events

    def _emit(self, kind, **payload):
        cb = self.on_event
        if cb is not None:
            payload['type'] = kind
            try:
                cb(payload)
            except Exception as exc:                      # never kill a callback
                self.get_logger().warn('event handler raised: %s' % exc)

    # ------------------------------------------------------------- speed

    def _odom_cb(self, msg):
        t = msg.twist.twist
        self._odom = (t.linear.x, t.angular.z, time.monotonic())

    def _cmd_cb(self, msg):
        self._cmd = (msg.linear.x, msg.angular.z, time.monotonic())

    def _guard_cb(self, msg):
        self._guard = GUARD_ACTIONS.get(msg.action_type, '')
        self._guard_polygon = msg.polygon_name

    def speed_snapshot(self):
        now = time.monotonic()
        odom, cmd = self._odom, self._cmd
        fresh = odom is not None and now - odom[2] <= ODOM_FRESH_S
        cmd_live = cmd is not None and now - cmd[2] <= CMD_FRESH_S
        return {
            'fresh': fresh,
            'v': float(odom[0]) if fresh else None,
            'w': float(odom[1]) if fresh else None,
            'cmd_v': float(cmd[0]) if cmd_live else 0.0,
            'cmd_w': float(cmd[1]) if cmd_live else 0.0,
            # The cap the ESP32 enforces, so the bar is drawn against it.
            'max_v': float(self.pid_limits['max_speed']),
            # Only meaningful while Nav2 runs; the explorer bypasses it.
            'guard': self._guard if self.count_publishers(
                '/collision_monitor_state') else '',
            'guard_polygon': self._guard_polygon,
        }

    def _speed_tick(self):
        self._emit('speed', **self.speed_snapshot())

    # ------------------------------------------------------------ ROS log

    def _rosout_cb(self, msg):
        """One /rosout entry -> the ring, and out to every open page.

        This callback must never log. Our own log lines come back here through
        /rosout, so a warn raised while handling one would produce another
        message, which would be handled here, and so on: a feedback loop that
        only ends when the handler stops failing. `_emit` logs on failure,
        which is right everywhere else and wrong here, so the event callback
        is invoked directly and a failure is swallowed in silence.
        """
        level = int(msg.level)
        if level < ROSOUT_MIN_LEVEL:
            return
        text = msg.msg or ''
        if len(text) > ROSOUT_MSG_MAX:
            text = text[:ROSOUT_MSG_MAX] + ' …[truncated]'
        with self._lock:
            self._log_seq += 1
            entry = {
                'seq': self._log_seq,
                # Wall clock, not the ROS clock: this lines up with the
                # timestamps in journalctl and in the browser.
                't': msg.stamp.sec + msg.stamp.nanosec / 1e9,
                'level': level,
                'level_name': ROSOUT_LEVELS.get(level, str(level)),
                'name': msg.name,
                'msg': text,
            }
            self._log_ring.append(entry)

        cb = self.on_event
        if cb is None:
            return
        try:
            cb(dict(entry, type='log'))
        except Exception:                     # noqa: BLE001 - see docstring
            pass

    def recent_logs(self, limit=ROSOUT_RING, since=0):
        """The tail of the ring, oldest first, for a page that just connected.

        `since` lets a page that briefly lost its WebSocket ask only for what
        it missed, rather than redrawing hundreds of lines it already has.
        """
        with self._lock:
            items = [e for e in self._log_ring if e['seq'] > int(since or 0)]
        return items[-int(limit):] if limit else items

    def _amcl_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self._lock:
            self._pose = (p.x, p.y, yaw_from_quat(q.z, q.w))
            # rclpy hands back float64[36] as a numpy array, so these are
            # numpy.float64. Comparing them later yields numpy.bool_, which
            # json.dumps refuses - that killed /api/state AND dropped every
            # WebSocket on connect, because the first thing a client is sent
            # is this state dict. Coerce at the boundary.
            self._cov_xx = float(msg.pose.covariance[0])
            self._cov_yy = float(msg.pose.covariance[7])
        self._emit('pose', x=p.x, y=p.y, yaw=yaw_from_quat(q.z, q.w))

    # ------------------------------------------------------------------ IMU

    def _imu_cb(self, msg):
        """Newest IMU sample. Runs at the driver's rate, so it only keeps a
        reference and decides whether it is time for an event."""
        now = time.monotonic()
        self._imu = msg
        self._imu_at = now
        self._imu_count += 1
        if now - self._imu_emit_at >= IMU_EMIT_S:
            self._imu_emit_at = now
            self._imu_stale_sent = False
            self._emit('imu', **self.imu_snapshot())

    def _imu_calib_cb(self, msg):
        self._imu_calib = str(msg.data)[:200]

    def _stalled_cb(self, msg):
        self._stalled = bool(msg.data)
        self._emit('imu', **self.imu_snapshot())

    def _stall_status_cb(self, msg):
        self._stall_status = str(msg.data)[:300]
        self._emit('imu', **self.imu_snapshot())

    def imu_snapshot(self):
        """What the diagnostics IMU card shows, as one JSON-safe dict.

        Numbers appear only while the newest sample is fresh, and orientation
        only when the driver says it is real (orientation_covariance[0] >= 0;
        it is -1 in accel-only mode, where the quaternion is a placeholder and
        showing roll/pitch from it would be inventing data).
        """
        msg, at = self._imu, self._imu_at
        age = None if msg is None else max(0.0, time.monotonic() - at)
        fresh = msg is not None and age <= IMU_FRESH_S
        snap = {
            'present': msg is not None,
            'fresh': bool(fresh),
            'age': None if age is None else round(age, 1),
            'count': self._imu_count,
            'calib': self._imu_calib,
            'stall': self._stall_snapshot(),
        }
        if fresh:
            a, w = msg.linear_acceleration, msg.angular_velocity
            valid = float(msg.orientation_covariance[0]) >= 0.0
            snap.update(
                accel=[_finite(a.x), _finite(a.y), _finite(a.z)],
                gyro_z=_finite(w.z),
                orientation_valid=bool(valid))
            if valid:
                q = msg.orientation
                snap['rpy'] = [_finite(v) for v in
                               rpy_from_quat(q.x, q.y, q.z, q.w)]
        return snap

    def _stall_snapshot(self):
        # A latched message outlives its publisher in our subscription, so a
        # stall_guard that has since died would otherwise keep showing its
        # last verdict forever. No publisher = not running.
        if (self.count_publishers('/motion_check/status') == 0
                or (self._stalled is None and not self._stall_status)):
            return {'running': False}
        return {'running': True, 'stalled': self._stalled,
                'status': self._stall_status}

    def imu_event(self):
        return dict(self.imu_snapshot(), type='imu')

    def _tf_pose(self):
        """Robot pose from TF, which updates even when AMCL is quiet."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quat(t.transform.rotation.z, t.transform.rotation.w))

    # ----------------------------------------------------------------- state

    def _tf_fresh(self):
        """True when map -> base_footprint exists AND is recent.

        lookup_transform(Time()) returns the newest transform in the buffer
        even if its publisher died minutes ago, so the stamp is checked. A
        stamp of 0 means a pure static chain (tf2 reports no time for it),
        which is always current.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception:                                   # noqa: BLE001
            return False
        stamp = rclpy.time.Time.from_msg(t.header.stamp)
        if stamp.nanoseconds == 0:
            return True
        age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
        return age <= TF_FRESH_S

    def localization(self):
        """(localized, source) with source in 'amcl' | 'slam' | None.

        AMCL running (something publishes /amcl_pose): the covariance decides,
        as before. No AMCL - explore / manual, where slam_toolbox provides
        map -> odom - a fresh transform is the localization.
        """
        if not self._tf_fresh():
            return False, None
        if self.count_publishers('/amcl_pose') == 0:
            return True, 'slam'
        with self._lock:
            if self._pose is None:
                # AMCL is broadcasting map -> odom, which it only does once it
                # has an estimate, but we have not seen an /amcl_pose yet. That
                # is the normal cold start against an already-localized robot:
                # trust the transform until a real covariance arrives.
                return True, 'amcl'
            # bool() is not redundant: a numpy scalar slipping through here
            # would make this whole dict unserializable again.
            return bool(self._cov_xx < LOCALIZED_COV_MAX
                        and self._cov_yy < LOCALIZED_COV_MAX), 'amcl'

    def localized(self):
        """True when the robot has a live, confident map pose (see above).

        Liveness comes from the map -> base_footprint transform rather than the
        age of the last /amcl_pose: AMCL publishes that topic only when the
        filter updates, so a robot standing still stops publishing it entirely
        while remaining perfectly well localized.
        """
        return self.localization()[0]

    def state(self):
        with self._lock:
            pose, active, result = self._pose, self._active, self._last_result
            teleop = self._teleop is not None
        pose = self._tf_pose() or pose        # TF is live even when AMCL is quiet
        is_localized, loc_source = self.localization()
        return {
            'teleop': teleop,
            'type': 'state',
            'map': self.waypoints.map_name,
            'map_ok': self._map_ok,
            'localized': is_localized,
            'localized_source': loc_source,
            'navigating': active is not None,
            'destination': active,
            'last_result': result,
            'pose': ({'x': pose[0], 'y': pose[1], 'yaw': pose[2]}
                     if pose else None),
            'nav_ready': self.nav_to_pose.server_is_ready(),
            # Whether anything is on the other end of the zone / waypoint
            # paths, so the page can say what is missing instead of acking
            # a message that went nowhere.
            'follow_ready': self.follow_waypoints.server_is_ready(),
            'zone_ready': self.count_subscribers(ZONE_TOPIC) > 0,
            'pid_limits': dict(self.pid_limits),
            'pid_limits_sent': self._pid_limits_sent,
            'robot_mode': dict(self.robot_mode),
            'explorer_mode': self.explorer_mode,
            'explorer_status': dict(self.explorer_status),
            'coverage_summary': self.coverage_summary(),
            'live_map_seq': self._map_seq,
            'log_seq': self._log_seq,
        }

    def _state_tick(self):
        self._explorer_liveness()
        # No sample for IMU_FRESH_S: say so once. Silence produces no event of
        # its own, so without this the page would keep showing the last
        # numbers it was sent.
        if (self._imu is not None and not self._imu_stale_sent
                and time.monotonic() - self._imu_at > IMU_FRESH_S):
            self._imu_stale_sent = True
            self._emit('imu', **self.imu_snapshot())
        s = self.state()
        # Pose is excluded: it changes constantly while driving and has its own
        # 'pose' event, so including it here would broadcast state at 1 Hz
        # forever for no reason.
        sig = (s['localized'], s['navigating'], s['destination'],
               s['nav_ready'], s['follow_ready'], s['zone_ready'],
               s['map_ok'], s['teleop'],
               s['explorer_mode'], s['robot_mode'].get('mode'),
               s['robot_mode'].get('error'))
        if sig != self._last_state_sig:
            self._last_state_sig = sig
            self._emit('state', **s)

    def check_map(self, timeout=5.0, wait=True):
        """Compare the loaded map against the one the waypoints belong to.

        `wait=False` never waits for map_server to appear: if its service is
        not up right now the answer is "not running" at once. HTTP handlers
        and the pose picker use that, because during an explore mission there
        is no map_server at all and waiting for one is a stall for nothing.
        """
        if not wait and not self.map_params.service_is_ready():
            self._map_ok = None
            self._map_path = None
            return None, 'map_server not running'
        if not self.map_params.wait_for_service(timeout_sec=timeout):
            self._map_ok = None
            self._map_path = None
            return None, 'map_server not reachable'
        req = GetParameters.Request()
        req.names = ['yaml_filename']
        future = self.map_params.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done() or not future.result().values:
            self._map_ok = None
            self._map_path = None
            return None, 'map_server did not answer'
        loaded = future.result().values[0].string_value
        ok = self.waypoints.matches_map(loaded)
        self._map_ok = ok
        self._map_path = loaded if ok else None
        if ok:
            return True, loaded
        return False, ('loaded map is %r but waypoints are for %r'
                       % (loaded, self.waypoints.map_name))

    # --------------------------------------------------------- initial pose

    def set_initial_pose(self, name, settle=2.0):
        """Seed AMCL from a named waypoint.

        The caller must have confirmed the robot is physically standing there.
        """
        wp = self.waypoints.get(name)

        # /initialpose is not latched: publishing before AMCL subscribes drops
        # the message on the floor with no error anywhere.
        self._require_amcl('seed AMCL')

        msg = wp.to_initial_pose(self.waypoints.frame_id)
        for _ in range(3):                    # cheap insurance against a drop
            self.initial_pose_pub.publish(msg)
            time.sleep(0.15)

        self.get_logger().info('seeded AMCL at %r' % name)
        deadline = time.time() + settle
        while time.time() < deadline:
            if self.localized():
                break
            time.sleep(0.1)
        self._emit('state', **self.state())
        return self.localized()

    def _require_amcl(self, what='set the 2D pose'):
        """Raise at once, with the reason, if nothing can hear /initialpose.

        AMCL only exists during NAVIGATE and CLEAN. While exploring (or driving
        manually) slam_toolbox owns the pose and /initialpose has no
        subscriber, so a published pose would vanish without a trace - which
        from the page looks exactly like "the button does nothing". Waits only
        DISCOVERY_WAIT_S, for a node that started a moment ago.
        """
        deadline = time.time() + DISCOVERY_WAIT_S
        while self.count_subscribers('/initialpose') == 0:
            if time.time() >= deadline:
                mode = (self.robot_mode or {}).get('mode')
                raise WaypointError(
                    'cannot %s: AMCL is not running%s, so nothing is '
                    'subscribed to /initialpose. This works while a NAVIGATE '
                    'or CLEAN mission is running, not while exploring - '
                    'SLAM owns the pose there.'
                    % (what, ' (mission: %s)' % mode if mode else ''))
            time.sleep(0.05)

    def _grid_for(self, path):
        """The parsed occupancy grid for `path`, re-read only if the file changed."""
        try:
            mtime = os.stat(path).st_mtime
        except OSError:
            return None
        cached = self._grid
        if cached and cached[0] == path and cached[1] == mtime:
            return cached[2]
        grid = mapcheck.OccupancyMap(str(path))
        self._grid = (path, mtime, grid)
        return grid

    def _pose_problems(self, x, y, yaw):
        """What is wrong with standing at (x, y), per the loaded map.

        Returns a list of human-readable problems, empty when the spot is fine.
        An empty list is also what a map we cannot read returns: refusing to
        seed because the checker failed would be worse than seeding unchecked,
        so the failure is logged and the operator's judgement wins.

        Never waits on map_server. It uses the path remembered from the last
        good probe, and only asks again (and only if the service is up right
        now) when it has none - this used to call check_map() every time, which
        is a 5 s wait whenever there is no map_server.
        """
        path = self._map_path
        if path and not os.path.isfile(str(path)):
            path = self._map_path = None
        if not path and self.map_params.service_is_ready():
            ok, detail = self.check_map(timeout=DISCOVERY_WAIT_S)
            path = detail if ok else None
        if not path:
            return []

        class _Pick:
            name = 'picked pose'

        pick = _Pick()
        pick.x, pick.y = x, y
        pick.qz, pick.qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

        try:
            grid = self._grid_for(str(path))
            if grid is None:
                return []
            return mapcheck.check_waypoint(grid, pick)['problems']
        except Exception as exc:                              # noqa: BLE001
            self.get_logger().warn(
                'could not check the picked pose against the map: %s' % exc)
            return []

    @staticmethod
    def _numeric_pose(x, y, yaw):
        try:
            x, y, yaw = float(x), float(y), float(yaw)
        except (TypeError, ValueError):
            raise WaypointError('pose needs numeric x, y and yaw')
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise WaypointError('pose values must be finite')
        return x, y, yaw

    def set_pose_at(self, x, y, yaw, settle=2.0):
        """Seed AMCL from a pose picked on the map, RViz "2D Pose Estimate" style.

        `yaw` is radians in the map frame. Unlike `set_initial_pose`, which
        trusts a surveyed waypoint, this is a human pointing at a picture - so
        it is checked against the map before it is published. Seeding AMCL
        inside a wall does not fail loudly; the particle filter just converges
        somewhere wrong and every goal afterwards misbehaves for reasons that
        look nothing like a bad initial pose.

        Answers fast: the AMCL check and the map check never wait on anything
        slow (see _require_amcl, _pose_problems). `settle` > 0 additionally
        waits that long for localization to come back; the bridge passes 0 and
        waits separately so it can tell the page "sent" first.
        """
        x, y, yaw = self._numeric_pose(x, y, yaw)

        self._require_amcl('set the 2D pose')

        problems = self._pose_problems(x, y, yaw)
        if problems:
            raise WaypointError(
                'the robot could not stand there: %s' % '; '.join(problems))

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.waypoints.frame_id
        # Stamp stays zero - see Waypoint.to_initial_pose for why.
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance = list(wp_mod.INITIAL_POSE_COVARIANCE)

        for _ in range(3):                    # cheap insurance against a drop
            self.initial_pose_pub.publish(msg)
            time.sleep(0.05)

        self.get_logger().info(
            'seeded AMCL at x=%.2f y=%.2f yaw=%.0f deg (picked on the map)'
            % (x, y, math.degrees(yaw)))

        if settle:
            return self.wait_localized(settle)
        self._emit('state', **self.state())
        return self.localized()

    def wait_localized(self, timeout=2.0):
        """Wait up to `timeout` s for AMCL to report a confident pose."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.localized():
                break
            time.sleep(0.1)
        self._emit('state', **self.state())
        return self.localized()

    # ----------------------------------------------------------------- goals

    def _mission_hint(self):
        mode = (self.robot_mode or {}).get('mode')
        return ' (mission: %s)' % mode if mode else ''

    def _require_ready(self):
        if self._map_ok is False:
            raise WaypointError(
                'loaded map does not match the waypoint file - refusing to move')
        if not self.nav_to_pose.server_is_ready():
            if not self.nav_to_pose.wait_for_server(
                    timeout_sec=DISCOVERY_WAIT_S):
                raise WaypointError(
                    'navigate_to_pose is not available - Nav2 is not running%s. '
                    'Start the NAVIGATE (or CLEAN) mission first.'
                    % self._mission_hint())
        if not self.localized():
            raise WaypointError(
                'robot is not localized - set the initial pose first')

    def _require_follower(self):
        """follow_waypoints is Nav2's waypoint_follower; tours need it."""
        if not self.follow_waypoints.server_is_ready():
            if not self.follow_waypoints.wait_for_server(
                    timeout_sec=DISCOVERY_WAIT_S):
                raise WaypointError(
                    'follow_waypoints is not available - the Nav2 waypoint '
                    'follower is not running%s. Start the NAVIGATE (or CLEAN) '
                    'mission first.' % self._mission_hint())

    def _send_nav_goal(self, pose, label):
        """NavigateToPose with `pose`; progress arrives as feedback/result."""
        self.cancel(quiet=True)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        with self._lock:
            self._active = label
            self._last_result = None
        self._emit('state', **self.state())

        future = self.nav_to_pose.send_goal_async(
            goal, feedback_callback=self._nav_feedback)
        future.add_done_callback(lambda f: self._goal_accepted(f, label))

    def goto(self, name):
        """Drive to one named waypoint. Any goal in flight is superseded."""
        wp = self.waypoints.get(name)
        self._require_ready()
        pose = wp.to_pose_stamped(self.waypoints.frame_id,
                                  self.get_clock().now().to_msg())
        self._send_nav_goal(pose, name)
        return True

    def goto_pose(self, x, y, yaw):
        """Drive to a spot picked on the map. `yaw` is radians, map frame.

        Same path as `goto`: the same readiness checks, the same goal
        bookkeeping, and the same feedback/result events, so the status card
        needs to know nothing about where the goal came from. Like
        `set_pose_at` it is checked against the map first - a goal inside a
        wall ends in recoveries and an abort that names nothing.
        """
        x, y, yaw = self._numeric_pose(x, y, yaw)
        self._require_ready()
        problems = self._pose_problems(x, y, yaw)
        if problems:
            raise WaypointError(
                'the robot cannot go there: %s' % '; '.join(problems))
        label = 'point (%.2f, %.2f)' % (x, y)
        pose = _pose_msg(self.waypoints.frame_id,
                         self.get_clock().now().to_msg(), x, y, yaw)
        self._send_nav_goal(pose, label)
        self.get_logger().info('goal: %s facing %.0f deg (picked on the map)'
                               % (label, math.degrees(yaw)))
        return label

    def _send_tour(self, poses, names, label, loops=0):
        """FollowWaypoints over `poses`; feedback names come from `names`."""
        self.cancel(quiet=True)
        goal = FollowWaypoints.Goal()
        goal.number_of_loops = int(loops)
        goal.goal_index = 0
        goal.poses = poses
        with self._lock:
            self._active = label
            self._last_result = None
        self._emit('state', **self.state())

        future = self.follow_waypoints.send_goal_async(
            goal, feedback_callback=lambda fb: self._tour_feedback(fb, names))
        future.add_done_callback(lambda f: self._goal_accepted(f, label))

    def tour(self, names, loops=0):
        """Visit several waypoints in order via the waypoint follower."""
        wps = [self.waypoints.get(n) for n in names]
        if not wps:
            raise WaypointError('tour needs at least one waypoint')
        self._require_ready()
        self._require_follower()

        stamp = self.get_clock().now().to_msg()
        poses = [w.to_pose_stamped(self.waypoints.frame_id, stamp)
                 for w in wps]
        self._send_tour(poses, list(names), ' -> '.join(names), loops)
        return True

    @staticmethod
    def plan_queue_yaws(points, start=None):
        """[[x, y, yaw?], ...] -> [(x, y, yaw), ...], faces the next point.

        A point with no yaw faces the next one; the last one keeps the heading
        it arrives on. A single point faces away from `start` (x, y) when the
        robot's pose is known. An explicit yaw is always respected.
        """
        out = []
        n = len(points)
        for i, (x, y, yaw) in enumerate(points):
            if yaw is None:
                if i + 1 < n:
                    nx, ny = points[i + 1][0], points[i + 1][1]
                    yaw = math.atan2(ny - y, nx - x)
                elif i > 0:
                    px, py = points[i - 1][0], points[i - 1][1]
                    yaw = math.atan2(y - py, x - px)
                elif start is not None and (start[0] != x or start[1] != y):
                    yaw = math.atan2(y - start[1], x - start[0])
                else:
                    yaw = 0.0
            out.append((x, y, yaw))
        return out

    def tour_points(self, points, loops=0):
        """Visit points picked on the map, in order, through FollowWaypoints.

        `points` is [[x, y], [x, y, yaw], ...] in map metres / radians. yaw is
        optional: left out, the robot faces the next point.
        """
        if not isinstance(points, (list, tuple)) or not points:
            raise WaypointError(
                'the queue is empty - click the map to drop at least one point')
        if len(points) > MAX_QUEUE_POINTS:
            raise WaypointError('at most %d queued points' % MAX_QUEUE_POINTS)
        parsed = []
        for i, p in enumerate(points):
            if not isinstance(p, (list, tuple)) or len(p) not in (2, 3):
                raise WaypointError('point %d must be [x, y] or [x, y, yaw]'
                                    % (i + 1))
            try:
                x, y = float(p[0]), float(p[1])
                yaw = None if len(p) < 3 or p[2] is None else float(p[2])
            except (TypeError, ValueError):
                raise WaypointError('point %d has a non-numeric value' % (i + 1))
            if not all(math.isfinite(v) for v in (x, y)) or (
                    yaw is not None and not math.isfinite(yaw)):
                raise WaypointError('point %d has a non-finite value' % (i + 1))
            parsed.append((x, y, yaw))

        self._require_ready()
        self._require_follower()

        start = self._tf_pose()
        planned = self.plan_queue_yaws(
            parsed, (start[0], start[1]) if start else None)
        for i, (x, y, yaw) in enumerate(planned):
            problems = self._pose_problems(x, y, yaw)
            if problems:
                raise WaypointError('point %d cannot be reached: %s'
                                    % (i + 1, '; '.join(problems)))

        stamp = self.get_clock().now().to_msg()
        frame = self.waypoints.frame_id
        poses = [_pose_msg(frame, stamp, x, y, yaw) for (x, y, yaw) in planned]
        names = ['P%d' % (i + 1) for i in range(len(planned))]
        label = 'queue of %d point%s' % (len(planned),
                                         '' if len(planned) == 1 else 's')
        self._send_tour(poses, names, label, loops)
        self.get_logger().info('%s sent to follow_waypoints' % label)
        return label

    def _goal_accepted(self, future, label):
        try:
            handle = future.result()
        except Exception as exc:
            self._finish(label, 'REJECTED', str(exc))
            return
        if handle is None or not handle.accepted:
            self._finish(label, 'REJECTED', 'server rejected the goal')
            return
        with self._lock:
            self._goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._goal_done(f, label))

    def _goal_done(self, future, label):
        try:
            outcome = future.result()
        except Exception as exc:
            self._finish(label, 'ERROR', str(exc))
            return
        status = STATUS_NAMES.get(outcome.status, 'UNKNOWN(%s)' % outcome.status)
        detail = getattr(outcome.result, 'error_msg', '') or ''
        self._finish(label, status, detail)

    def _finish(self, label, status, detail=''):
        with self._lock:
            self._goal_handle = None
            self._active = None
            self._last_result = {'destination': label, 'status': status,
                                 'detail': detail}
        self.get_logger().info('%s: %s %s' % (label, status, detail))
        self._emit('result', destination=label, status=status, detail=detail)
        self._emit('state', **self.state())

    def _nav_feedback(self, msg):
        fb = msg.feedback
        self._emit(
            'feedback',
            distance_remaining=round(float(fb.distance_remaining), 3),
            eta=round(fb.estimated_time_remaining.sec
                      + fb.estimated_time_remaining.nanosec / 1e9, 1),
            elapsed=round(fb.navigation_time.sec
                          + fb.navigation_time.nanosec / 1e9, 1),
            recoveries=int(fb.number_of_recoveries))

    def _tour_feedback(self, msg, names):
        i = int(msg.feedback.current_waypoint)
        self._emit('feedback', waypoint_index=i,
                   waypoint_name=names[i] if i < len(names) else '?',
                   total=len(names))

    # --------------------------------------------------------------- zones

    def _zone_plan(self, msg):
        """Cache the sweep so the browser can draw it over the map."""
        self.zone_plan = [(p.pose.position.x, p.pose.position.y)
                          for p in msg.poses]
        self._emit('zone_plan', points=len(self.zone_plan))

    def _zone_state(self, s):
        self.zone_state = s
        self._emit('zone_state', state=s, progress=self.zone_progress)

    def _zone_progress(self, v):
        self.zone_progress = float(v)
        self._emit('zone_state', state=self.zone_state, progress=float(v))

    def _zone_listeners(self, topic):
        """How many subscribers `topic` has, waiting DISCOVERY_WAIT_S for one.

        These topics are volatile: a message published with no subscriber is
        gone and nothing anywhere says so. The page used to be told "sent"
        regardless, so a zone aimed at a backend that was not running looked
        exactly like a zone that was working.
        """
        deadline = time.time() + DISCOVERY_WAIT_S
        while True:
            n = self.count_subscribers(topic)
            if n or time.time() >= deadline:
                return n
            time.sleep(0.05)

    def _require_zone_backend(self, topic):
        n = self._zone_listeners(topic)
        if n == 0:
            raise WaypointError(
                'zone_coverage is not running - nothing is subscribed to %s. '
                'Start the navigate mission (zone cleaning runs with it), '
                'then try again.' % topic)
        return n

    def send_zone(self, points):
        """Publish a drawn zone. `points` is [[x, y], ...] in map metres.

        Returns the number of subscribers it was delivered to; raises if there
        are none.
        """
        if not points or len(points) < 3:
            raise WaypointError('a zone needs at least 3 points')
        listeners = self._require_zone_backend(ZONE_TOPIC)
        msg = PolygonStamped()
        msg.header.frame_id = self.waypoints.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in points:
            pt = Point32()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.0
            msg.polygon.points.append(pt)
        # Latch-free topic, and zone_coverage may still be starting: repeat.
        for _ in range(3):
            self.zone_pub.publish(msg)
            time.sleep(0.1)
        self.get_logger().info('sent zone with %d points to %d listener(s)'
                               % (len(points), listeners))
        return listeners

    def zone_command(self, command):
        """'start <name>' | 'plan <name>' | 'stop'.

        Returns the number of subscribers it was delivered to; raises if there
        are none.
        """
        if not command:
            raise WaypointError('empty zone command')
        listeners = self._require_zone_backend(ZONE_CMD_TOPIC)
        msg = StringMsg()
        msg.data = str(command)
        for _ in range(2):
            self.zone_cmd_pub.publish(msg)
            time.sleep(0.05)
        self.get_logger().info('zone command: %s (%d listener(s))'
                               % (command, listeners))
        return listeners

    # ------------------------------------------------- autonomous missions

    def _publish_string(self, pub, text, repeat=2, gap=0.05):
        msg = StringMsg()
        msg.data = str(text)
        for i in range(repeat):
            pub.publish(msg)
            if gap and i + 1 < repeat:
                time.sleep(gap)

    def _robot_mode_cb(self, msg):
        try:
            parsed = json.loads(msg.data)
            if not isinstance(parsed, dict):
                parsed = {'mode': str(parsed)}
        except ValueError:
            parsed = {'mode': msg.data.strip()}    # bare mode name
        before = (self.robot_mode.get('mode'), self.robot_mode.get('map'))
        self.robot_mode = parsed
        if (parsed.get('mode'), parsed.get('map')) != before:
            # A different mission means a different (or no) map_server; the
            # path and grid remembered for the old one must not be trusted.
            self._map_path = None
            self._grid = None
        if parsed.get('error'):
            self.get_logger().warn('mode_manager: %s' % parsed['error'])
        self._emit('robot_mode', **parsed)

    def _explorer_mode_cb(self, msg):
        mode = msg.data.strip()
        self.explorer_mode = mode
        self._explorer_seen_at = time.time()
        self._emit('explorer_mode', mode=mode)

    def _explorer_status_cb(self, msg):
        try:
            parsed = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(parsed, dict):
            return
        self.explorer_status = parsed
        self._explorer_seen_at = time.time()
        self._emit('explorer_status', **parsed)

    def _explorer_liveness(self):
        """Forget the explorer once its process is gone.

        Its state topic is latched, but a latched message dies with its
        publisher - and mode_manager ends a mission with a signal, so the last
        thing we heard may well be 'explore', not 'stopped'. Left alone that
        would keep the page's mission row, the link-lost warning and the
        teleop pause forever.
        """
        if self.explorer_mode is None:
            return
        gone = self.count_publishers('/frontier_explorer/mode') == 0
        stale = (self.explorer_mode in EXPLORER_ALIVE
                 and time.time() - self._explorer_seen_at > EXPLORER_STALE_S)
        if gone or stale:
            self.get_logger().info(
                'frontier_explorer is gone (last state %r)' % self.explorer_mode)
            self.explorer_mode = None
            self.explorer_status = {}
            self.explore_path = []
            self._emit('explorer_mode', mode=None)
            self._emit('explore_path', points=0)

    def _coverage_cb(self, msg):
        # The list can be hundreds of squares at 1 Hz, so only the count goes
        # out over the WebSocket; the page fetches the list over HTTP.
        self.coverage_blocks = [reduce_marker(m) for m in msg.markers
                                if m.action in (0,)]    # ADD/MODIFY only
        self._emit('coverage_blocks', count=len(self.coverage_blocks))

    def _explore_path_cb(self, msg):
        pts = [(round(p.pose.position.x, 3), round(p.pose.position.y, 3))
               for p in msg.poses]
        step = max(1, -(-len(pts) // EXPLORE_PATH_MAX_POINTS))   # ceil
        thinned = pts[::step]
        if pts and thinned[-1] != pts[-1]:
            thinned.append(pts[-1])                  # keep the true endpoint
        self.explore_path = thinned
        self._emit('explore_path', points=len(thinned))

    def _map_cb(self, msg):
        with self._lock:
            self._map_msg = msg
            self._map_seq += 1
            seq = self._map_seq
        self._emit('live_map', seq=seq, width=int(msg.info.width),
                   height=int(msg.info.height))

    def live_map(self):
        """The latest /map as the page's JSON, encoded once per message."""
        with self._lock:
            msg, seq, cached = self._map_msg, self._map_seq, self._map_encoded
        if msg is None:
            return None
        if cached is not None and cached['seq'] == seq:
            return cached
        info = msg.info
        encoded = {
            'seq': seq,
            'width': int(info.width),
            'height': int(info.height),
            'resolution': float(info.resolution),
            'origin': [float(info.origin.position.x),
                       float(info.origin.position.y)],
            'frame_id': msg.header.frame_id,
            'rle': encode_occupancy_rle(msg.data),
        }
        with self._lock:
            if self._map_seq == seq:
                self._map_encoded = encoded
        return encoded

    def coverage_summary(self):
        st = self.explorer_status
        if 'blocks_total' in st:
            return {'done': int(st.get('blocks_done') or 0),
                    'total': int(st.get('blocks_total') or 0),
                    'skipped': int(st.get('blocks_skipped') or 0)}
        blocks = self.coverage_blocks
        return {'done': sum(1 for b in blocks if b['s'] == 'g'),
                'total': len(blocks),
                'skipped': sum(1 for b in blocks if b['s'] == 'x')}

    def set_robot_mode(self, mode, map_name='', new=False):
        """Ask mode_manager for a mission. It validates the map; we only
        refuse what is certainly wrong so the page hears about it at once."""
        mode = str(mode or '').strip().lower()
        if mode not in ROBOT_MODES:
            raise WaypointError('unknown mode %r (one of %s)'
                                % (mode, ', '.join(ROBOT_MODES)))
        map_name = str(map_name or '').strip()
        if mode in ('explore', 'clean') and not map_name:
            raise WaypointError('%s needs a map name' % mode)
        payload = json.dumps({'mode': mode, 'map': map_name, 'new': bool(new)})
        # Volatile topic and mode_manager may be mid-restart: repeat.
        self._publish_string(self.set_mode_pub, payload, repeat=2)
        self.get_logger().info('set_robot_mode: %s' % payload)
        return True

    def save_map(self, name=''):
        """Ask map_saver to save the live map, the pose and the pose graph.

        Empty name = whatever the running mission calls its map. The saver
        validates the name; this only refuses what can never work. Returns the
        request id, which the matching 'save_result' event echoes.
        """
        name = str(name or '').strip()
        if len(name) > 64:
            raise WaypointError('map name is too long (64 characters max)')
        if self.count_subscribers('/save_map') == 0:
            raise WaypointError(
                'nothing is listening on /save_map - no mission with a map '
                'saver is running (start EXPLORE, CLEAN or MANUAL)')
        request_id = '%x' % int(time.time() * 1000)
        self._publish_string(
            self.save_map_pub,
            json.dumps({'name': name or self.robot_mode.get('map') or '',
                        'id': request_id}),
            repeat=1)
        self.get_logger().info('save_map: %r (%s)' % (name, request_id))
        return request_id

    def _save_result_cb(self, msg):
        try:
            parsed = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(parsed, dict):
            return
        if not parsed.get('ok'):
            self.get_logger().warn('map save failed: %s' % parsed.get('error'))
        self._emit('save_result', **parsed)

    def explorer_command(self, cmd):
        cmd = str(cmd or '').strip().lower()
        if cmd not in EXPLORER_COMMANDS:
            raise WaypointError('unknown explorer command %r (one of %s)'
                                % (cmd, ', '.join(EXPLORER_COMMANDS)))
        self._publish_string(self.explorer_cmd_pub, cmd, repeat=2)
        self.get_logger().info('explorer command: %s' % cmd)
        return True

    def _halt_missions(self, gap=0.05):
        """Tell every autonomous brain to let go of the base."""
        self._publish_string(self.explorer_cmd_pub, 'stop', repeat=2, gap=gap)
        self._publish_string(
            self.set_mode_pub,
            json.dumps({'mode': 'stop', 'map': '', 'new': False}),
            repeat=2, gap=gap)

    # -------------------------------------------------------- motor limits

    def _publish_pid_limits(self):
        msg = Vector3()
        msg.x = float(self.pid_limits['min_pwm'])
        msg.y = float(self.pid_limits['max_pwm'])
        msg.z = float(self.pid_limits['max_speed'])
        self.pid_limits_pub.publish(msg)

    def _pid_limits_tick(self):
        # Only once the operator has actually set something. Until then the
        # ESP32's own defaults are the truth, and republishing a copy of them
        # would just be noise on the serial link.
        if self._pid_limits_sent:
            self._publish_pid_limits()

    def set_pid_limits(self, min_pwm=None, max_pwm=None, max_speed=None):
        """Set the ESP32's PWM band and speed cap. Unset fields keep their value.

        Validated here as well as in the firmware. The firmware clamps silently
        - it has no way to answer - so a value rejected only there would leave
        the page displaying a limit the robot is not using.
        """
        limits = dict(self.pid_limits)

        if min_pwm is not None:
            limits['min_pwm'] = int(min_pwm)
        if max_pwm is not None:
            limits['max_pwm'] = int(max_pwm)
        if max_speed is not None:
            limits['max_speed'] = float(max_speed)

        if not (PWM_ABS_MIN <= limits['min_pwm'] <= PWM_ABS_MAX):
            raise WaypointError(
                'min PWM must be %d-%d' % (PWM_ABS_MIN, PWM_ABS_MAX))
        if not (MAX_PWM_LOWER_BOUND <= limits['max_pwm'] <= PWM_ABS_MAX):
            raise WaypointError(
                'max PWM must be %d-%d' % (MAX_PWM_LOWER_BOUND, PWM_ABS_MAX))
        if limits['min_pwm'] >= limits['max_pwm']:
            # The firmware would quietly fix this by dropping min to max-1.
            # Refusing is better: an inverted band means the slider was dragged
            # the wrong way, and silently accepting it hides that.
            raise WaypointError('min PWM (%d) must be below max PWM (%d)'
                                % (limits['min_pwm'], limits['max_pwm']))
        if not (SPEED_ABS_MIN <= limits['max_speed'] <= SPEED_ABS_MAX):
            raise WaypointError('max speed must be %.2f-%.2f m/s'
                                % (SPEED_ABS_MIN, SPEED_ABS_MAX))

        self.pid_limits = limits
        self._pid_limits_sent = True

        # micro-ROS may still be reconnecting; repeat as the zone publisher does.
        for _ in range(3):
            self._publish_pid_limits()
            time.sleep(0.05)

        self.get_logger().info(
            'motor limits: pwm %d..%d, max speed %.3f m/s'
            % (limits['min_pwm'], limits['max_pwm'], limits['max_speed']))
        self._emit('pid_limits', **limits)
        return limits

    def reset_pid_limits(self):
        """Back to the firmware's compiled defaults."""
        return self.set_pid_limits(**PID_LIMIT_DEFAULTS)

    # ------------------------------------------------------------- teleop

    def teleop(self, vx, wz):
        """Accept a manual drive command from the UI.

        Stores the command; the timer below is what actually publishes, at a
        fixed rate. The browser can send as fast or as erratically as it likes
        without that reaching the serial link.
        """
        try:
            vx = float(vx)
            wz = float(wz)
        except (TypeError, ValueError):
            raise WaypointError('teleop needs numeric vx and wz')
        if not (math.isfinite(vx) and math.isfinite(wz)):
            raise WaypointError('teleop values must be finite')

        vx = max(-TELEOP_VX_MAX, min(TELEOP_VX_MAX, vx))
        wz = max(-TELEOP_WZ_MAX, min(TELEOP_WZ_MAX, wz))

        # Driving by hand while Nav2 owns the base means two controllers
        # fighting over /cmd_vel. The human wins.
        with self._lock:
            navigating = self._active is not None
        if navigating:
            self.cancel(quiet=True)
            self.get_logger().info('teleop took over - navigation cancelled')

        # Same for the explorer, which publishes /cmd_vel at 10 Hz of its
        # own. Paused, not stopped: the mission and its map survive, and it
        # is never resumed from here - that is the operator's decision. This
        # runs on the hot path at 10 Hz, so no sleeping, and re-sent at most
        # once a second until the explorer reports 'paused'.
        now = time.time()
        if (self.explorer_mode in EXPLORER_DRIVING
                and now - self._explorer_pause_sent > 1.0):
            self._explorer_pause_sent = now
            self._publish_string(self.explorer_cmd_pub, 'pause', repeat=2,
                                 gap=0)
            self.get_logger().warn(
                'teleop took over - explorer paused (was %r); resume it '
                'yourself when done' % self.explorer_mode)

        with self._lock:
            self._teleop = (vx, wz, time.time() + TELEOP_DEADMAN)
        return True

    def teleop_stop(self):
        """Zero the base now. Never sleeps.

        The browser sends this every time the stick is released and the bridge
        calls it when a client disconnects; both used to sleep ~0.1 s, on the
        event loop. One zero goes out immediately and the two repeats (a drop
        insurance, as before) follow from a timer thread.
        """
        with self._lock:
            active = self._teleop is not None
            self._teleop = None
        if active:
            self.cmd_vel_pub.publish(Twist())

            def again():
                with self._lock:
                    if self._teleop is not None:   # driving again; leave it
                        return
                self.cmd_vel_pub.publish(Twist())

            for delay in (0.05, 0.10):
                t = threading.Timer(delay, again)
                t.daemon = True
                t.start()
        return active

    def _teleop_tick(self):
        with self._lock:
            held = self._teleop
        if held is None:
            return
        vx, wz, expiry = held
        if time.time() > expiry:
            # Browser went quiet: released, tab closed, or wifi dropped.
            with self._lock:
                self._teleop = None
            self.cmd_vel_pub.publish(Twist())
            self._emit('state', **self.state())
            return
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    # ------------------------------------------------------------ stopping

    def cancel(self, quiet=False):
        with self._lock:
            handle = self._goal_handle
        if handle is None:
            return False
        handle.cancel_goal_async()
        if not quiet:
            self.get_logger().info('goal cancelled')
        return True

    def estop(self, stop_mission=True):
        """Cancel and actively zero the base.

        Cancelling alone is not enough: the last velocity command stays live
        until the firmware's 500 ms timeout expires, so publish zeros too.

        And zeros alone are not enough either: the ESP32 obeys the last
        command it hears, and a running explore/clean mission publishes at
        10 Hz. So the explorer is told to stop and mode_manager is told to end
        the mission first, and the explorer stop is repeated while the zeros
        go out. Note this also ends a running `navigate` mission - an e-stop
        is not meant to be subtle.

        `stop_mission=False` keeps the mission alive and only zeroes what this
        node owns. That is for the bridge's own shutdown path: mode_manager
        owns mission lifetime precisely so a mission survives the web UI going
        away, and restarting ./web.sh must not abort a clean that is halfway
        through a room. A human pressing STOP always gets the full stop.
        """
        if stop_mission:
            self._halt_missions(gap=0)
        self.cancel(quiet=True)
        with self._lock:
            self._teleop = None
        stop = Twist()
        for i in range(15):
            self.cmd_vel_pub.publish(stop)
            if stop_mission and i % 5 == 4:
                self._publish_string(self.explorer_cmd_pub, 'stop',
                                     repeat=1)
            time.sleep(0.05)
        with self._lock:
            self._active = None
            self._last_result = {'destination': None, 'status': 'STOPPED',
                                 'detail': 'emergency stop'}
        self.get_logger().warn('EMERGENCY STOP')
        self._emit('result', destination=None, status='STOPPED',
                   detail='emergency stop')
        self._emit('state', **self.state())
        return True
