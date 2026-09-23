"""ROS side of the waypoint bridge: localization, goal dispatch, stop.

Everything the web layer needs from ROS lives here, so the bridge never touches
rclpy directly. The node is spun by a MultiThreadedExecutor on its own thread;
public methods below are called from the asyncio thread and are safe to call
there because they only publish, start actions, or read state under a lock.

State changes are pushed out through `on_event`, a plain callable the bridge
installs. It is invoked from ROS callback threads, so the bridge is responsible
for hopping back onto its event loop.
"""

import math
import threading
import time

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from .waypoints import WaypointError

# Above this the particle cloud is too spread out to trust for navigation.
LOCALIZED_COV_MAX = 0.5

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


def yaw_from_quat(z, w):
    return math.degrees(2.0 * math.atan2(z, w))


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

        self.create_timer(1.0 / TELEOP_HZ, self._teleop_tick,
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

    def _amcl_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self._lock:
            self._pose = (p.x, p.y, yaw_from_quat(q.z, q.w))
            self._cov_xx = msg.pose.covariance[0]
            self._cov_yy = msg.pose.covariance[7]
        self._emit('pose', x=p.x, y=p.y, yaw=yaw_from_quat(q.z, q.w))

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

    def localized(self):
        """True when AMCL is alive and its last estimate was confident.

        Liveness comes from the map -> base_footprint transform rather than the
        age of the last /amcl_pose: AMCL publishes that topic only when the
        filter updates, so a robot standing still stops publishing it entirely
        while remaining perfectly well localized. The covariance is sticky - it
        describes the last real update, which is exactly what we want.
        """
        if not self.tf_buffer.can_transform('map', 'base_footprint',
                                            rclpy.time.Time()):
            return False
        with self._lock:
            if self._pose is None:
                # AMCL is broadcasting map -> odom, which it only does once it
                # has an estimate, but we have not seen an /amcl_pose yet. That
                # is the normal cold start against an already-localized robot:
                # trust the transform until a real covariance arrives.
                return True
            return (self._cov_xx < LOCALIZED_COV_MAX
                    and self._cov_yy < LOCALIZED_COV_MAX)

    def state(self):
        with self._lock:
            pose, active, result = self._pose, self._active, self._last_result
            teleop = self._teleop is not None
        pose = self._tf_pose() or pose        # TF is live even when AMCL is quiet
        return {
            'teleop': teleop,
            'type': 'state',
            'map': self.waypoints.map_name,
            'map_ok': self._map_ok,
            'localized': self.localized(),
            'navigating': active is not None,
            'destination': active,
            'last_result': result,
            'pose': ({'x': pose[0], 'y': pose[1], 'yaw': pose[2]}
                     if pose else None),
            'nav_ready': self.nav_to_pose.server_is_ready(),
        }

    def _state_tick(self):
        s = self.state()
        # Pose is excluded: it changes constantly while driving and has its own
        # 'pose' event, so including it here would broadcast state at 1 Hz
        # forever for no reason.
        sig = (s['localized'], s['navigating'], s['destination'],
               s['nav_ready'], s['map_ok'], s['teleop'])
        if sig != self._last_state_sig:
            self._last_state_sig = sig
            self._emit('state', **s)

    def check_map(self, timeout=5.0):
        """Compare the loaded map against the one the waypoints belong to."""
        if not self.map_params.wait_for_service(timeout_sec=timeout):
            self._map_ok = None
            return None, 'map_server not reachable'
        req = GetParameters.Request()
        req.names = ['yaml_filename']
        future = self.map_params.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done() or not future.result().values:
            self._map_ok = None
            return None, 'map_server did not answer'
        loaded = future.result().values[0].string_value
        ok = self.waypoints.matches_map(loaded)
        self._map_ok = ok
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
        deadline = time.time() + 5.0
        while self.count_subscribers('/initialpose') == 0:
            if time.time() > deadline:
                raise WaypointError(
                    'nothing is subscribed to /initialpose - is AMCL running?')
            time.sleep(0.1)

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

    # ----------------------------------------------------------------- goals

    def _require_ready(self):
        if self._map_ok is False:
            raise WaypointError(
                'loaded map does not match the waypoint file - refusing to move')
        if not self.nav_to_pose.server_is_ready():
            if not self.nav_to_pose.wait_for_server(timeout_sec=3.0):
                raise WaypointError('navigate_to_pose action server not available')
        if not self.localized():
            raise WaypointError(
                'robot is not localized - set the initial pose first')

    def goto(self, name):
        """Drive to one named waypoint. Any goal in flight is superseded."""
        wp = self.waypoints.get(name)
        self._require_ready()
        self.cancel(quiet=True)

        goal = NavigateToPose.Goal()
        goal.pose = wp.to_pose_stamped(self.waypoints.frame_id,
                                       self.get_clock().now().to_msg())
        with self._lock:
            self._active = name
            self._last_result = None
        self._emit('state', **self.state())

        future = self.nav_to_pose.send_goal_async(
            goal, feedback_callback=self._nav_feedback)
        future.add_done_callback(lambda f: self._goal_accepted(f, name))
        return True

    def tour(self, names, loops=0):
        """Visit several waypoints in order via the waypoint follower."""
        wps = [self.waypoints.get(n) for n in names]
        if not wps:
            raise WaypointError('tour needs at least one waypoint')
        self._require_ready()
        if not self.follow_waypoints.wait_for_server(timeout_sec=3.0):
            raise WaypointError('follow_waypoints action server not available')
        self.cancel(quiet=True)

        stamp = self.get_clock().now().to_msg()
        goal = FollowWaypoints.Goal()
        goal.number_of_loops = int(loops)
        goal.goal_index = 0
        goal.poses = [w.to_pose_stamped(self.waypoints.frame_id, stamp)
                      for w in wps]

        label = ' -> '.join(names)
        with self._lock:
            self._active = label
            self._last_result = None
        self._emit('state', **self.state())

        future = self.follow_waypoints.send_goal_async(
            goal, feedback_callback=lambda fb: self._tour_feedback(fb, names))
        future.add_done_callback(lambda f: self._goal_accepted(f, label))
        return True

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

        with self._lock:
            self._teleop = (vx, wz, time.time() + TELEOP_DEADMAN)
        return True

    def teleop_stop(self):
        with self._lock:
            active = self._teleop is not None
            self._teleop = None
        if active:
            stop = Twist()
            for _ in range(3):
                self.cmd_vel_pub.publish(stop)
                time.sleep(0.03)
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

    def estop(self):
        """Cancel and actively zero the base.

        Cancelling alone is not enough: the last velocity command stays live
        until the firmware's 500 ms timeout expires, so publish zeros too.
        """
        self.cancel(quiet=True)
        with self._lock:
            self._teleop = None
        stop = Twist()
        for _ in range(15):
            self.cmd_vel_pub.publish(stop)
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
