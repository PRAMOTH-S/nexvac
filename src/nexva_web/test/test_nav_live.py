"""NavClient and Bridge against a real ROS graph - isolated on ROS_DOMAIN_ID 64.

A second node ("web_test_helper") plays everything the robot would: a fake
AMCL listening on /initialpose, fake Nav2 action servers, a zone backend, an
IMU and motion_check. Nothing here touches a real robot or the default domain.

Skipped (not failed) if rclpy cannot be imported. Run it with ROS sourced:
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/nexva_web/test/test_nav_live.py
"""

import asyncio
import math
import os
import sys
import tempfile
import threading
import time

# Isolate BEFORE rclpy.init: another domain, and no discovery beyond this host.
os.environ['ROS_DOMAIN_ID'] = '64'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _stubs                                                   # noqa: E402

_stubs.install_aiohttp_stub()

try:
    import rclpy
    from geometry_msgs.msg import (
        PolygonStamped, PoseWithCovarianceStamped, TransformStamped)
    from nav2_msgs.action import FollowWaypoints, NavigateToPose
    from rclpy.action import ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import (
        QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
        QoSReliabilityPolicy)
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool, String
    from tf2_ros import StaticTransformBroadcaster
    HAVE_ROS = True
except ImportError:                                             # pragma: no cover
    HAVE_ROS = False

import pytest                                                   # noqa: E402

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='rclpy not available')

if HAVE_ROS:
    from nexva_web import waypoints as wp_mod                   # noqa: E402
    from nexva_web import web_bridge                            # noqa: E402
    from nexva_web.nav_client import NavClient                  # noqa: E402
    from nexva_web.web_bridge import dumps                      # noqa: E402


def wait_until(pred, timeout=5.0, step=0.05):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(step)
    return bool(pred())


class Rig:
    """The NavClient under test plus a helper node that fakes the robot."""

    def __init__(self):
        rclpy.init()
        wps = wp_mod.WaypointSet(
            'testmap', 'map',
            [wp_mod.Waypoint('A', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))], '')
        self.node = NavClient(wps)
        self.helper = rclpy.create_node('web_test_helper')
        self.cb = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.events = []
        # map -> base_footprint, so NavClient.localized() is true
        self.tf = StaticTransformBroadcaster(self.helper)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'base_footprint'
        t.transform.rotation.w = 1.0
        t.header.stamp = self.helper.get_clock().now().to_msg()
        self.tf.sendTransform(t)
        assert wait_until(self.node.localized, 5.0), 'static TF never arrived'

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.helper.destroy_node()
        rclpy.try_shutdown()


_RIG = None


def rig():
    global _RIG
    if _RIG is None:
        _RIG = Rig()
    r = _RIG
    r.events = []
    r.node.on_event = r.events.append
    return r


def teardown_module(module):
    global _RIG
    if _RIG is not None:
        _RIG.shutdown()
        _RIG = None


def direct(frames):
    """Replies only: drop the batched event frames the node also pushes."""
    return [f for f in frames if f.get('type') != 'batch']


def run_bridge(r, coro_fn):
    """Run `coro_fn(bridge, ws)` on a fresh loop with a real Bridge on r.node."""
    async def main():
        ws = _stubs.FakeWS()
        bridge = web_bridge.Bridge(r.node, asyncio.get_running_loop(), None)
        bridge.clients.add(ws)
        try:
            return await coro_fn(bridge, ws)
        finally:
            bridge.pools.shutdown()
    try:
        return asyncio.run(main())
    finally:
        r.node.on_event = r.events.append


# ----------------------------------------------------------- Set 2D pose

def test_set_pose_reaches_amcl_and_replies_within_a_second():
    r = rig()
    got = []
    sub = r.helper.create_subscription(
        PoseWithCovarianceStamped, '/initialpose', got.append, 10)
    try:
        assert wait_until(lambda: r.node.count_subscribers('/initialpose') >= 1)

        async def go(bridge, ws):
            t0 = time.monotonic()
            task = asyncio.ensure_future(bridge.dispatch(
                ws, {'cmd': 'set_pose', 'x': 1.25, 'y': -0.5, 'yaw': 0.7}))
            while not ws.sent and time.monotonic() - t0 < 5:
                await asyncio.sleep(0.01)
            first = time.monotonic() - t0
            await task
            return first, ws.sent

        first, frames = run_bridge(r, go)
        frames = direct(frames)
        assert first < 1.0, 'no feedback for %.2fs' % first
        assert [f.get('stage') for f in frames] == ['sent', 'done']
        assert frames[0]['type'] == 'initial_pose'
        assert frames[1]['localized'] is True
        assert wait_until(lambda: len(got) >= 3, 3.0), len(got)
        m = got[0]
        assert m.header.frame_id == 'map'
        assert (m.header.stamp.sec, m.header.stamp.nanosec) == (0, 0)
        assert m.pose.pose.position.x == pytest.approx(1.25)
        assert m.pose.pose.position.y == pytest.approx(-0.5)
        assert m.pose.pose.orientation.z == pytest.approx(math.sin(0.35))
        assert m.pose.pose.orientation.w == pytest.approx(math.cos(0.35))
        assert list(m.pose.covariance) == list(wp_mod.INITIAL_POSE_COVARIANCE)
    finally:
        r.helper.destroy_subscription(sub)


def test_set_pose_with_no_amcl_fails_fast_and_says_why():
    r = rig()
    assert wait_until(lambda: r.node.count_subscribers('/initialpose') == 0)
    r.node.robot_mode = {'mode': 'explore'}

    async def go(bridge, ws):
        t0 = time.monotonic()
        await bridge.dispatch(
            ws, {'cmd': 'set_pose', 'x': 1.0, 'y': 1.0, 'yaw': 0.0})
        return time.monotonic() - t0, ws.sent
    try:
        took, frames = run_bridge(r, go)
    finally:
        r.node.robot_mode = {}
    frames = direct(frames)
    assert took < 1.0, took
    assert len(frames) == 1 and frames[0]['type'] == 'error'
    assert frames[0]['cmd'] == 'set_pose'
    msg = frames[0]['msg']
    assert 'AMCL is not running' in msg and '(mission: explore)' in msg
    assert 'NAVIGATE' in msg and 'CLEAN' in msg


def test_pose_check_never_probes_map_server():
    r = rig()
    calls = []
    real = r.node.check_map
    r.node.check_map = lambda *a, **k: calls.append((a, k)) or real(*a, **k)
    try:
        t0 = time.monotonic()
        assert r.node._pose_problems(0.0, 0.0, 0.0) == []
        assert time.monotonic() - t0 < 0.2
        assert calls == [], 'pose check re-probed map_server'

        # check_map(wait=False) answers "not running" at once
        t0 = time.monotonic()
        ok, detail = real(timeout=5.0, wait=False)
        assert ok is None and 'not running' in detail
        assert time.monotonic() - t0 < 0.2
    finally:
        r.node.check_map = real


def test_pose_check_uses_the_remembered_map_and_forgets_it_on_a_mission_change():
    r = rig()
    with tempfile.TemporaryDirectory() as d:
        w, h = 20, 20
        data = bytearray([254] * (w * h))             # free
        for j in range(h):                            # a wall: column 10
            data[j * w + 10] = 0
        with open(os.path.join(d, 'm.pgm'), 'wb') as fh:
            fh.write(b'P5\n20 20\n255\n' + bytes(data))
        yml = os.path.join(d, 'm.yaml')
        with open(yml, 'w') as fh:
            fh.write('image: m.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0]\n')
        r.node._map_path = yml
        try:
            inside_wall = r.node._pose_problems(1.05, 1.0, 0.0)
            assert inside_wall and any('occupied' in p for p in inside_wall)
            assert r.node._pose_problems(0.3, 1.0, 0.0) == []     # clear of the wall
            # a different mission: the remembered map is for the old one
            msg = String()
            msg.data = '{"mode": "explore", "map": "other"}'
            r.node._robot_mode_cb(msg)
            assert r.node._map_path is None
            assert r.node._pose_problems(1.05, 1.0, 0.0) == []
        finally:
            r.node._map_path = None
            r.node.robot_mode = {}


# ------------------------------------------------------------- zones

def test_zone_and_zone_command_report_whether_anyone_received_them():
    r = rig()
    assert r.node.count_subscribers('/zone_coverage/zone') == 0
    poly = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]

    async def refused(bridge, ws):
        t0 = time.monotonic()
        await bridge.dispatch(ws, {'cmd': 'zone', 'points': poly})
        await bridge.dispatch(ws, {'cmd': 'zone_cmd', 'command': 'start drawn'})
        return time.monotonic() - t0, ws.sent
    took, frames = run_bridge(r, refused)
    frames = direct(frames)
    assert took < 2.0
    assert [f['type'] for f in frames] == ['error', 'error'], frames
    assert [f['cmd'] for f in frames] == ['zone', 'zone_cmd']
    assert all('zone_coverage is not running' in f['msg'] for f in frames)
    assert not [f for f in frames if f['type'] == 'ack'], 'acked into the void'

    zones, cmds = [], []
    s1 = r.helper.create_subscription(PolygonStamped, '/zone_coverage/zone',
                                      zones.append, 10)
    s2 = r.helper.create_subscription(String, '/zone_coverage/command',
                                      cmds.append, 10)
    try:
        assert wait_until(
            lambda: r.node.count_subscribers('/zone_coverage/zone') >= 1
            and r.node.count_subscribers('/zone_coverage/command') >= 1)

        async def accepted(bridge, ws):
            await bridge.dispatch(ws, {'cmd': 'zone', 'points': poly})
            await bridge.dispatch(
                ws, {'cmd': 'zone_cmd', 'command': 'plan drawn'})
            return ws.sent
        frames = direct(run_bridge(r, accepted))
        assert [f['type'] for f in frames] == ['ack', 'ack'], frames
        assert frames[0]['listeners'] == 1 and frames[1]['listeners'] == 1
        assert wait_until(lambda: zones and cmds, 3.0)
        assert len(zones[0].polygon.points) == 4
        assert zones[0].header.frame_id == 'map'
        assert cmds[0].data == 'plan drawn'
    finally:
        r.helper.destroy_subscription(s1)
        r.helper.destroy_subscription(s2)


# ------------------------------------------------- goals and tours (absent)

def test_goals_and_tours_name_the_server_that_is_missing():
    r = rig()
    assert not r.node.nav_to_pose.server_is_ready()
    for label, call in [
            ('goto', lambda: r.node.goto('A')),
            ('goto_pose', lambda: r.node.goto_pose(1.0, 1.0, 0.0)),
            ('tour', lambda: r.node.tour(['A'])),
            ('tour_points', lambda: r.node.tour_points([[1.0, 1.0]]))]:
        t0 = time.monotonic()
        with pytest.raises(wp_mod.WaypointError) as err:
            call()
        assert time.monotonic() - t0 < 1.0, label
        assert 'navigate_to_pose is not available' in str(err.value), label
        assert 'NAVIGATE' in str(err.value)

    # and it reaches the browser as an error frame naming the command
    async def go(bridge, ws):
        await bridge.dispatch(ws, {'cmd': 'goto', 'waypoint': 'A'})
        return ws.sent
    frames = direct(run_bridge(r, go))
    assert frames[0]['type'] == 'error' and frames[0]['cmd'] == 'goto'
    assert 'navigate_to_pose is not available' in frames[0]['msg']


# ------------------------------------------------ goals and tours (present)

class FakeNav2:
    """navigate_to_pose and follow_waypoints that accept and succeed."""

    def __init__(self, r, with_follow=True):
        self.r = r
        self.goals, self.tours = [], []
        self.nav = ActionServer(
            r.helper, NavigateToPose, 'navigate_to_pose', self._nav,
            callback_group=r.cb)
        self.follow = ActionServer(
            r.helper, FollowWaypoints, 'follow_waypoints', self._follow,
            callback_group=r.cb) if with_follow else None

    def _nav(self, handle):
        self.goals.append(handle.request.pose)
        fb = NavigateToPose.Feedback()
        fb.distance_remaining = 1.5
        handle.publish_feedback(fb)
        time.sleep(0.2)
        handle.succeed()
        return NavigateToPose.Result()

    def _follow(self, handle):
        self.tours.append(list(handle.request.poses))
        for i in range(len(handle.request.poses)):
            fb = FollowWaypoints.Feedback()
            fb.current_waypoint = i
            handle.publish_feedback(fb)
            time.sleep(0.1)
        handle.succeed()
        return FollowWaypoints.Result()

    def ready(self):
        return (self.r.node.nav_to_pose.server_is_ready()
                and (self.follow is None
                     or self.r.node.follow_waypoints.server_is_ready()))

    def close(self):
        self.nav.destroy()
        if self.follow is not None:
            self.follow.destroy()


def _yaw(pose):
    q = pose.pose.orientation
    return 2.0 * math.atan2(q.z, q.w)


def test_tour_says_when_only_the_follower_is_missing():
    r = rig()
    nav = FakeNav2(r, with_follow=False)
    try:
        assert wait_until(lambda: r.node.nav_to_pose.server_is_ready())
        with pytest.raises(wp_mod.WaypointError) as err:
            r.node.tour_points([[1.0, 1.0]])
        assert 'follow_waypoints is not available' in str(err.value)
    finally:
        nav.close()


def test_goto_pose_and_tour_points_run_through_nav2():
    r = rig()
    nav2 = FakeNav2(r)
    try:
        assert wait_until(nav2.ready, 6.0)

        # ---- one drag: position + heading -> NavigateToPose
        label = r.node.goto_pose(2.0, 1.0, math.pi / 2)
        assert label == 'point (2.00, 1.00)'
        assert wait_until(lambda: any(
            e['type'] == 'result' for e in r.events), 5.0), r.events
        assert len(nav2.goals) == 1
        g = nav2.goals[0]
        assert g.header.frame_id == 'map'
        assert (g.pose.position.x, g.pose.position.y) == (2.0, 1.0)
        assert _yaw(g) == pytest.approx(math.pi / 2)
        kinds = [e['type'] for e in r.events]
        state = [e for e in r.events
                 if e['type'] == 'state' and e.get('navigating')]
        assert state and state[0]['destination'] == label
        assert any(e['type'] == 'feedback'
                   and e.get('distance_remaining') == 1.5 for e in r.events)
        res = [e for e in r.events if e['type'] == 'result'][0]
        assert (res['status'], res['destination']) == ('SUCCEEDED', label)
        assert kinds.index('state') < kinds.index('result')

        # ---- queued points -> FollowWaypoints, yaw optional
        r.events.clear()
        label = r.node.tour_points(
            [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0, math.pi]])
        assert label == 'queue of 3 points'
        assert wait_until(lambda: any(
            e['type'] == 'result' for e in r.events), 5.0), r.events
        assert len(nav2.tours) == 1
        poses = nav2.tours[0]
        assert [(p.pose.position.x, p.pose.position.y) for p in poses] == \
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        assert _yaw(poses[0]) == pytest.approx(0.0)              # faces next
        assert _yaw(poses[1]) == pytest.approx(math.pi / 2)      # faces next
        assert abs(_yaw(poses[2])) == pytest.approx(math.pi)     # explicit
        names = [e['waypoint_name'] for e in r.events
                 if e['type'] == 'feedback' and 'waypoint_name' in e]
        assert names and set(names) <= {'P1', 'P2', 'P3'}
        res = [e for e in r.events if e['type'] == 'result'][0]
        assert (res['status'], res['destination']) == \
            ('SUCCEEDED', 'queue of 3 points')

        # ---- refusals are clear and nothing is sent
        for bad, text in [([], 'queue is empty'),
                          ([[1.0]], 'must be [x, y]'),
                          ([[1.0, 'x']], 'non-numeric'),
                          ([[1.0, float('nan')]], 'non-finite'),
                          ([[0.0, 0.0]] * 51, 'at most')]:
            with pytest.raises(wp_mod.WaypointError) as err:
                r.node.tour_points(bad)
            assert text in str(err.value), (bad[:1], err.value)
        assert len(nav2.tours) == 1
        for args in [('a', 1, 0), (1, 1, float('inf'))]:
            with pytest.raises(wp_mod.WaypointError):
                r.node.goto_pose(*args)
        assert len(nav2.goals) == 1

        # ---- the same two commands through the bridge
        async def go(bridge, ws):
            await bridge.dispatch(ws, {'cmd': 'goto_pose', 'x': 0.5,
                                       'y': 0.5, 'yaw': 0.0})
            await asyncio.sleep(0.6)
            await bridge.dispatch(ws, {'cmd': 'tour_points',
                                       'points': [[0.2, 0.2], [0.4, 0.4]]})
            await asyncio.sleep(0.8)
            return ws.sent
        frames = direct(run_bridge(r, go))
        acks = [f for f in frames if f['type'] == 'ack']
        assert [a['cmd'] for a in acks] == ['goto_pose', 'tour_points']
        assert 'queue of 2 points' in acks[1]['detail']
        assert not [f for f in frames if f['type'] == 'error'], frames
    finally:
        nav2.close()


def test_single_queued_point_faces_away_from_the_robot():
    plan = NavClient.plan_queue_yaws
    assert plan([(3.0, 0.0, None)], (0.0, 0.0)) == [(3.0, 0.0, 0.0)]
    assert plan([(0.0, 3.0, None)], (0.0, 0.0))[0][2] == \
        pytest.approx(math.pi / 2)
    assert plan([(1.0, 1.0, None)], None) == [(1.0, 1.0, 0.0)]
    # last point keeps the heading it arrives on
    assert plan([(0.0, 0.0, None), (0.0, 2.0, None)])[1][2] == \
        pytest.approx(math.pi / 2)


# --------------------------------------------------------------- IMU card

def test_imu_card_data_is_honest():
    r = rig()
    node = r.node
    snap = node.imu_snapshot()
    assert snap['present'] is False and snap['fresh'] is False
    assert 'accel' not in snap and snap['stall'] == {'running': False}

    imu_qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    imu_pub = r.helper.create_publisher(Imu, '/imu', imu_qos)
    stalled_pub = r.helper.create_publisher(Bool, '/motion_check/moved', latched)
    status_pub = r.helper.create_publisher(String, '/motion_check/status', latched)
    calib_pub = r.helper.create_publisher(String, '/imu/status', 10)
    try:
        assert wait_until(lambda: imu_pub.get_subscription_count() >= 1)
        stalled_pub.publish(Bool(data=True))
        status_pub.publish(String(data='STALL gap 0.00 m/s2'))
        calib_pub.publish(String(data='sys 3 gyr 3 acc 3 mag 1'))

        def sample(cov0, yaw=0.0, ax=0.1):
            m = Imu()
            m.linear_acceleration.x = ax
            m.linear_acceleration.y = 0.2
            m.linear_acceleration.z = 9.81
            m.angular_velocity.z = 0.05
            m.orientation.z = math.sin(yaw / 2)
            m.orientation.w = math.cos(yaw / 2)
            m.orientation_covariance[0] = cov0
            return m

        # 50 Hz for 2 s, accel-only: far fewer events than samples
        end = time.time() + 2.0
        while time.time() < end:
            imu_pub.publish(sample(-1.0))
            time.sleep(0.02)
        imu_events = [e for e in r.events if e['type'] == 'imu']
        assert 2 <= len(imu_events) <= 8, len(imu_events)       # ~2 Hz, not 50
        snap = node.imu_snapshot()
        assert snap['fresh'] is True
        assert snap['accel'] == pytest.approx([0.1, 0.2, 9.81])
        assert snap['gyro_z'] == pytest.approx(0.05)
        assert snap['orientation_valid'] is False
        assert 'rpy' not in snap, 'accel-only must not invent roll/pitch/yaw'
        assert snap['stall']['running'] is True
        assert snap['stall']['stalled'] is True
        assert snap['stall']['status'] == 'STALL gap 0.00 m/s2'
        assert snap['calib'].startswith('sys 3')

        # fused orientation, valid: yaw 90 deg
        end = time.time() + 1.0
        while time.time() < end:
            imu_pub.publish(sample(0.01, yaw=math.pi / 2))
            time.sleep(0.02)
        snap = node.imu_snapshot()
        assert snap['orientation_valid'] is True
        assert snap['rpy'][2] == pytest.approx(90.0, abs=0.5)

        # NaN must not reach the browser as NaN
        imu_pub.publish(sample(-1.0, ax=float('nan')))
        assert wait_until(lambda: node.imu_snapshot()['accel'][0] is None, 2.0)
        text = dumps(node.imu_event())
        assert 'NaN' not in text

        # silence: no new samples. Within a few seconds a stale event goes out
        # and the numbers are gone from the snapshot.
        r.events.clear()
        assert wait_until(lambda: any(
            e['type'] == 'imu' and e['fresh'] is False
            for e in r.events), 5.0), 'no stale event'
        stale = [e for e in r.events if e['type'] == 'imu'][-1]
        assert stale['present'] is True and 'accel' not in stale
        assert node.imu_snapshot()['fresh'] is False

        # stall_guard dies: its latched verdict must not outlive it
        r.helper.destroy_publisher(stalled_pub)
        r.helper.destroy_publisher(status_pub)
        assert wait_until(
            lambda: node.imu_snapshot()['stall'] == {'running': False}, 5.0)
    finally:
        r.helper.destroy_publisher(imu_pub)
        r.helper.destroy_publisher(calib_pub)



def test_speed_card_reports_measured_commanded_and_obstacle_guard():
    from geometry_msgs.msg import Twist
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import Odometry
    r = rig()
    node = r.node
    odom_pub = r.helper.create_publisher(Odometry, '/odom', 10)
    cmd_pub = r.helper.create_publisher(Twist, '/cmd_vel', 10)
    guard_pub = r.helper.create_publisher(
        CollisionMonitorState, '/collision_monitor_state', 10)
    try:
        # nothing yet: not fresh, no numbers invented
        snap = node.speed_snapshot()
        assert snap['fresh'] is False and snap['v'] is None

        odom = Odometry()
        odom.twist.twist.linear.x = 0.21
        odom.twist.twist.angular.z = -0.5
        cmd = Twist()
        cmd.linear.x = 0.25
        state = CollisionMonitorState()
        state.action_type = CollisionMonitorState.APPROACH
        state.polygon_name = 'FootprintApproach'

        def feed():
            odom_pub.publish(odom)
            cmd_pub.publish(cmd)
            guard_pub.publish(state)
            s = node.speed_snapshot()
            return s['fresh'] and s['cmd_v'] > 0 and s['guard']
        assert wait_until(feed, 5.0), node.speed_snapshot()
        snap = node.speed_snapshot()
        assert snap['v'] == pytest.approx(0.21)
        assert snap['w'] == pytest.approx(-0.5)
        assert snap['cmd_v'] == pytest.approx(0.25)
        assert snap['guard'] == 'approach'
        assert snap['max_v'] > 0
        # commanded split: 0.25 m/s straight -> both wheels 0.25
        assert snap['cmd_left'] == pytest.approx(0.25)
        assert snap['cmd_right'] == pytest.approx(0.25)

        # per-wheel speed from raw encoder counts: left 0.20 m/s, right
        # 0.10 m/s in reverse (a turn: the right wheel is going backwards)
        from geometry_msgs.msg import Vector3
        counts_pub = r.helper.create_publisher(Vector3, '/enco/counts', 10)
        try:
            per_m = 662.0 / (2 * math.pi * 0.0335)
            t0 = time.monotonic()

            def wheels_ok():
                dt = time.monotonic() - t0
                counts_pub.publish(Vector3(x=0.20 * dt * per_m,
                                           y=-0.10 * dt * per_m))
                time.sleep(0.05)
                s = node.speed_snapshot()
                return (s['left'] is not None and dt > 0.6
                        and abs(s['left'] - 0.20) < 0.02
                        and abs(s['right'] + 0.10) < 0.02)
            assert wait_until(wheels_ok, 5.0), node.speed_snapshot()
            # ESP32 reboot: counters restart at zero - no huge spike
            counts_pub.publish(Vector3(x=0.0, y=0.0))
            time.sleep(0.1)
            s = node.speed_snapshot()
            assert s['left'] is None or abs(s['left']) < 1.0
        finally:
            r.helper.destroy_publisher(counts_pub)

        # the timer pushes it to the page as a 'speed' event
        assert wait_until(
            lambda: any(e['type'] == 'speed' for e in r.events), 2.0)
        assert 'NaN' not in dumps([e for e in r.events if e['type'] == 'speed'][-1])

        # silence: a stale command reads as zero, stale odom as no number
        assert wait_until(lambda: node.speed_snapshot()['fresh'] is False, 3.0)
        assert node.speed_snapshot()['cmd_v'] == 0.0
        assert node.speed_snapshot()['v'] is None

        # the monitor goes away: its last action must not outlive it
        r.helper.destroy_publisher(guard_pub)
        guard_pub = None
        assert wait_until(lambda: node.speed_snapshot()['guard'] == '', 5.0)
    finally:
        r.helper.destroy_publisher(odom_pub)
        r.helper.destroy_publisher(cmd_pub)
        if guard_pub is not None:
            r.helper.destroy_publisher(guard_pub)



def test_wheel_speed_falls_back_to_joint_states_without_counts():
    from sensor_msgs.msg import JointState
    r = rig()
    node = r.node
    # counts from an earlier test must have gone stale first
    assert wait_until(lambda: node._wheels is None
                      or time.monotonic() - node._wheels[2] > 1.0, 3.0)
    pub = r.helper.create_publisher(JointState, '/joint_states', 10)
    try:
        msg = JointState()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.velocity = [0.15 / 0.0335, -0.05 / 0.0335]       # rad/s

        def ok():
            pub.publish(msg)
            s = node.speed_snapshot()
            return s['wheels_source'] == 'joint_states'
        assert wait_until(ok, 5.0), node.speed_snapshot()
        s = node.speed_snapshot()
        assert s['left'] == pytest.approx(0.15)
        assert s['right'] == pytest.approx(-0.05)
        # a JointState without the wheel joints is ignored, not a crash
        pub.publish(JointState(name=['other'], velocity=[1.0]))
    finally:
        r.helper.destroy_publisher(pub)


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-q']))
