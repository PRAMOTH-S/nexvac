# Copyright 2026 vac_main1
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The stop / aim / go recovery, driven closed-loop against synthetic rooms.

The recovery methods are lifted out of `frontier_explorer.py` by AST and bound
to a plain rig, so nothing needs a ROS context. The rig is a 10 Hz kinematic
robot in a world of fixed points: every tick the scan is recomputed from the
pose, the same `drive_recovery` -> `publish_safe` pair `follow_path` runs is
called, and the published Twist moves the robot. That is what lets a test say
"it rotated, THEN drove", not just "it picked a branch".

Throughout, three safety invariants are checked on EVERY published command:
no forward motion at or inside `stop_distance`, no rotation inside the swept
circle, and nothing but zero while the lidar is stale.
"""

import ast
import math
import os
import types

SOURCE = os.path.join(
    os.path.dirname(__file__), '..', 'nexva_explore', 'frontier_explorer.py')

LIFTED = {
    'begin_recovery', 'enter_recovery_phase', 'end_recovery',
    'heading_clearance', 'way_is_clear', 'is_clear_ahead', 'aim_error',
    'start_rotation', 'start_go', 'start_fallback', 'choose_after_aim',
    'drive_recovery', 'drive_recovery_fallback', 'forward_clearance',
    'rear_clearance', 'can_turn', 'rotation_clearance', 'escape_heading',
    'sector_is_clear', 'side_clearance', 'nearest_obstacle', 'avoid_steer',
    'back_out', 'backing_out', 'publish_safe',
}

DT = 0.1


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)


class Clock:
    t = 1000.0

    @classmethod
    def monotonic(cls):
        return cls.t


def _lift():
    tree = ast.parse(open(SOURCE).read())
    node = next(n for n in tree.body
                if isinstance(n, ast.ClassDef) and n.name == 'FrontierExplorer')
    kept = [n for n in node.body
            if isinstance(n, ast.FunctionDef) and n.name in LIFTED]
    assert {n.name for n in kept} == LIFTED, LIFTED - {n.name for n in kept}
    holder = ast.ClassDef(name='Rec', bases=[], keywords=[], body=kept,
                          decorator_list=[], type_params=[])
    module = ast.fix_missing_locations(
        ast.Module(body=[holder], type_ignores=[]))
    scope = {'math': math, 'time': Clock, 'Twist': Twist}
    exec(compile(module, SOURCE, 'exec'), scope)
    return scope['Rec']


Rec = _lift()


class Log:
    def warn(self, *a, **k):
        pass

    error = info = warn


class Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append((msg.linear.x, msg.angular.z))


class Bot(Rec):
    """Kinematic robot at (x, y, yaw) in a world of fixed points."""

    def __init__(self, world, pose=(0.0, 0.0, 0.0), path=None, **over):
        Clock.t = 1000.0
        self.world = world
        self.x, self.y, self.yaw = pose
        self.path = list(path or [])
        self.stale = False
        self.cmd_pub = Pub()
        self.log = []
        self.fallback_ticks = 0

        # Defaults of frontier_explorer.py / config/explore.yaml.
        self.half_length = 0.15
        self.half_width = 0.15
        self.turn_radius = math.hypot(0.15, 0.15)
        self.turn_margin = 0.03
        self.stop_distance = 0.18
        self.slow_distance = 0.45
        self.linear_speed = 0.15
        self.angular_speed = 0.5
        self.max_linear = 0.26
        self.max_angular = 1.5
        self.avoid_turn = 0.45
        self.allow_blind_reverse = False
        self.use_imu_stall = False
        self.watch = types.SimpleNamespace(stalled=False)
        self.gated_ticks = 0
        self.gated_limit = 10
        self.forced_reverses = 0
        self.max_forced_reverse = 5
        self.mode = 'explore'
        self.recovery_time = 2.5
        self.backout_time = 1.5
        self.backout_max = 8.0
        self.backout_until = 0.0
        self.backout_started = 0.0
        self.max_recovery_attempts = 3
        self.recovery_attempts = 0
        self.recovery_settle = 0.3
        self.aim_tolerance = 0.15
        self.aim_timeout = 7.0
        self.recovery_go_time = 1.0
        self.recovery_budget = 9.0
        self.recovery_until = 0.0
        self.recovery_phase = None
        self.recovery_phase_until = 0.0
        self.recovery_target = None
        self.recovery_started = 0.0
        self.recovery_turned = False
        self.progress_pose = None
        self.progress_time = 0.0
        self.abandoned = 0
        for key, value in over.items():
            setattr(self, key, value)
        self.scan()

    def get_logger(self):
        return Log()

    def abandon_goal(self):
        self.abandoned += 1
        self.path = []
        self.recovery_until = 0.0
        self.recovery_phase = None

    def scan_is_stale(self):
        return self.stale

    @property
    def last_pose(self):
        return (self.x, self.y, self.yaw)

    def scan(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        pts = []
        for wx, wy in self.world:
            dx, dy = wx - self.x, wy - self.y
            rx, ry = dx * c + dy * s, -dx * s + dy * c
            pts.append((rx, ry, math.hypot(rx, ry)))
        self.scan_points = pts

    def tick(self):
        """One 10 Hz follow_path tick during a recovery; returns the Twist."""
        self.scan()
        cmd = Twist()
        before = len(self.cmd_pub.sent)
        fwd_before = self.forward_clearance()
        can_turn_before = self.can_turn()
        if Clock.t < self.recovery_until:
            self.drive_recovery(cmd)
            self.publish_safe(cmd)
        assert len(self.cmd_pub.sent) <= before + 1
        sent = self.cmd_pub.sent[-1] if len(self.cmd_pub.sent) > before else (0.0, 0.0)

        # Safety invariants, on every published command.
        if sent[0] > 0.0:
            assert fwd_before > self.stop_distance, 'drove into stop_distance'
        if abs(sent[1]) > 1e-9 and abs(sent[0]) < 1e-9:
            assert can_turn_before, 'spun inside the turning circle'
        if self.stale:
            assert sent == (0.0, 0.0), 'moved on a stale lidar'
        assert abs(sent[0]) <= self.max_linear + 1e-9
        assert abs(sent[1]) <= self.max_angular + 1e-9

        self.x += sent[0] * math.cos(self.yaw) * DT
        self.y += sent[0] * math.sin(self.yaw) * DT
        self.yaw += sent[1] * DT
        Clock.t += DT
        if self.recovery_phase == 'fallback':
            self.fallback_ticks += 1
        return sent

    def run(self, seconds=20.0):
        """Tick until the recovery ends; return [(t, phase, v, w), ...]."""
        trace = []
        t0 = Clock.t
        while Clock.t - t0 < seconds and (
                Clock.t < self.recovery_until or self.recovery_phase):
            v, w = self.tick()
            # The phase that produced this command: hand-overs happen inside
            # the tick, so it is the phase AFTER it (None once finished).
            trace.append((round(Clock.t - t0, 2), self.recovery_phase or 'end',
                          v, w))
        return trace


def wall(x0, y0, x1, y1, step=0.02):
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def room(half=3.0):
    return (wall(-half, -half, half, -half) + wall(half, -half, half, half)
            + wall(half, half, -half, half) + wall(-half, half, -half, -half))


def start(bot):
    bot.begin_recovery(bot.x, bot.y)


def angle_to(bot, point):
    return math.atan2(point[1] - bot.y, point[0] - bot.x)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ----------------------------------------------------------------------
# The sequence
# ----------------------------------------------------------------------

def test_stop_settles_first_with_zero_twist():
    bot = Bot(room(), path=[(2.0, 0.0)])
    start(bot)
    assert bot.recovery_phase == 'stop'
    trace = bot.run()
    stop = [row for row in trace if row[1] == 'stop']
    assert len(stop) == 3                       # 0.3 s at 10 Hz
    assert all(v == 0.0 and w == 0.0 for _, _, v, w in stop)


def test_goal_ahead_and_clear_goes_straight_away_no_rotation_no_reverse():
    bot = Bot(room(), path=[(2.0, 0.0)])
    start(bot)
    trace = bot.run()
    assert {r[1] for r in trace} == {'stop', 'go', 'end'}
    assert sum(1 for r in trace if r[1] == 'go') >= 9
    assert all(w == 0.0 for *_, w in trace)
    assert all(v >= 0.0 for _, _, v, _ in trace)
    assert trace[-1][0] <= 0.3 + 1.0 + DT + 1e-9   # stop + go, ~1.3 s
    assert bot.x > 0.10
    assert bot.recovery_phase is None and bot.recovery_until == 0.0


def test_aims_at_goal_then_goes():
    # Goal 90 degrees to the left, open room: spin to face it, then drive.
    bot = Bot(room(), path=[(0.0, 2.0)])
    start(bot)
    trace = bot.run()
    phases = [r[1] for r in trace]
    assert phases.index('aim') < phases.index('go')
    assert abs(wrap(angle_to(bot, (0.0, 2.0)) - bot.yaw)) <= bot.aim_tolerance + 0.06
    spins = [r for r in trace if r[1] == 'aim']
    assert all(w == bot.angular_speed and v == 0.0 for _, _, v, w in spins)
    assert all(v >= 0.0 for _, _, v, _ in trace)           # never reverses
    # 0.3 settle + 1.57 rad / 0.5 rad/s + 1.0 go, give or take a tick
    assert trace[-1][0] <= 0.3 + (math.pi / 2) / 0.5 + 1.0 + 0.4


def test_aim_turns_the_short_way_round():
    bot = Bot(room(), path=[(0.0, -2.0)])
    start(bot)
    trace = bot.run()
    assert all(w <= 0.0 for *_, w in trace)


def test_goal_blocked_ahead_turns_to_clear_heading_then_goes():
    # Wall 0.30 m ahead, the way to the goal is through it; the room is open
    # to the left only (a wall on the right close alongside).
    world = wall(0.45, -1.0, 0.45, 1.0) + wall(-1.0, -0.4, 1.0, -0.4) \
        + wall(-1.0, 0.9, 0.0, 0.9)
    bot = Bot(world, pose=(0.15, 0.0, 0.0), path=[(2.0, 0.0)])
    assert bot.forward_clearance() < bot.slow_distance
    start(bot)
    trace = bot.run()
    phases = [r[1] for r in trace]
    assert 'turn' in phases
    assert 'aim' not in phases                 # no spinning to face a wall
    assert phases.index('turn') < phases.index('go')
    assert 'fallback' not in phases
    assert all(v >= 0.0 for _, _, v, _ in trace)           # never reverses


def test_blind_back_out_only_when_nothing_clear_and_cannot_turn():
    # Boxed in on all sides inside the turning circle: no heading, no turn.
    world = [(0.25 * math.cos(a / 10), 0.25 * math.sin(a / 10))
             for a in range(63)]
    bot = Bot(world, path=[(2.0, 0.0)])
    start(bot)
    trace = bot.run()
    phases = [r[1] for r in trace]
    assert phases[:3] == ['stop'] * 3
    assert 'fallback' in phases
    assert 'aim' not in phases and 'turn' not in phases and 'go' not in phases


def test_cannot_turn_but_forward_clear_goes_without_rotating():
    # A post inside the turning circle off to the side: spinning is refused,
    # but straight ahead is open.
    bot = Bot([(0.0, 0.21)] + wall(3.0, -1, 3.0, 1), path=[(0.0, 2.0)])
    assert not bot.can_turn()
    start(bot)
    trace = bot.run()
    phases = [r[1] for r in trace]
    assert 'aim' not in phases and 'fallback' not in phases
    assert 'go' in phases
    assert all(w == 0.0 for *_, w in trace)


def test_no_path_goes_if_forward_clear_else_turns():
    bot = Bot(room())
    start(bot)
    assert [r[1] for r in bot.run() if r[1] != 'stop'][0] == 'go'

    world = wall(0.40, -1.0, 0.40, 1.0)
    bot = Bot(world, pose=(0.15, 0.0, 0.0))
    start(bot)
    phases = [r[1] for r in bot.run()]
    assert 'turn' in phases or 'fallback' in phases


def test_go_ends_early_when_the_way_closes():
    # Clear enough to commit to (0.50 m gap > slow_distance) but the wall is
    # inside go-distance: GO must stop short of stop_distance, not at the wall.
    bot = Bot(wall(0.65, -1, 0.65, 1), path=[(2.0, 0.0)])
    start(bot)
    bot.run()
    assert bot.forward_clearance() > bot.stop_distance
    assert bot.recovery_phase is None


# ----------------------------------------------------------------------
# Timing
# ----------------------------------------------------------------------

def test_half_turn_aim_fits_the_timeout_and_budget():
    bot = Bot(room(), path=[(-2.0, 0.0)])
    start(bot)
    trace = bot.run()
    # 0.3 settle + 6.3 s rotation + 1.0 go, one tick of slack each hand-over
    assert trace[-1][0] <= 0.3 + math.pi / 0.5 + 1.0 + 0.4
    assert trace[-1][0] <= bot.recovery_budget + bot.recovery_go_time


def test_rotation_that_does_not_complete_times_out_and_decides():
    # Wheels slipping: the rotation never changes yaw. Must give up after
    # aim_timeout, not spin forever.
    class Slip(Bot):
        def tick(self):
            yaw = self.yaw
            out = super().tick()
            self.yaw = yaw
            return out

    bot = Slip(room(), path=[(-2.0, 0.0)])
    start(bot)
    trace = bot.run(seconds=30.0)
    aim_time = sum(DT for r in trace if r[1] == 'aim')
    assert aim_time <= bot.aim_timeout + 2 * DT
    assert trace[-1][0] < 30.0 - 1.0


def test_budget_clips_the_second_rotation():
    bot = Bot(room(), path=[(0.0, 2.0)], recovery_budget=2.0)
    start(bot)
    bot.run()
    # budget 2.0 s: stop 0.3 then at most 1.7 s of rotation (>= 0.5 s floor)
    assert Clock.t - bot.recovery_started <= 2.0 + 1.0 + 0.5


# ----------------------------------------------------------------------
# Safety gates and escalation
# ----------------------------------------------------------------------

def test_stale_lidar_publishes_nothing_but_zero():
    bot = Bot(room(), path=[(2.0, 0.0)])
    start(bot)
    bot.stale = True
    trace = bot.run()
    assert all(v == 0.0 and w == 0.0 for _, _, v, w in trace)


def test_imu_pinned_is_not_clear_so_forward_is_never_chosen_blind():
    bot = Bot(room(), path=[(2.0, 0.0)], use_imu_stall=True)
    bot.watch.stalled = True
    start(bot)
    phases = [r[1] for r in bot.run()]
    assert 'go' not in phases


def test_rear_gate_still_refuses_fallback_reverse_into_a_wall():
    # Boxed in with a wall right behind: fallback asks to reverse, the
    # (unchanged) rear gate must still stop it.
    world = [(0.25 * math.cos(a / 10), 0.25 * math.sin(a / 10))
             for a in range(63)]
    bot = Bot(world, path=[(2.0, 0.0)])
    start(bot)
    trace = bot.run()
    assert all(v >= -1e-9 for _, _, v, _ in trace)


def test_escalation_counts_attempts_and_abandons_after_the_cap():
    bot = Bot(room(), path=[(2.0, 0.0)])
    for attempt in range(1, bot.max_recovery_attempts + 1):
        start(bot)
        assert bot.recovery_attempts == attempt
        assert bot.recovery_phase == 'stop'
        bot.end_recovery()
    assert bot.abandoned == 0
    start(bot)                                    # one over the cap
    assert bot.abandoned == 1
    assert bot.recovery_attempts == 0
    assert bot.recovery_phase is None and bot.recovery_until == 0.0


def test_fallback_is_the_unchanged_bounded_backout():
    # Boxed in, but with the back clear of the swept circle: reverse is fine
    world = wall(-1.0, -1.0, -1.0, 1.0) + wall(-1, -0.2, 1.0, -0.2)
    bot = Bot(world)
    bot.recovery_phase = 'fallback'
    bot.recovery_phase_until = Clock.t + bot.recovery_time
    bot.recovery_until = bot.recovery_phase_until + 0.15
    trace = bot.run()
    assert trace[-1][0] <= bot.recovery_time + 0.3
    assert bot.recovery_phase is None
