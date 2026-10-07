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
The pre-exploration localization gate: localize FIRST, then drive.

`gate_localization` and `localization_status` are lifted out of
`frontier_explorer.py` by AST and bound to a plain rig, so nothing needs a
ROS context. The rig fakes exactly the two things the gate reads - a TF
buffer that may raise, and a transform whose header stamp has an age - and
records every Twist the gate publishes.

What is being pinned down here is the thing a fixed `TimerAction(8.0)` could
never promise: no wheels turn until there really is a pose, a stale cached
transform does not count as one, the wait is never silent, and it ends in a
loud failure rather than a blind drive.
"""

import ast
import os
import types

SOURCE = os.path.join(
    os.path.dirname(__file__), '..', 'nexva_explore', 'frontier_explorer.py')

LIFTED = {'localization_status', 'gate_localization', 'reported_mode',
          'unpause_clocks'}


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)


class Clock:
    t = 1000.0

    @classmethod
    def monotonic(cls):
        return cls.t


class TransformException(Exception):
    pass


class Time:
    """Stands in for rclpy.time.Time: Time() is 'latest', from_msg unwraps."""

    def __init__(self):
        self.latest = True

    @staticmethod
    def from_msg(stamp):
        return stamp


def _lift():
    tree = ast.parse(open(SOURCE).read())
    node = next(n for n in tree.body
                if isinstance(n, ast.ClassDef) and n.name == 'FrontierExplorer')
    kept = [n for n in node.body
            if isinstance(n, ast.FunctionDef) and n.name in LIFTED]
    assert {n.name for n in kept} == LIFTED, LIFTED - {n.name for n in kept}
    holder = ast.ClassDef(name='Gate', bases=[], keywords=[], body=kept,
                          decorator_list=[], type_params=[])
    module = ast.fix_missing_locations(
        ast.Module(body=[holder], type_ignores=[]))
    scope = {
        'time': Clock,
        'Twist': Twist,
        'Time': Time,
        'tf2_ros': types.SimpleNamespace(TransformException=TransformException),
    }
    exec(compile(module, SOURCE, 'exec'), scope)
    return scope['Gate']


Gate = _lift()


class Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append((msg.linear.x, msg.angular.z))


class Log:
    def __init__(self):
        self.lines = {'info': [], 'warn': [], 'error': []}

    def _add(self, level):
        def record(text, **kwargs):
            self.lines[level].append(text)
        return record

    def info(self, text, **kwargs):
        self._add('info')(text, **kwargs)

    def warn(self, text, **kwargs):
        self._add('warn')(text, **kwargs)

    def error(self, text, **kwargs):
        self._add('error')(text, **kwargs)

    def all(self):
        return self.lines['info'] + self.lines['warn'] + self.lines['error']


class Buffer:
    """TF buffer that either raises or hands back a stamped transform."""

    def __init__(self):
        self.stamp = None          # None -> no transform at all

    def lookup_transform(self, target, source, when):
        if self.stamp is None:
            raise TransformException(f'{target} -> {source} does not exist')
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=self.stamp))


class Stamp:
    """A ROS time: subtracting two gives something with `.nanoseconds`."""

    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        other = other.seconds if isinstance(other, Stamp) else other
        return types.SimpleNamespace(
            nanoseconds=(self.seconds - other) * 1e9)


class NowClock:
    """Stands in for the node clock: `now()` is a Stamp."""

    def __init__(self):
        self.seconds = 1000.0

    def now(self):
        return Stamp(self.seconds)


class Grid:
    info = types.SimpleNamespace(width=200, height=200)


class Bot(Gate):
    """The gate's world: a TF buffer, a /map slot and a cmd_vel publisher."""

    def __init__(self, **over):
        Clock.t = 1000.0
        self.tf_buffer = Buffer()
        self.ros_clock = NowClock()
        self.map_msg = None
        self.map_frame = 'map'
        self.base_frame = 'base_footprint'
        self.pose_timeout = 2.0
        self.localize_timeout = 60.0
        self.localized = False
        self.localize_started = Clock.t
        self.mode = 'explore'
        self.paused = False
        self.recovery_until = 0.0
        self.handoff = None
        self.progress_time = Clock.t
        self.target_since = Clock.t
        self.target_gain_time = Clock.t
        self.cmd_pub = Pub()
        self.log = Log()
        self.mode_published = []
        self.__dict__.update(over)

    def get_logger(self):
        return self.log

    def get_clock(self):
        return self.ros_clock

    def publish_mode(self):
        self.mode_published.append(self.reported_mode())

    # -- helpers --------------------------------------------------------
    def slam_up(self, age=0.0):
        """SLAM publishes a transform stamped `age` seconds ago, and a map."""
        self.tf_buffer.stamp = self.ros_clock.seconds - age
        self.map_msg = Grid()

    def tick(self, seconds=0.1):
        """One 10 Hz follow_path tick's worth of gate; returns `ready`."""
        ready = self.gate_localization()
        Clock.t += seconds
        self.ros_clock.seconds += seconds
        # A live publisher re-stamps; a dead one does not, which is what
        # makes the stamp age grow and the transform go stale.
        if self.tf_buffer.stamp is not None and getattr(self, 'live', True):
            self.tf_buffer.stamp = self.ros_clock.seconds
        return ready

    def run(self, seconds):
        """Tick for `seconds`; returns when the gate opened, or None."""
        opened = None
        ticks = int(round(seconds / 0.1))
        for i in range(ticks):
            if self.tick() and opened is None:
                opened = i * 0.1
        return opened


# ----------------------------------------------------------------------
# (a) No transform: it waits, and it waits with the wheels at zero.
# ----------------------------------------------------------------------

def test_no_transform_never_opens_the_gate():
    bot = Bot()
    assert bot.run(10.0) is None
    assert bot.localized is False


def test_while_waiting_it_publishes_zero_twist_every_single_tick():
    bot = Bot()
    bot.run(3.0)
    assert len(bot.cmd_pub.sent) == 30, 'a silent cmd_vel looks like a crash'
    assert set(bot.cmd_pub.sent) == {(0.0, 0.0)}


def test_waiting_says_which_condition_is_missing():
    bot = Bot()
    bot.run(1.0)
    assert any('map -> base_footprint' in line for line in bot.log.all())

    bot = Bot()
    bot.tf_buffer.stamp = bot.ros_clock.seconds       # TF but no /map yet
    bot.run(1.0)
    assert any('/map' in line for line in bot.log.all())
    assert bot.localized is False


def test_tf_without_a_map_is_not_localized():
    bot = Bot()
    bot.tf_buffer.stamp = bot.ros_clock.seconds
    assert bot.run(5.0) is None


def test_reported_mode_is_localizing_while_it_holds():
    bot = Bot()
    bot.run(1.0)
    assert bot.reported_mode() == 'localizing'


# ----------------------------------------------------------------------
# (b) Transform appears: it proceeds, promptly.
# ----------------------------------------------------------------------

def test_opens_within_one_tick_of_the_transform_appearing():
    bot = Bot()
    bot.run(2.0)
    assert bot.localized is False
    bot.slam_up()
    assert bot.tick() is True
    assert bot.localized is True


def test_once_open_it_stays_open_and_costs_nothing():
    bot = Bot()
    bot.slam_up()
    assert bot.tick() is True
    before = len(bot.cmd_pub.sent)
    bot.tf_buffer.stamp = None            # SLAM dies after the gate opened
    assert bot.tick() is True, 'the gate is a start condition, not a watchdog'
    assert len(bot.cmd_pub.sent) == before, 'the gate stops publishing zeros'


def test_opening_does_not_leave_the_wedge_timer_already_expired():
    """A long localization must not read as 'no progress for a long time'."""
    bot = Bot()
    bot.run(30.0)
    bot.slam_up()
    bot.tick()
    assert bot.progress_time >= Clock.t - 0.2
    assert bot.target_since >= Clock.t - 0.2


def test_opening_is_logged_with_how_long_it_took():
    bot = Bot()
    bot.run(5.0)
    bot.slam_up()
    bot.tick()
    assert any('Localized after' in line for line in bot.log.lines['info'])


def test_reported_mode_returns_to_explore_once_localized():
    bot = Bot()
    bot.slam_up()
    bot.tick()
    assert bot.reported_mode() == 'explore'


# ----------------------------------------------------------------------
# (c) Timeout: a loud failure, never a blind drive.
# ----------------------------------------------------------------------

def test_times_out_and_stops_instead_of_driving():
    bot = Bot(localize_timeout=5.0)
    assert bot.run(8.0) is None
    assert bot.mode == 'stopped'
    assert bot.localized is False


def test_timeout_message_names_the_missing_condition_and_what_to_do():
    bot = Bot(localize_timeout=5.0)
    bot.run(8.0)
    errors = ' '.join(bot.log.lines['error'])
    assert 'NOT LOCALIZED' in errors
    assert 'map -> base_footprint' in errors
    assert 'slam_toolbox' in errors


def test_the_wheels_stay_at_zero_right_through_the_timeout():
    bot = Bot(localize_timeout=2.0)
    bot.run(6.0)
    assert set(bot.cmd_pub.sent) == {(0.0, 0.0)}


# ----------------------------------------------------------------------
# (d) Staleness: a cached transform is not a pose.
# ----------------------------------------------------------------------

def test_a_stale_transform_does_not_count_as_localized():
    """
    The failure a `can_transform` check cannot see.

    A lookup at Time() returns the newest transform in the buffer however
    old it is, so slam_toolbox dying leaves a transform that resolves
    forever. Only its stamp age gives it away.
    """
    bot = Bot()
    bot.live = False                       # publisher dead: stamp never moves
    bot.tf_buffer.stamp = bot.ros_clock.seconds - 5.0
    bot.map_msg = Grid()
    assert bot.run(5.0) is None
    assert bot.localized is False


def test_stale_is_judged_against_pose_timeout_exactly():
    fresh = Bot()
    fresh.live = False
    fresh.tf_buffer.stamp = fresh.ros_clock.seconds - 1.9
    fresh.map_msg = Grid()
    assert fresh.tick() is True

    stale = Bot()
    stale.live = False
    stale.tf_buffer.stamp = stale.ros_clock.seconds - 2.1
    stale.map_msg = Grid()
    assert stale.tick() is False


def test_a_transform_that_goes_stale_mid_wait_is_still_not_localized():
    bot = Bot()
    bot.live = False
    bot.map_msg = Grid()
    bot.tf_buffer.stamp = bot.ros_clock.seconds        # fresh, then ages
    bot.run(0.5)
    assert bot.localized is True, 'it was fresh at the first tick'

    other = Bot()
    other.live = False
    other.map_msg = Grid()
    other.tf_buffer.stamp = other.ros_clock.seconds - 3.0
    other.run(1.0)
    assert other.localized is False
    assert any('stale' in line for line in other.log.all())


def test_staleness_reason_quotes_the_age_and_the_limit():
    bot = Bot()
    bot.live = False
    bot.tf_buffer.stamp = bot.ros_clock.seconds - 7.0
    bot.map_msg = Grid()
    ready, reason = bot.localization_status()
    assert ready is False
    assert '7.0 s' in reason and '2.0 s' in reason
