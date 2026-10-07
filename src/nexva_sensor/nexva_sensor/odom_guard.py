"""
Stop slipping wheels from inventing motion, in the odometry itself.

THE FAILURE THIS EXISTS FOR
`nexva_frimware/wheel_odometry.py` integrates encoder counts into `/odom` and
into the `odom -> base_footprint` transform. The encoders are honest about the
WHEELS and say nothing about the ROBOT, so when the robot is pinned against a
chair leg with its motors still turning, the pose walks forward anyway. SLAM
inserts its scans at a pose the robot never reached, AMCL localises against the
smear that results, the coverage planner ticks off floor it never touched, and
the web UI shows the robot driving through a wall. That is exactly the user's
report: "the accelerometer is below 1 m/s^2 but the bot still says it moved".

`stall_guard` already catches the physics of this (see its module docstring:
it cross-examines the change in wheel speed against the change in body speed
the accelerometer actually felt). What it did NOT do is correct the odometry -
it raised the motor floor and stopped commanding forward motion, but `/odom`
kept lying the whole time. This node closes that gap.

DETECTOR / ACTUATOR SPLIT
  odom_guard (here)        DETECTOR. Fuses stall_guard's verdict with IMU
                           freshness and the wheels' own claim, applies its
                           own hysteresis, and publishes one simple boolean,
                           `/wheel_slip`, at the odometry rate. It also
                           publishes `/odom_guarded`, a corrected copy of
                           `/odom`, for anything that wants to consume the
                           correction without changing its `/odom` wiring.

  wheel_odometry (there)   ACTUATOR. Subscribes `/wheel_slip` behind its
                           `trust_imu` parameter and, while the flag is true
                           AND fresh, stops accumulating forward distance and
                           reports zero linear velocity - at the source, in
                           the `/odom` topic AND the TF that everything
                           downstream actually reads.

The split exists because the detector needs the IMU, lives in nexva_sensor
beside the IMU driver and the stall guard, and can be killed or disabled
without touching odometry; the actuator must live in the node that owns the
transform. Two nodes publishing `odom -> base_footprint` would fight, so this
node deliberately publishes NO TF. `/odom_guarded` is advisory: nothing is
forced to use it, and the authoritative correction happens in wheel_odometry.
(The simulation original, vaccum/odom_guard.py, made the same choice for the
same reason, with its `publish_tf` parameter off.)

WHY IT CANNOT SUPPRESS ODOMETRY WITHOUT AN IMU
Three independent gates, all of which must pass before `/wheel_slip` can ever
go true:

  1. `stall_guard` must say so. Without an `imu` topic it gives NO verdict at
     all - its own gates stand it down and it reports "NO IMU" on
     `stall_guard/status` - so `stall_guard/stalled` stays false for ever.
  2. This node must have seen an IMU message within `imu_timeout`. A driver
     that dies mid-run, or a latched stale `stall_guard/stalled` from before
     the IMU was unplugged, is therefore not enough on its own.
  3. The wheels must be claiming at least `min_wheel_speed`. There is no such
     thing as a slipping wheel that is not turning.

And on the actuator side, `wheel_odometry` suppresses only while `/wheel_slip`
is both TRUE and FRESH. If this node is not running, is killed, or stops
publishing, the flag goes stale and the odometry reverts to today's behaviour
within `slip_timeout`. No IMU, or no odom_guard, means the robot behaves
exactly as it does today.

CONFIRMED VERSUS SUSPECTED
`stall_guard`'s `confirm` window is 0.6 s, chosen so that a single odd
accelerometer sample cannot cut motor power. At 0.15 m/s that is 9 cm of
phantom travel already baked into the map before the flag rises. For gating
ODOMETRY that is needlessly slow, because suppressing odometry is cheap and
reversible: a false suppression costs a few centimetres of real motion that
SLAM's scan matching puts straight back, whereas a false power cut stops a
clean. So this node also accepts `stall_guard/suspect`, which rises the moment
evidence starts accumulating, and uses it (with `use_suspect`, default true)
to hold odometry early - roughly 1.5 cm of phantom travel instead of 9 cm.

A suspicion NEVER cuts power. `stall_guard/suspect` is published for this node
only; stall_guard's own escalation (motor floor, zero cmd_vel) still runs off
the confirmed verdict and is untouched by it.

MEASURED, AND THEREFORE OFF BY DEFAULT
That was the argument. The measurement says no. `stall_guard`'s channel A
compares the wheels' change in speed over a one-second window against the
body's, and on EVERY start from rest the wheels jump to speed in one encoder
sample while the body takes a few hundred milliseconds to follow - so the
shortfall is real, large (measured 0.141 m/s against a 0.101 threshold) and
entirely innocent. `confirm = 0.6 s` is precisely what rides that out. A
`suspect_hold` short enough to be worth having (0.15 s) does not, so every
departure from rest suppressed about 1.2 s of genuine motion.

So `use_suspect` defaults FALSE and the confirmed verdict is what gates
odometry: 0.6 s, about 9 cm of phantom travel at 0.15 m/s, against metres of
drift today. The signal is still published and still wired, because it is
worth watching on the real robot - if a real pin turns out to produce
evidence a start from rest does not, raising `suspect_hold` past the start
transient and turning this on is a one-parameter change.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

# Latched, for the same reason stall_guard latches its verdict: wheel_odometry
# may start before or after this node, and a subscriber that joins late must
# still learn the current state rather than sit on a default.
LATCHED = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class OdomGuard(Node):
    """Detect phantom wheel motion and say so on /wheel_slip."""

    def __init__(self):
        super().__init__('odom_guard')

        # Off switch, so the correction can be removed from the command line
        # rather than by editing a launch file. Disabled, the node still
        # republishes /odom_guarded untouched and holds /wheel_slip false,
        # which makes an A/B comparison against the raw odometry trivial.
        self.declare_parameter('enabled', True)

        # Use stall_guard's early, lower-confidence signal as well as its
        # confirmed verdict. OFF, because it was measured and it does not
        # pay - see CONFIRMED VERSUS SUSPECTED above.
        self.declare_parameter('use_suspect', False)

        # Gate 2. Matches stall_guard's own imu_timeout so the two nodes agree
        # on when the IMU has gone away.
        self.declare_parameter('imu_timeout', 0.5)

        # Gate 3. Matches stall_guard's min_wheel_speed.
        self.declare_parameter('min_wheel_speed', 0.03)

        # Hysteresis on top of stall_guard's own, in SECONDS rather than
        # samples, because /odom arrives at the encoder rate and a count of
        # three samples is 60 ms - far too short to be hysteresis at all.
        #
        # Asymmetric on purpose. A CONFIRMED stall holds odometry at once:
        # stall_guard already spent 0.6 s confirming it. A mere SUSPICION has
        # to persist for suspect_hold first, which throws away the one-cycle
        # flickers a genuine acceleration produces while still catching a
        # real pin in 0.15 s instead of 0.6 s - about 2 cm of phantom travel
        # at 0.15 m/s instead of 9 cm.
        self.declare_parameter('suspect_hold', 0.15)

        # Releasing is slow. A robot grinding against an obstacle twitches,
        # the evidence flickers, and a guard that lets go on the first quiet
        # sample hands back a burst of phantom motion every few hundred
        # milliseconds - measured, exactly that leaked 56% of the phantom
        # distance straight through. Both signals must stay clear for this
        # long before the wheels are believed again.
        self.declare_parameter('release_hold', 1.0)

        # How often /wheel_slip is republished when nothing has changed. The
        # actuator treats a flag older than its own slip_timeout as absent, so
        # this must be comfortably faster than that.
        self.declare_parameter('flag_period', 0.2)

        get = self.get_parameter
        self.enabled = bool(get('enabled').value)
        self.use_suspect = bool(get('use_suspect').value)
        self.imu_timeout = float(get('imu_timeout').value)
        self.min_wheel_speed = float(get('min_wheel_speed').value)
        self.suspect_hold = float(get('suspect_hold').value)
        self.release_hold = float(get('release_hold').value)
        self.flag_period = float(get('flag_period').value)

        self.last_imu = None
        self.stall_confirmed = False
        self.stall_suspect = False

        self.slipping = False
        self.evidence_since = None
        self.clear_since = None

        # Phantom pose accumulated while slipping, subtracted from the raw
        # pose so the guarded pose stands still while the wheels spin and
        # resumes from where it stopped - no jump on release.
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.previous_pose = None
        self.blocked_metres = 0.0

        self.last_flag_pub = None

        # RELIABLE depth 10 to match wheel_odometry's publisher exactly.
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # BEST_EFFORT sensor QoS to match the BNO055 driver. Only used for
        # liveness here - the physics is stall_guard's job, not this node's.
        self.create_subscription(
            Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Bool, 'stall_guard/stalled', self.on_stalled, LATCHED)
        self.create_subscription(
            Bool, 'stall_guard/suspect', self.on_suspect, LATCHED)

        self.guarded_pub = self.create_publisher(
            Odometry, '/odom_guarded', 10)
        self.slip_pub = self.create_publisher(Bool, '/wheel_slip', LATCHED)

        self.publish_flag()

        self.get_logger().info(
            'odom_guard %s: /odom -> /odom_guarded, verdict on /wheel_slip '
            '(suspect signal %s, imu timeout %.1fs). No TF is published - '
            'wheel_odometry owns odom -> base_footprint and applies the '
            'correction there.'
            % ('active' if self.enabled else 'DISABLED (pass-through)',
               'used' if self.use_suspect else 'ignored', self.imu_timeout))

    # ------------------------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_imu(self, msg):
        # Liveness only. A message arriving at all is what gate 2 asks about.
        self.last_imu = self.now()

    def on_stalled(self, msg):
        self.stall_confirmed = bool(msg.data)

    def on_suspect(self, msg):
        self.stall_suspect = bool(msg.data)

    def imu_fresh(self):
        return (self.last_imu is not None
                and self.now() - self.last_imu < self.imu_timeout)

    # ------------------------------------------------------------------

    def on_odom(self, msg):
        wheel_speed = msg.twist.twist.linear.x

        # Gates 2 and 3. Either one missing means believe the wheels, which
        # is what makes an absent IMU behave exactly as it does today.
        #
        # Gate 3 reads the wheel speed off `/odom`, which stays honest even
        # while wheel_odometry is suppressing the pose - deliberately, see
        # the note in wheel_odometry.update(). If the twist were zeroed too,
        # this gate would close the moment the guard acted and the whole
        # thing would oscillate.
        armed = self.enabled and self.imu_fresh()
        claiming = abs(wheel_speed) >= self.min_wheel_speed

        confirmed = armed and claiming and self.stall_confirmed
        suspected = (armed and claiming and self.use_suspect
                     and self.stall_suspect)

        self.register(confirmed, suspected)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        position = msg.pose.pose.position
        previous = self.previous_pose
        self.previous_pose = (position.x, position.y)

        if self.slipping and previous is not None:
            # Keep accumulating the raw pose's advance into the offset so the
            # guarded pose does not move at all. Orientation is passed
            # through: a pinned robot can still be yawing, and stall_guard
            # only ever reasons about the forward axis.
            dx = position.x - previous[0]
            dy = position.y - previous[1]
            self.offset_x += dx
            self.offset_y += dy
            self.blocked_metres += math.hypot(dx, dy)

        out.pose.pose.position.x = position.x - self.offset_x
        out.pose.pose.position.y = position.y - self.offset_y

        if self.slipping:
            out.twist.twist.linear.x = 0.0
            out.twist.twist.linear.y = 0.0

        self.guarded_pub.publish(out)
        self.publish_flag(periodic=True)

    def register(self, confirmed, suspected):
        """Hysteresis, so neither a flicker nor a twitch toggles odometry."""
        now = self.now()

        if confirmed or suspected:
            self.clear_since = None
            if self.evidence_since is None:
                self.evidence_since = now
        else:
            self.evidence_since = None
            if self.clear_since is None:
                self.clear_since = now

        if not self.slipping:
            # Confirmed goes straight through - stall_guard already waited
            # out its own confirm window. A suspicion has to persist.
            ready = confirmed or (
                suspected and self.evidence_since is not None
                and now - self.evidence_since >= self.suspect_hold)
            if ready:
                self.slipping = True
                self.blocked_metres = 0.0
                self.get_logger().warn(
                    'wheel slip (%s): the wheels claim motion the body never '
                    'felt - holding the odometry still so the pose does not '
                    'walk through the obstacle'
                    % ('confirmed' if confirmed else 'suspected'))
                self.publish_flag()

        elif self.clear_since is not None \
                and now - self.clear_since >= self.release_hold:
            self.slipping = False
            self.get_logger().info(
                'wheels gripping again - suppressed %.3f m of motion that '
                'never happened' % self.blocked_metres)
            self.publish_flag()

    def publish_flag(self, periodic=False):
        now = self.now()

        if periodic and self.last_flag_pub is not None \
                and now - self.last_flag_pub < self.flag_period:
            return

        self.last_flag_pub = now
        msg = Bool()
        msg.data = self.slipping
        self.slip_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
