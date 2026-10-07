"""
Catch "the wheels are spinning but the robot is not moving", and do something.

THE FAILURE THIS EXISTS FOR
The wheels report through encoders, so `/odom` reports motion whenever the
motors turn - pinned against a chair leg, hung on a cable, stopped by a
threshold, or just sitting on carpet with the tyres slipping. Everything
downstream believes it: the pose walks through the wall, SLAM inserts scans
at a pose the robot never reached, the coverage planner ticks off floor it
never cleaned, and Nav2 keeps commanding forward because as far as it can
tell forward is working.

This node sits beside the wheels, not inside a planner, so teleop, Nav2,
explore and clean are all protected by the same code. It does not need to be
told which mode is running; it only needs `/imu`, `/odom` and `/cmd_vel`.

WHAT AN ACCELEROMETER CAN AND CANNOT ANSWER
It cannot measure velocity. Integrating it drifts, and worse, a robot rolling
at a steady 0.15 m/s and a robot pinned with its wheels spinning both read
zero forward acceleration. That is Galilean relativity, not a bad part. So
raw |accel| alone can never separate moving from stalled, and the first
tempting detector - "the body is not accelerating, so it is stuck" - would
fire on every healthy robot cruising down a corridor.

What it CAN answer, exactly and without drift, is: *did this body change
velocity?* That is the question the wheels can be cross-examined with,
because a change in wheel speed and a change in body speed are the same
physical event seen two ways. Two channels come out of that, and they cover
each other's blind spot.

CHANNEL A - THE TRANSITION TEST (primary, drift-free)
Over a short window the wheels claim a change in forward speed,
dv_wheel = v_odom(t) - v_odom(t - window). The accelerometer independently
reports the change the body actually underwent, dv_accel = integral of
(ax - bias) over the same window. One second of integration, so bias error
contributes at most bias_error * window - a few hundredths of a m/s.

A free robot MUST show dv_accel ~ dv_wheel. A pinned robot whose wheels ramp
from rest to 0.15 m/s shows dv_wheel = +0.15 and dv_accel ~ 0. A robot
cruising at 0.15 that hits something shows dv_wheel ~ 0 and dv_accel ~ -0.15,
which is the same disagreement with the sign swapped - so the test is run on
the shortfall, |dv_wheel| - (dv_accel projected on the direction of the wheel
change), and catches both the start-up pin and the impact.

The window is only armed when the wheels claim a change worth arguing about
(min_delta_v). At genuine steady state it abstains rather than guesses, which
is the honest answer and is why it cannot fire on a healthy cruise.

CHANNEL B - DIVERGENCE SINCE THE LAST REST (covers steady state)
Channel A says nothing about a robot that has been pinned for ten seconds
with no transitions left to catch. So the body's speed is also integrated
forward from the last moment the robot was provably at rest (no command, no
wheel motion): v_body = integral of (ax - bias) dt, zeroed at every rest.
Against it sits the wheels' claim. A pin that happened anywhere after the
reset leaves a gap that persists - wheels at 0.15, body at 0.00 - and that
gap is the stall, visible at steady state.

This one DOES drift, so it is bounded twice. It is abandoned entirely after
max_integration_time since the last rest (after which the integral is a
guess), and the threshold it must beat GROWS with elapsed time at
accel_bias_drift m/s per second, which is the honest error bar on an
integrated bias. A long pin therefore has to be a big pin, and a long clean
run with no rest stops producing verdicts instead of producing wrong ones.

CHANNEL C - VIBRATION (computed, reported, OFF by default)
A pinned robot driving its motors into a rigid obstacle usually buzzes, and
high-frequency energy with no net dv would be a steady-state signal needing
no integration at all. It is computed here every cycle and published in the
status string. It is NOT a trigger by default, because the only measurement
anyone on this robot has actually taken says it does not separate: the lidar
stall_watch in nexva_explore measured sd(ax) = 0.10 for steady rolling and
sd(ax) = 0.10 for pinned-with-wheels-spinning, identical, and an earlier
86x "separation" turned out to be mislabelled start/stop transients. That was
in simulation, where contact is stiff and the chassis has no resonances, so
on real hardware the difference is probably genuine - but "probably" is not
enough to stop a robot mid-clean. Watch the number in `stall_guard/status`
on the real robot, pinned and rolling, and if it does separate set
`vibration_threshold` from what you measured and `use_vibration:=true`.

GRAVITY AND THE MOUNT
The driver publishes the BNO055's gravity-compensated linear acceleration, so
the forward axis should already be gravity-free. "Should" is doing work
there: it is compensated using the chip's own fused orientation, the mount is
assumed level because the URDF has no imu_link to say otherwise, and any tilt
or axis_remap error leaks a slice of 9.81 onto x. So nothing here assumes a
zero mean. A bias is learned whenever the robot is provably at rest and
subtracted from every sample, which removes a constant leak whatever its
cause and makes the node work equally well if the driver is ever switched to
raw acceleration. The bias is only updated at rest, so a real acceleration
can never be learned as one. No verdict at all is given until a bias exists.

WHAT IT DOES ABOUT IT - "auto adjust itself according to the wheel spin"
Escalating, bounded, reversible:

  1. Say so. `stall_guard/stalled` (latched Bool) and `stall_guard/status`
     (String, with the numbers behind the decision) so any other node can act.
  2. Raise the motor floor. The wheels turn but the body does not, so the
     direct lever is torque at the bottom of the PWM band: publish
     `/pid_limits` with a min PWM stepped up by pwm_step, every
     boost_interval, never past max_min_pwm. Backed off to the baseline the
     moment motion resumes, so the motors do not run hot for the rest of the
     clean. Re-asserted every reassert_period because the web UI re-sends its
     own motor limits every 5 s and would otherwise undo this.
  3. Stop asking for forward motion. If the floor has not freed it within
     escalate_after, publish zero Twist at halt_rate and hold the flag, so
     whoever is driving can run its own recovery. No reversing manoeuvre is
     attempted: this robot has no rear sensing, and backing blind into
     something is a worse failure than the one being fixed.

CLEARING A STALL IS NOT THE ABSENCE OF EVIDENCE
Channel A goes quiet at steady state by design, so "no stall evidence right
now" must NOT be read as "the robot is free" - that releases a still-pinned
robot a second after catching it, backs the motor floor off, and loops. The
verdict is therefore three-valued: STALL, FREE, or NO OPINION. Only positive
FREE evidence clears a latched stall - the body actually felt a change of
speed matching the wheels, the body gained speed the wheels never claimed
(which is exactly what breaking loose looks like), channel B's gap closed, or
the robot stopped being driven at all. NO OPINION freezes both timers and
holds whatever was last decided, which is the right answer to a question the
accelerometer cannot answer and the right behaviour for a robot that is
probably still stuck.

DEGRADING SAFELY
No IMU, a stale IMU, a stale `/odom`, no bias yet, no forward command, wheels
not claiming motion, or a yaw rate too high for the forward-axis model: in
every one of those the node gives NO verdict, cannot raise a flag, and says
which gate it is sitting behind on `stall_guard/status`. An unwired BNO055
therefore makes this node inert, not dangerous.
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

# The flag and the status line are latched: a node that starts after a stall
# has already been declared must still learn about it, and the web UI or a
# planner subscribing late is the normal case rather than the exception.
LATCHED = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _clamp(value, low, high):
    return max(low, min(high, value))


class StallGuard(Node):
    """Watch the wheels against the accelerometer, and act when they lie."""

    def __init__(self):
        super().__init__('stall_guard')

        # ---------------- master switches ----------------

        # On by default: the failure it catches is one the robot cannot
        # notice any other way, and every action it takes is bounded and
        # reversible. Here so it can be turned off from the command line
        # rather than by editing a launch file.
        self.declare_parameter('enabled', True)

        # Detector cycle. 20 Hz is four times the confirm window's resolution
        # and well under the IMU's 50 Hz, so no sample is wasted and no
        # decision waits on the next tick.
        self.declare_parameter('rate', 20.0)

        # How often the status line is republished when nothing changed. A
        # state change always publishes immediately.
        self.declare_parameter('status_period', 1.0)

        # ---------------- gates: when a verdict is allowed at all ----------

        # A command older than this is no command. Matches the ESP32's own
        # 500 ms cmd_vel watchdog, so the guard and the firmware agree on
        # when the robot has stopped being driven.
        self.declare_parameter('cmd_timeout', 0.5)

        # Below this commanded forward speed there is nothing to be stalled
        # out of. 0.03 m/s is under half the slowest speed anything on this
        # robot commands, and excludes pure spot turns.
        self.declare_parameter('min_cmd_speed', 0.03)

        # The wheels must also be claiming motion - a commanded robot whose
        # wheels are NOT turning is a different fault (dead driver, flat
        # pack) and not this node's business.
        self.declare_parameter('min_wheel_speed', 0.03)

        # Sensor freshness. Beyond these the sensor is absent, not quiet, and
        # a missing sensor must never read as a stall.
        self.declare_parameter('imu_timeout', 0.5)
        self.declare_parameter('odom_timeout', 0.5)

        # Above this measured yaw rate the forward-axis model stops holding:
        # the IMU is off the turn centre, so rotation puts real acceleration
        # on x that no wheel speed predicts. 1.0 rad/s is well above the
        # 0.5-0.6 rad/s this robot is normally commanded to turn at, so the
        # gate only catches spin-in-place and violent recoveries.
        self.declare_parameter('max_yaw_rate', 1.0)

        # ---------------- bias (gravity leak and zero-g offset) ------------

        # What counts as "provably at rest": no command at all, and wheels
        # under rest_speed, held for rest_settle.
        self.declare_parameter('rest_speed', 0.01)
        self.declare_parameter('rest_settle', 0.5)

        # EMA weight per cycle while at rest. 0.05 at 20 Hz is a ~1 s time
        # constant: fast enough to have a usable bias a second after boot,
        # slow enough that one noisy sample does not move it.
        self.declare_parameter('bias_alpha', 0.05)

        # Cycles at rest before any verdict is given. 20 = 1 s at the default
        # rate, i.e. one full time constant.
        self.declare_parameter('bias_warmup', 20)

        # ---------------- channel A: the transition test -------------------

        # Seconds of history the two deltas are taken over. Long enough to
        # contain a full ramp from rest (the ESP32 reaches target in ~0.3 s)
        # and short enough that integrated bias error stays ~0.02 m/s.
        self.declare_parameter('window', 1.0)

        # Below this claimed change the window is not armed. 0.06 m/s is
        # about twice the integration noise over the window, so an unarmed
        # window is one where the answer would have been noise.
        self.declare_parameter('min_delta_v', 0.06)

        # How much forward speed the BODY must gain, with the wheels
        # claiming no change, before a latched stall is called over. This is
        # what breaking loose after a PWM boost looks like from the
        # accelerometer. 0.10 m/s is two thirds of the 0.15 m/s cruise, and
        # deliberately well above min_delta_v: the release test has to clear
        # the integrated bias error too, which over a 1 s window is a few
        # hundredths of a m/s and would otherwise free the robot on paper
        # while it sat there spinning its wheels. Measured doing exactly
        # that at 0.048 m/s of drift.
        self.declare_parameter('break_free_delta_v', 0.10)

        # Fraction of the claimed change the body must actually show. 0.3 is
        # deliberately generous - wheel slip, a gentle ramp and a soft carpet
        # all cost real fraction - so only a gross disagreement counts.
        self.declare_parameter('accel_follow_ratio', 0.3)

        # Absolute floor on the shortfall, in m/s, so a small claimed change
        # cannot be failed by integration noise alone.
        self.declare_parameter('delta_v_noise', 0.03)

        # ---------------- channel B: divergence since rest -----------------

        # Sustained gap between wheel speed and integrated body speed, as
        # it stands at the moment of the reset. 0.05 m/s is a third of this
        # robot's 0.15 m/s cruise - a pin shows the whole 0.15, ordinary
        # slip far less - and it is the FLOOR of the test, not the whole of
        # it: the drift allowance below is added on top and grows with time,
        # so at 5 s since the last rest the gap must beat 0.15 m/s.
        self.declare_parameter('divergence_threshold', 0.05)

        # The honest error bar on the integral, m/s added to the threshold
        # per second since the last rest. 0.02 m/s^2 is the residual offset
        # to expect from a calibrated BNO055 after the at-rest bias has been
        # removed. This makes a long integration require a bigger gap, which
        # is exactly what it should require.
        self.declare_parameter('accel_bias_drift', 0.02)

        # After this long without a rest the integral is a guess and channel
        # B abstains. At the drift figure above, 6 s is a +-0.12 m/s error
        # bar, which is already most of a cruise speed.
        self.declare_parameter('max_integration_time', 6.0)

        # The same integral is also the evidence that the wheels and the
        # body AGREE, which is one of the ways a latched stall is released -
        # and a release has to be held to a higher standard than a trigger,
        # because releasing wrongly hands a still-stuck robot back to a
        # planner that believes it. So agreement is only accepted on a fresh
        # integral. Measured without this: integrated drift closed the gap
        # on its own and freed a robot that was still pinned, after 3.3 s
        # (quiet) and 5.5 s (vibrating).
        self.declare_parameter('agreement_window', 3.0)

        # A rough floor shakes the accelerometer, and integrating a shaken
        # signal is a random walk: the error after T seconds of samples
        # spaced dt apart grows as rms * sqrt(dt * T). That is a known
        # quantity here - the vibration rms is measured every cycle - so
        # instead of picking one threshold that is either deaf on carpet or
        # jumpy on tile, the gap test is given exactly that much more room
        # on a floor that is actually rough. 1.0 is one standard deviation
        # of the walk; 0 switches the term off. Measured without it: a
        # synthetic rough floor (bumps of 2-4 m/s^2, rms 0.3-1.1) produced a
        # false stall at gap 0.106 against a 0.072 limit, while the same
        # bumps on the quiet-floor runs never reached 0.03.
        self.declare_parameter('vibration_allowance', 1.0)

        # Hard ceiling on one sample's acceleration before it is integrated,
        # m/s^2. This chassis is capped at 0.30 m/s and reaches it in about
        # 0.12 s, so its own dynamics cannot exceed ~2.5 m/s^2; a reading
        # above that is the floor hitting a wheel, not the body changing
        # velocity. Clipping keeps one 4 m/s^2 strike from injecting 0.08
        # m/s into an integral that has no way to give it back.
        self.declare_parameter('accel_clip', 2.0)

        # ---------------- channel C: vibration (off by default) ------------

        # See the module docstring: computed and reported always, a trigger
        # only once someone has measured rolling against pinned on the real
        # robot. The threshold below is a placeholder, NOT a measurement.
        self.declare_parameter('use_vibration', False)
        self.declare_parameter('vibration_threshold', 0.35)
        self.declare_parameter('vibration_window', 0.5)

        # ---------------- hysteresis ---------------------------------------

        # Evidence must hold for this long before it is called a stall, and
        # must be gone this long before it is called over. One bump, one
        # dropped sample or one noisy window cannot flip the verdict.
        self.declare_parameter('confirm', 0.6)
        self.declare_parameter('release', 1.0)

        # ---------------- response ------------------------------------------

        # Step 2 of the escalation: raise the PWM floor.
        self.declare_parameter('adjust_pwm', True)

        # The floor to go back to. 150 is DEFAULT_MIN_PWM in
        # microros_code.ino. If anything else publishes /pid_limits - the web
        # UI's motor-limits card does, every 5 s - its value is adopted as
        # the baseline instead, so backing off restores what the operator set
        # rather than this default.
        self.declare_parameter('baseline_min_pwm', 150.0)

        # How much to add per escalation step, and how often.
        self.declare_parameter('pwm_step', 20.0)
        self.declare_parameter('boost_interval', 1.0)

        # Hard ceiling on the floor, enforced here and not left to the
        # firmware. 210 of 255 keeps real headroom between min and max so the
        # PI loop still has a band to work in; the firmware's own
        # "min = max - 1" rescue would leave it none.
        self.declare_parameter('max_min_pwm', 210.0)

        # The web UI re-sends its motor limits every 5 s, which would quietly
        # undo the boost. Re-assert ours faster than that.
        self.declare_parameter('reassert_period', 2.0)

        # Step 3: after this long still stalled, stop asking for forward
        # motion. 3 s is three boost steps - enough to know the floor is not
        # the answer here.
        self.declare_parameter('escalate_after', 3.0)
        self.declare_parameter('publish_zero_cmd', True)
        self.declare_parameter('halt_rate', 10.0)

        get = self.get_parameter
        self.enabled = bool(get('enabled').value)
        self.rate = float(get('rate').value)
        self.status_period = float(get('status_period').value)
        self.cmd_timeout = float(get('cmd_timeout').value)
        self.min_cmd_speed = float(get('min_cmd_speed').value)
        self.min_wheel_speed = float(get('min_wheel_speed').value)
        self.imu_timeout = float(get('imu_timeout').value)
        self.odom_timeout = float(get('odom_timeout').value)
        self.max_yaw_rate = float(get('max_yaw_rate').value)
        self.rest_speed = float(get('rest_speed').value)
        self.rest_settle = float(get('rest_settle').value)
        self.bias_alpha = float(get('bias_alpha').value)
        self.bias_warmup = int(get('bias_warmup').value)
        self.window = float(get('window').value)
        self.min_delta_v = float(get('min_delta_v').value)
        self.break_free_delta_v = float(get('break_free_delta_v').value)
        self.accel_follow_ratio = float(get('accel_follow_ratio').value)
        self.delta_v_noise = float(get('delta_v_noise').value)
        self.divergence_threshold = float(get('divergence_threshold').value)
        self.accel_bias_drift = float(get('accel_bias_drift').value)
        self.max_integration_time = float(get('max_integration_time').value)
        self.agreement_window = float(get('agreement_window').value)
        self.vibration_allowance = float(get('vibration_allowance').value)
        self.accel_clip = float(get('accel_clip').value)
        self.use_vibration = bool(get('use_vibration').value)
        self.vibration_threshold = float(get('vibration_threshold').value)
        self.vibration_window = float(get('vibration_window').value)
        self.confirm = float(get('confirm').value)
        self.release = float(get('release').value)
        self.adjust_pwm = bool(get('adjust_pwm').value)
        self.baseline_min_pwm = float(get('baseline_min_pwm').value)
        self.pwm_step = float(get('pwm_step').value)
        self.boost_interval = float(get('boost_interval').value)
        # Hard ceiling on the ceiling. The firmware's max PWM is 255 and it
        # rescues an inverted band by setting min = max - 1, which leaves
        # the PI loop no band at all; this node must never get it into that
        # position, whatever max_min_pwm is set to.
        self.max_min_pwm = min(254.0, float(get('max_min_pwm').value))
        self.reassert_period = float(get('reassert_period').value)
        self.escalate_after = float(get('escalate_after').value)
        self.publish_zero_cmd = bool(get('publish_zero_cmd').value)
        self.halt_rate = float(get('halt_rate').value)

        # ---------------- state ----------------

        # (t, forward acceleration, yaw rate) and (t, wheel forward speed).
        # Timestamped on arrival, not from the header: everything here runs
        # on one machine off one clock, and arrival time is what the
        # freshness gates need to be measuring anyway.
        self.imu_history = deque()
        self.odom_history = deque()

        self.last_imu = None            # arrival time of the newest sample
        self.last_odom = None
        self.last_cmd = None
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.wheel_speed = 0.0
        self.yaw_rate = 0.0

        self.bias = 0.0
        self.bias_count = 0
        self.rest_since = None

        self.body_speed = 0.0           # channel B integral
        self.integration_start = None
        self.last_integrated = None

        self.stalled = False
        self.evidence_since = None
        self.clear_since = None
        self.stall_since = None
        self.reason = ''

        self.boost_steps = 0
        self.current_min_pwm = self.baseline_min_pwm
        self.last_boost = None
        self.last_limits_pub = None
        self.sent_limits = deque()      # (t, value) we published ourselves
        self.halting = False
        self.last_zero = None

        self.last_status = None
        self.status_text = ''

        # ---------------- plumbing ----------------

        # BEST_EFFORT to match the driver, which publishes the IMU as a
        # stream where a late sample is worse than no sample.
        self.create_subscription(
            Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Vector3, '/pid_limits', self.on_limits, 10)

        self.stalled_pub = self.create_publisher(
            Bool, 'stall_guard/stalled', LATCHED)
        self.status_pub = self.create_publisher(
            String, 'stall_guard/status', LATCHED)

        # An EARLY, lower-confidence signal, for odometry gating only.
        #
        # `confirm` is 0.6 s because cutting motor power on one odd
        # accelerometer sample is unacceptable. But 0.6 s at 0.15 m/s is 9 cm
        # of phantom travel already written into the map before `stalled`
        # rises, and SUPPRESSING ODOMETRY is not the same kind of decision as
        # cutting power: it is free and instantly reversible, and a false
        # suppression costs a few centimetres of real motion that scan
        # matching puts straight back. So `suspect` rises the moment evidence
        # starts accumulating - roughly one detector cycle, 50 ms - and
        # `odom_guard` gates `/wheel_slip` on it.
        #
        # It is NEVER a trigger for anything in this node. The motor floor
        # and the zero-cmd_vel halt in `respond` run off `self.stalled` and
        # nothing else, so a suspicion on its own can never cut power.
        self.suspect_pub = self.create_publisher(
            Bool, 'stall_guard/suspect', LATCHED)
        # Not None, so the first publish_suspect() below always latches an
        # explicit false rather than leaving a late subscriber with nothing.
        self.suspect = None
        self.limits_pub = self.create_publisher(Vector3, '/pid_limits', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.publish_flag()

        if not self.enabled:
            self.set_status('DISABLED (enabled:=false) - no verdicts, no '
                            'pid_limits, no cmd_vel')
            self.get_logger().warn(
                'stall_guard is disabled; nothing is watching the wheels')
            return

        self.create_timer(1.0 / self.rate, self.step)

        self.get_logger().info(
            'stall_guard watching imu + /odom against /cmd_vel: window %.1fs, '
            'confirm %.1fs, dv ratio %.2f, divergence %.2f m/s, pwm floor '
            '%.0f -> max %.0f in steps of %.0f, halt after %.1fs, '
            'vibration %s'
            % (self.window, self.confirm, self.accel_follow_ratio,
               self.divergence_threshold, self.baseline_min_pwm,
               self.max_min_pwm, self.pwm_step, self.escalate_after,
               'ON' if self.use_vibration else 'reported only'))

    # ------------------------------------------------------------------
    # inputs

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg):
        now = self.now()
        ax = msg.linear_acceleration.x
        wz = msg.angular_velocity.z

        if not (math.isfinite(ax) and math.isfinite(wz)):
            return

        self.last_imu = now
        self.yaw_rate = wz
        self.imu_history.append((now, ax, wz))
        self.trim(self.imu_history, now)

        # Channel B's integral is advanced here rather than on the detector
        # tick so it uses every sample the board gave us, at the interval it
        # actually gave them.
        previous = self.last_integrated
        self.last_integrated = now

        if previous is None:
            return

        step = now - previous

        # A wild timestep means a dropped stretch of samples, not a real
        # interval. Folding it in would invent a velocity that never happened.
        if 0.0 < step < 0.2:
            self.body_speed += self.corrected(ax) * step

    def on_odom(self, msg):
        now = self.now()
        self.last_odom = now
        self.wheel_speed = msg.twist.twist.linear.x
        self.odom_history.append((now, self.wheel_speed))
        self.trim(self.odom_history, now)

    def on_cmd(self, msg):
        # Our own zero-velocity messages come back to us here. Recording them
        # as "a command" would be harmless - they are zero, so they fail the
        # min_cmd_speed gate - but recording them as a *recent* command would
        # keep the robot out of the at-rest state and stop the bias ever
        # being re-learned. So a zero from anyone is simply not a command.
        if abs(msg.linear.x) < 1e-6 and abs(msg.angular.z) < 1e-6:
            return

        self.last_cmd = self.now()
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

    def on_limits(self, msg):
        """
        Learn the operator's motor floor from whoever else is setting it.

        The web UI's motor-limits card re-publishes its values every 5 s. If
        that number is taken as the baseline, backing off after a stall
        restores what the operator chose instead of this node's default - and
        the guard stops fighting the UI over the same field.
        """
        if msg.x <= 0.0:
            return

        # Our own boost, echoed back. Every value we have sent recently has
        # to be remembered, not just the latest: during an escalation the
        # echo of step 1 arrives after step 2 has gone out, so comparing
        # against "the last thing we sent" adopts our own 170 as the
        # operator's baseline and ratchets it up one stall at a time until
        # the robot runs permanently hot. Measured doing exactly that.
        now = self.now()
        while self.sent_limits and now - self.sent_limits[0][0] > 15.0:
            self.sent_limits.popleft()

        if any(abs(msg.x - value) < 0.5 for _, value in self.sent_limits):
            return

        if abs(msg.x - self.baseline_min_pwm) < 0.5:
            return

        self.get_logger().info(
            'baseline min PWM adopted from /pid_limits: %.0f -> %.0f'
            % (self.baseline_min_pwm, msg.x))
        self.baseline_min_pwm = float(msg.x)

        if not self.stalled:
            self.current_min_pwm = self.baseline_min_pwm

    def trim(self, history, now):
        horizon = max(self.window, self.vibration_window) + 1.0
        while history and now - history[0][0] > horizon:
            history.popleft()

    # ------------------------------------------------------------------
    # detector

    def step(self):
        now = self.now()

        self.update_rest(now)

        verdict, detail = self.assess(now)

        if verdict == 'stall':
            self.clear_since = None
            if self.evidence_since is None:
                self.evidence_since = now
            if (not self.stalled
                    and now - self.evidence_since >= self.confirm):
                self.declare_stall(now, detail)
        elif verdict == 'free_now':
            # Positive, unambiguous evidence the body accelerated forward
            # when the wheels did not ask it to - the robot came unstuck.
            # Acted on at once rather than after `release`: if it is wrong
            # the stall re-confirms within `confirm`, and in the meantime
            # the motor floor goes back down where it belongs.
            self.evidence_since = None
            self.clear_since = None
            if self.stalled:
                self.clear_stall(now, detail)

        elif verdict == 'free':
            self.evidence_since = None
            if self.clear_since is None:
                self.clear_since = now
            if self.stalled and now - self.clear_since >= self.release:
                self.clear_stall(now, detail)
        else:
            # No opinion: the last verdict stands. The confirm timer freezes
            # (a stall part-way to being confirmed is not forgotten), but
            # the release timer is cleared, because release demands
            # CONTINUOUS positive evidence of motion. Measured without this:
            # one drift-induced free sample started a release clock that the
            # following second of no-opinion samples could not stop, and a
            # still-pinned robot un-stalled itself 2.8 s after being caught.
            self.clear_since = None

        self.publish_suspect()

        if self.stalled:
            self.respond(now)

        state = 'STALL' if self.stalled else (
            'hold' if verdict == 'unknown' else 'ok')
        self.set_status('%s %s' % (state, detail), periodic=True)

    def update_rest(self, now):
        """
        Learn the accelerometer's forward-axis bias while provably at rest.

        Only at rest: a bias learned while the robot was accelerating would
        absorb the very signal this node exists to measure, and would do it
        silently.
        """
        commanded = (self.last_cmd is not None
                     and now - self.last_cmd < self.cmd_timeout)
        still = abs(self.wheel_speed) < self.rest_speed

        if commanded or not still:
            self.rest_since = None
            return

        if self.rest_since is None:
            self.rest_since = now
            return

        if now - self.rest_since < self.rest_settle:
            return

        # At rest for long enough: the body speed is zero by definition, so
        # the integral is reset here and nowhere else, and whatever the
        # accelerometer is reading now is offset.
        self.body_speed = 0.0
        self.integration_start = now

        if self.imu_history:
            ax = self.imu_history[-1][1]
            self.bias += self.bias_alpha * (ax - self.bias)
            self.bias_count += 1

    def assess(self, now):
        """
        What does the accelerometer say about the wheels right now?

        Returns one of 'stall', 'free' or 'unknown' and a human-readable
        detail. 'free' is POSITIVE evidence of motion (or of a robot that is
        not being driven), never merely the absence of stall evidence -
        channel A is silent at steady state by design, and treating that
        silence as freedom releases a robot that is still pinned. 'unknown'
        holds the last verdict. Every gate names itself in the detail,
        because a guard that goes quiet without saying why is
        indistinguishable from a guard that is working.
        """
        if self.last_imu is None:
            return 'free', ('NO IMU - never received a sample on imu; guard '
                           'inactive, it cannot and will not fire')

        imu_age = now - self.last_imu
        if imu_age > self.imu_timeout:
            return 'free', ('NO IMU - last sample %.1fs ago (>%.1fs); guard '
                           'inactive' % (imu_age, self.imu_timeout))

        if self.last_odom is None or now - self.last_odom > self.odom_timeout:
            age = 'never' if self.last_odom is None else (
                '%.1fs ago' % (now - self.last_odom))
            return 'free', 'NO ODOM - last /odom %s; guard inactive' % age

        if self.bias_count < self.bias_warmup:
            return 'free', ('WARMUP - no at-rest accelerometer bias yet '
                           '(%d/%d rest samples); no verdicts until there is'
                           % (self.bias_count, self.bias_warmup))

        commanding = (self.last_cmd is not None
                      and now - self.last_cmd < self.cmd_timeout
                      and abs(self.cmd_linear) >= self.min_cmd_speed)

        if not commanding:
            return 'free', ('IDLE - no forward cmd_vel (>=%.2f m/s) in the '
                           'last %.1fs; a stationary robot is not stalled'
                           % (self.min_cmd_speed, self.cmd_timeout))

        if abs(self.wheel_speed) < self.min_wheel_speed:
            return 'free', ('WHEELS STOPPED - odom %.3f m/s under %.2f; '
                           'commanded but not turning is a different fault'
                           % (self.wheel_speed, self.min_wheel_speed))

        if abs(self.yaw_rate) > self.max_yaw_rate:
            return 'free', ('TURNING - gyro %.2f rad/s over %.2f; the '
                           'forward-axis model does not hold here'
                           % (self.yaw_rate, self.max_yaw_rate))

        dv_wheel, dv_accel, samples = self.deltas(now)
        vibration = self.vibration(now)
        period = self.sample_period()

        numbers = ('cmd %.2f wheel %.3f dv_wheel %+.3f dv_accel %+.3f '
                   'v_body %+.3f bias %+.3f vib %.3f'
                   % (self.cmd_linear, self.wheel_speed, dv_wheel, dv_accel,
                      self.body_speed, self.bias, vibration))

        if samples < 2:
            return 'unknown', 'NO WINDOW - too few IMU samples; ' + numbers

        # ---- channel B's gap, computed first: channel A leans on it ----
        gap = None
        limit = 0.0
        elapsed = 0.0

        if self.integration_start is not None:
            elapsed = now - self.integration_start
            if elapsed <= self.max_integration_time:
                heading = 1.0 if self.wheel_speed >= 0.0 else -1.0
                gap = abs(self.wheel_speed) - heading * self.body_speed
                limit = (self.divergence_threshold
                         + self.accel_bias_drift * elapsed
                         + self.vibration_allowance * vibration
                         * math.sqrt(max(elapsed, period) * period))

        # ---- channel A: the transition test ----
        #
        # Both deltas are projected on the direction of TRAVEL, not on the
        # direction of the wheel change, because the two halves of this
        # failure look opposite from the wheels' side: a robot pinned at
        # start-up has the wheels claiming +0.15 and the body feeling
        # nothing, while a robot that drives into a chair at full speed has
        # the body feeling -0.15 and the wheels claiming nothing. Arming on
        # |dv_wheel| alone misses the second one entirely - measured, the
        # impact case never fired - so the window is armed by whichever side
        # moved, and the shortfall is what the body owes the wheels.
        direction = 1.0 if self.wheel_speed >= 0.0 else -1.0
        claimed = direction * dv_wheel
        realised = direction * dv_accel
        shortfall = claimed - realised
        magnitude = max(abs(claimed), abs(realised))
        needed = max(self.delta_v_noise,
                     (1.0 - self.accel_follow_ratio) * magnitude)

        # When the WHEELS are the side that changed, the shortfall stands on
        # its own: the wheels claimed an acceleration and the body did not
        # feel it, and nothing but a stall does that. When only the BODY
        # changed - a deceleration the wheels never asked for - it needs
        # corroboration, because a hard floor bump is also a deceleration
        # the wheels never asked for. The difference is what happens next:
        # after a collision the body stays slow and channel B's gap opens
        # and stays open, while after a bump the robot carries on and the
        # gap closes again. Measured without this: a synthetic rough floor
        # (bumps of 2-4 m/s^2) produced two false stalls in one 12 s run,
        # at dv_accel -0.127 and -0.170 with the gap NEGATIVE throughout.
        wheels_claimed = abs(claimed) >= self.min_delta_v
        corroborated = gap is not None and gap > limit

        if (magnitude >= self.min_delta_v and shortfall > needed
                and (wheels_claimed or corroborated)):
            return 'stall', ('DV MISMATCH over %.1fs: wheels claim %+.3f '
                             'm/s, body felt %+.3f, shortfall %.3f > %.3f '
                             '| %s' % (self.window, claimed, realised,
                                       shortfall, needed, numbers))

        # ---- channel B: divergence since the last rest ----
        if corroborated:
            return 'stall', ('VELOCITY DIVERGENCE %.3f > %.3f m/s after '
                             '%.1fs of integration: wheels %.3f, body '
                             '%+.3f | %s'
                             % (gap, limit, elapsed, self.wheel_speed,
                                self.body_speed, numbers))

        # ---- channel C: vibration, only if someone has measured it ----
        if (self.use_vibration
                and vibration > self.vibration_threshold
                and abs(realised) < self.delta_v_noise):
            return 'stall', ('VIBRATION %.3f > %.3f m/s^2 with no net dv | %s'
                          % (vibration, self.vibration_threshold, numbers))

        # ---- positive evidence that the body is moving ----
        #
        # Each of these is something a pinned robot cannot produce. Nothing
        # else clears a latched stall.
        if magnitude >= self.min_delta_v:
            if shortfall > needed:
                # A shortfall that reached here is a body-only deceleration
                # with no corroboration: a bump, or a collision so long
                # after the last rest that channel B has already given up.
                # Those two cannot be told apart from here, so neither a
                # stall nor a release is justified - and NOT re-anchoring
                # matters, because re-anchoring at the instant of a real
                # collision would erase the very gap that proves it.
                return 'unknown', ('body lost %.3f m/s the wheels did not '
                                   'claim, uncorroborated - no verdict | %s'
                                   % (shortfall, numbers))

            self.reanchor(now)
            return 'free', ('body followed the wheels: claimed %+.3f, felt '
                            '%+.3f | %s' % (claimed, realised, numbers))

        if realised > self.break_free_delta_v:
            # Strong enough to act on by itself - see step().
            # The body sped up in the direction of travel without the wheels
            # claiming it did - which is exactly what breaking loose after a
            # PWM boost looks like, and a pinned robot never does it.
            return 'free_now', ('body gained %+.3f m/s (> %.3f) the wheels '
                                'did not claim - broke loose | %s'
                                % (realised, self.break_free_delta_v,
                                   numbers))

        if (gap is not None and gap < 0.5 * limit
                and elapsed <= self.agreement_window):
            if self.stalled:
                # While a stall is latched the wheels are known to be lying
                # and the integral is known to be contaminated, so "the gap
                # closed" is as likely to be drift catching up with the lie
                # as it is to be the robot moving - and re-anchoring to the
                # wheels here would make that self-confirming for ever.
                # Measured: under vibration the integral crept up until the
                # gap closed on its own and freed a robot still pinned, at
                # 2.6 s. Release needs the drift-free evidence above.
                return 'unknown', ('gap closed while stalled - not trusted '
                                   'as release | %s' % numbers)

            self.reanchor(now)
            return 'free', ('wheels and body agree: gap %.3f well under %.3f '
                            'after %.1fs | %s'
                            % (gap, limit, elapsed, numbers))

        # Steady state, past the integration horizon, nothing to measure.
        return 'unknown', 'no opinion (steady state); ' + numbers

    def reanchor(self, now):
        """
        Trust the wheels again, at a moment the accelerometer just agreed.

        Channel B integrates from the last rest, and on a clean run there
        may not be another rest for minutes - past max_integration_time the
        integral is a guess and the channel goes silent, which would leave a
        collision ten seconds into a row undetectable. But every time the
        two sensors positively agree, the body's speed IS the wheel speed to
        within the test that just passed, so the integral can be reset to it
        and the clock restarted. Drift is then bounded by the time since the
        last agreement rather than the time since the last stop. This is the
        one place the wheels are believed, and it only happens when the
        accelerometer has just said they are telling the truth.
        """
        self.body_speed = self.wheel_speed
        self.integration_start = now

    def deltas(self, now):
        """
        Wheel-reported and accelerometer-reported change in forward speed.

        Both over the same window, which is the only reason they can be
        compared: one is a difference of two reported speeds, the other the
        integral of the acceleration between those two instants.
        """
        start = now - self.window

        samples = [s for s in self.imu_history if s[0] >= start]
        dv_accel = 0.0
        for index in range(1, len(samples)):
            t0, a0, _ = samples[index - 1]
            t1, a1, _ = samples[index]
            step = t1 - t0
            if 0.0 < step < 0.2:
                dv_accel += 0.5 * (self.corrected(a0)
                                   + self.corrected(a1)) * step

        older = [s for s in self.odom_history if s[0] <= start]
        if older:
            then = older[-1][1]
        elif self.odom_history:
            then = self.odom_history[0][1]
        else:
            then = self.wheel_speed

        return self.wheel_speed - then, dv_accel, len(samples)

    def corrected(self, ax):
        """One sample, de-biased and clipped to what this chassis can do."""
        return _clamp(ax - self.bias, -self.accel_clip, self.accel_clip)

    def sample_period(self):
        """Measured IMU sample spacing, for the random-walk noise term."""
        if len(self.imu_history) < 3:
            return 0.02
        span = self.imu_history[-1][0] - self.imu_history[0][0]
        step = span / (len(self.imu_history) - 1)
        return step if 0.0 < step < 0.2 else 0.02

    def vibration(self, now):
        """RMS of the forward acceleration about its own mean, m/s^2."""
        start = now - self.vibration_window
        values = [s[1] for s in self.imu_history if s[0] >= start]

        if len(values) < 3:
            return 0.0

        mean = sum(values) / len(values)
        return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))

    # ------------------------------------------------------------------
    # response

    def declare_stall(self, now, detail):
        self.stalled = True
        self.stall_since = now
        self.boost_steps = 0
        self.last_boost = None
        self.halting = False
        self.reason = detail

        self.publish_flag()
        self.set_status('STALL %s' % detail)
        self.get_logger().warn('STALL: %s' % detail)

    def clear_stall(self, now, detail):
        self.stalled = False
        self.stall_since = None
        self.halting = False
        self.reason = ''

        self.publish_flag()

        # Back the motor floor off. Leaving it raised would have the robot
        # run the rest of the clean hotter and louder than it needs to, and
        # would hide the next stall behind the last one's fix.
        if self.current_min_pwm > self.baseline_min_pwm:
            self.get_logger().info(
                'stall cleared after %d boost step(s); min PWM %.0f -> %.0f'
                % (self.boost_steps, self.current_min_pwm,
                   self.baseline_min_pwm))
            self.send_limits(self.baseline_min_pwm)

        self.boost_steps = 0
        self.set_status('RECOVERED %s' % detail)

    def respond(self, now):
        """Escalate while the stall holds: flag, then torque, then stop."""
        if self.adjust_pwm:
            due = (self.last_boost is None
                   or now - self.last_boost >= self.boost_interval)

            if due and self.current_min_pwm < self.max_min_pwm:
                target = _clamp(self.current_min_pwm + self.pwm_step,
                                0.0, self.max_min_pwm)
                self.boost_steps += 1
                self.last_boost = now
                self.get_logger().warn(
                    'stall step %d: raising min PWM %.0f -> %.0f (ceiling '
                    '%.0f) to break static friction'
                    % (self.boost_steps, self.current_min_pwm, target,
                       self.max_min_pwm))
                self.send_limits(target)

            elif (self.current_min_pwm > self.baseline_min_pwm
                  and self.last_limits_pub is not None
                  and now - self.last_limits_pub >= self.reassert_period):
                # The web UI re-sends its own limits every 5 s; without this
                # the boost silently evaporates mid-stall.
                self.send_limits(self.current_min_pwm)

        if not self.publish_zero_cmd or self.stall_since is None:
            return

        if now - self.stall_since < self.escalate_after:
            return

        if not self.halting:
            self.halting = True
            self.get_logger().error(
                'stall not cleared after %.1fs and %d PWM step(s) - holding '
                'cmd_vel at zero and leaving stall_guard/stalled latched '
                'true. No reverse is attempted: this robot has no rear '
                'sensing.' % (now - self.stall_since, self.boost_steps))

        # Only while something is still asking for forward motion. Once the
        # driver gives up on its own there is nothing left to suppress, and
        # publishing into an idle topic would only fight the next mode.
        commanding = (self.last_cmd is not None
                      and now - self.last_cmd < self.cmd_timeout
                      and abs(self.cmd_linear) >= self.min_cmd_speed)

        if not commanding:
            return

        if (self.last_zero is not None
                and now - self.last_zero < 1.0 / self.halt_rate):
            return

        self.last_zero = now
        self.cmd_pub.publish(Twist())

    def send_limits(self, min_pwm):
        """
        Set the firmware's minimum PWM, bounded here and not only there.

        x only: y and z are left at <= 0, which the firmware reads as "leave
        that field alone", so the max PWM and the speed cap the operator set
        are untouched.
        """
        value = _clamp(float(min_pwm), 0.0, self.max_min_pwm)

        msg = Vector3()
        msg.x = value
        msg.y = -1.0
        msg.z = -1.0

        self.limits_pub.publish(msg)

        self.current_min_pwm = value
        self.last_limits_pub = self.now()
        self.sent_limits.append((self.last_limits_pub, value))

    def publish_flag(self):
        msg = Bool()
        msg.data = self.stalled
        self.stalled_pub.publish(msg)
        self.publish_suspect()

    def publish_suspect(self):
        """Republish the early signal, on change only."""
        # True while evidence is accumulating towards a stall, and for as
        # long as a confirmed stall holds. Falls with the evidence, so a
        # single bad sample raises it for one cycle and no longer.
        suspect = self.stalled or self.evidence_since is not None
        if suspect == self.suspect:
            return
        self.suspect = suspect
        msg = Bool()
        msg.data = suspect
        self.suspect_pub.publish(msg)

    def set_status(self, text, periodic=False):
        now = self.now()

        if periodic and text == self.status_text and self.last_status \
                is not None and now - self.last_status < self.status_period:
            return

        self.status_text = text
        self.last_status = now

        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def shutdown(self):
        """Leave the motors exactly as they were found."""
        try:
            if self.current_min_pwm > self.baseline_min_pwm:
                self.send_limits(self.baseline_min_pwm)
            if self.stalled:
                self.stalled = False
                self.publish_flag()
        except Exception:                                   # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = StallGuard()

    # ExternalShutdownException is what a SIGTERM looks like from in here -
    # a launch file tearing down, systemd, `timeout`. Without catching it the
    # node exits through a traceback every time it is stopped normally.
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
