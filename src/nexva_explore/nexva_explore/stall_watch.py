"""
Deciding whether the robot is really moving, when the wheels cannot be asked.

Wheel odometry integrates whatever the wheels do. Drive into a wall and the
wheels keep turning, so `/odom` keeps reporting 0.16 m/s while the robot has
not moved at all. Measured in this sim, driving into an obstacle and holding
the command: 24 s of reported motion, nearly 4 m of floor the robot never
crossed, with the forward lidar range frozen at 4.61 m the whole time. Nothing
downstream doubts it - the pose walks through the wall, SLAM inserts scans at
that wrong pose, and the map grows floor that is not there.

WHAT THE ACCELEROMETER CANNOT DO, AND WHY THAT IS NOT A BUG
An accelerometer cannot tell constant-velocity motion from standing still.
That is Galilean relativity, not a shortcoming of the part. Rolling at a
steady 0.16 m/s and being pinned against a wall are both zero acceleration,
and both readings are dominated by the same slice of gravity leaking through
chassis tilt.

It is tempting anyway, because a rolling robot ought to VIBRATE. That was
tried here and measured, and it does not survive contact with the simulator:

    steady rolling at 0.16 m/s   sd(ax) = 0.10
    pinned, wheels spinning      sd(ax) = 0.10

Identical. An earlier measurement appeared to show an 86x separation, but that
split the samples by a test that mislabelled the classes, so the "moving" set
was really the start, stop and impact transients - which are of course violent.
Steady-state rolling is not. On real hardware the vibration difference is
probably genuine, but it cannot be verified here, and a trigger that fires on
an unverified signal would stop the robot dead in the middle of a clean run.
So vibration is NOT used as a trigger. Do not add it back without measuring
steady rolling against steady pinning on the actual robot.

WHAT ACTUALLY ANSWERS IT
The lidar, asked properly. For a translation d along the robot's x axis, a
beam at bearing th measuring range r changes by about -d*cos(th). So the
change every beam SHOULD show, if the wheels are telling the truth, is known
in advance. Fitting the observed changes against those predicted changes by
least squares gives one number: the fraction of the claimed motion the world
actually backs up.

    fit = sum(observed * predicted) / sum(predicted * predicted)

1.0 means the room moved past exactly as fast as the wheels claim. 0.0 means
the wheels are turning and the world is standing still.

This is not the same as averaging |range change| over all beams, which is what
was here before and what fails. That average is diluted by beams pointing
across the direction of travel, which barely change however fast the robot is
going - so it splits cleanly in a corridor (0.025 m moving against 0.0000 m
pinned) and then calls a freely driving robot "frozen" for a whole 50 s run in
the middle of an open room. In the least-squares form those same beams predict
~0, contribute ~0 to both sums, and drop out on their own.

WHY THE PREDICTION USES THE WALL, NOT THE BEAM
The first version of this predicted -d*cos(th) per beam, which treats every
return as a point floating in space. Real returns come off SURFACES, and for
a surface the range change depends on how the wall is angled, not just where
the beam points: a wall dead ahead changes by d/cos(incidence), a wall
parallel to travel does not change at all however far the robot drives, and
at an obstacle edge the beam slides off onto whatever is behind, jumping by
metres. Switched to boustrophedon rows - long straight runs beside walls,
past box corners and a cylinder - the point model came apart on exactly
those terms: measured against Gazebo's true pose, a robot driving freely at
the commanded 0.150 m/s read fits swinging -1.82 to +1.72, and 40% of a
30-minute sweep was spent on CRASH recoveries that ground truth says never
happened (true speed 0.150 at ~60% of the triggers).

So each beam's prediction now comes from the surface it actually hit: the
local wall direction is taken from the beam's neighbours in the previous
scan, and the predicted change is -d * n_x / (n . b) for surface normal n
and beam direction b. Beams are dropped when the neighbours are not the same
surface (a range jump, an edge about to slide off) or when the beam grazes
the wall at near-parallel incidence (|n . b| < 0.26), where the division
explodes and the simulated lidar's return is unstable anyway.

WHEN THE SCENE CARRIES NO ANSWER
The least-squares denominator, sum(predicted^2), is exactly how much the
view in front of the lidar is ABLE to say about the claimed motion.
Normalised by claimed^2 it is pure geometry - call it the information term.
When every surviving beam lies on walls parallel to travel it collapses, the
fit becomes a ratio of noise to nothing, and the honest answer is no verdict
at all: poll() reports `degenerate` (same no-verdict family as 'no lidar'
and idle) instead of inventing a stall. A genuinely pinned robot has the
obstacle it hit dead ahead at short range, which is the HIGHEST-information
geometry there is, so abstaining can not hide a real stall - measured below,
the pinned information term never came within 5x of the cutoff.

Measured against Gazebo's true model pose (median over five scan pairs;
12 truth-steered 0.15 m/s rows in the sweep geometry, 11 deliberate pins
against walls, box faces, a cylinder and a corner):

    surface-fit                     n     min     p05     p50     p95
    really moving, wheels 0.15   6551   -1.607  +0.975  +0.998  +1.010
    pinned, wheels 0.15           382   -0.035  -0.005  +0.000  +0.000

    information term                n     min     p05     p50     p95
    really moving, wheels 0.15   6551     142     191     236     382
    pinned, wheels 0.15           382     377     396     591     617

At the same 0.25 fit threshold: 1 sample of 6551 genuine driving under it
(beside a cylinder; a lone 0.1 s dip the confirm window ignores), where the
point model put 967 of the same 6551 under it - 46 sustained would-be
crashes against zero. Every one of the 382 pinned samples read
|fit| <= 0.035, and replayed through poll() all 11 deliberate pins trigger
and no row does. On the live sweep recording whose 3 CRASHes all fired at
a true 0.150 m/s, the surface model fires none of them and instead catches
the one moment the robot really was held (true 0.03 m/s, fit 0.00).
Information cutoff 25: quietest real motion measured 142 (5.7x margin),
quietest pin 377 (15x), so in a normal room it never fires; it exists for
the scene with nothing to fit against, which previously read as a stall.

Still open: beams off the cylinder at (-2.5, -2.0) occasionally pull the
fit down to ~0.25-0.4 for a few pairs (tangent estimation is noisy on a
curved edge). The confirm window absorbs it; if it ever becomes a trigger,
raise MIN_INCIDENCE before touching the threshold.

WHERE THE IMU COMES IN
Two jobs, both real, neither of them "feel the crash":

1. DE-ROTATION. The fit above is derived for a pure translation. If the robot
   also turned between the two scans, every beam has moved to a different
   bearing and the comparison is meaningless. The gyro measures that rotation
   directly, so the previous scan can be rolled back by the angle actually
   turned before differencing. The wheels could not be used for this - they
   are the thing under test, and during a stall their reported turn is exactly
   as fake as their reported speed.

2. BLOCKED ROTATION. A robot wedged with its wheels driving a turn is the
   other half of being stuck, and the gyro answers it outright: it measures
   the turn the wheels only claim. Measured free: 0.538 rad/s against a 0.6
   command, so a real turn lands near 0.9 of the claim and anything under a
   quarter is not turning at all.
"""

import math

# Below this fraction of the claimed motion, the world is not moving past the
# robot. Measured with the surface model: genuine driving sat at +0.975 to
# +1.010 (p05..p95) over 6551 samples and pinned at 0.000 with |fit| never
# above 0.035, so this has a wide margin on both sides.
DEFAULT_FIT_THRESHOLD = 0.25

# Below this information term - sum(predicted^2) / claimed^2, the geometry's
# ability to testify about the claimed motion - the fit is noise over nothing
# and no verdict is given. Measured: the quietest genuine driving carried 142
# and the quietest pin 377 (a wall dead ahead is high information), so 25
# abstains only when the scene really is blind to forward motion.
DEFAULT_MIN_INFO = 25.0

# Beams hitting their surface at near-grazing incidence (|n . b| below this)
# are dropped: the predicted change diverges and the return is unstable.
MIN_INCIDENCE = 0.26

# How many beams to either side the local wall direction is taken from.
TANGENT_SPAN = 2


def _median(values):
    """Return the median of a short sample."""
    ordered = sorted(values)
    count = len(ordered)

    if not count:
        return None

    middle = count // 2

    if count % 2:
        return ordered[middle]

    return 0.5 * (ordered[middle - 1] + ordered[middle])


class Verdict:
    """What the watch currently believes, and why."""

    def __init__(self, stalled, reason='', fit=None, threshold=0.0):
        self.stalled = stalled
        self.reason = reason
        self.fit = fit
        self.threshold = threshold

    def __bool__(self):
        return self.stalled


class StallWatch:
    """
    Report when the wheels claim motion the world does not back up.

    Feed it scans, gyro samples and the wheel-reported speeds; ask it for a
    verdict as often as you like. Deliberately free of ROS types, so the same
    logic runs in the cleaner and in odom_guard rather than being written
    twice and drifting apart.
    """

    def __init__(
        self,
        window=0.6,
        confirm=0.6,
        release=1.0,
        min_speed=0.05,
        min_rate=0.25,
        fit_threshold=DEFAULT_FIT_THRESHOLD,
        min_info=DEFAULT_MIN_INFO,
        rotation_ratio=0.25,
        min_beams=20,
        sensor_timeout=1.0,
    ):
        # Seconds of scan-fit history the median is taken over. Five pairs at
        # 10 Hz, which is what the threshold above was measured against.
        self.window = window

        # How long the wheels and the world must keep disagreeing before this
        # is called a stall, and how long they must agree again before it is
        # called over. Hysteresis, so one noisy pair cannot flip the verdict
        # either way.
        self.confirm = confirm
        self.release = release

        # Below these the wheels are not claiming enough motion to argue about.
        self.min_speed = min_speed
        self.min_rate = min_rate

        self.fit_threshold = fit_threshold
        self.min_info = min_info
        self.rotation_ratio = rotation_ratio

        # Too little of the world in view to fit anything against.
        self.min_beams = min_beams

        # Older than this and the sensor is treated as absent rather than
        # quiet. A dead lidar reports no change at all, which is precisely the
        # stall signature, and would have the robot declare a crash while
        # driving perfectly well.
        self.sensor_timeout = sensor_timeout

        self.fits = []                # (t, fit, info)
        self.gyro = []                # (t, yaw_rate)
        self.last_scan_time = None

        # (ranges, angle_min, angle_increment, range_min, range_max)
        self.previous = None
        self.previous_time = None
        self.yaw_since_scan = 0.0
        self.last_gyro_time = None

        self.wheel_speed = 0.0
        self.wheel_rate = 0.0

        self.stalled = False
        self.disagree_since = None
        self.agree_since = None
        self.last_reason = ''

    # ------------------------------------------------------------------

    def feed_gyro(self, now, yaw_rate):
        """
        Add one gyro sample, and accumulate the angle turned since the scan.

        The accumulated angle is what de-rotates the next scan comparison.
        """
        if self.last_gyro_time is not None:
            step = now - self.last_gyro_time

            # Ignore a wild timestep rather than folding it in: a stalled or
            # jumping clock would otherwise inject a rotation that never
            # happened and corrupt the very comparison this exists to enable.
            if 0.0 < step < 0.5:
                self.yaw_since_scan += yaw_rate * step

        self.last_gyro_time = now

        self.gyro.append((now, yaw_rate))
        self._trim(self.gyro, now)

    def feed_wheels(self, linear, angular):
        """Tell the watch what the wheels are claiming."""
        self.wheel_speed = linear
        self.wheel_rate = angular

    def feed_scan(self, now, ranges, angle_min, angle_increment,
                  range_min, range_max):
        """Compare this scan with the last one and record how far the world moved."""
        previous, previous_time = self.previous, self.previous_time
        turned, self.yaw_since_scan = self.yaw_since_scan, 0.0

        self.previous = (list(ranges), angle_min, angle_increment,
                         range_min, range_max)
        self.previous_time = now
        self.last_scan_time = now

        if previous is None or len(previous[0]) != len(ranges):
            return

        step = now - previous_time
        if not 0.02 < step < 0.5:
            return

        # How far the wheels say the robot travelled between the two scans.
        claimed = self.wheel_speed * step

        # Too small to fit against: the predicted change is below the range
        # quantisation and the answer would be noise.
        if abs(claimed) < 1e-3:
            return

        fitted = self._fit(previous[0], ranges, angle_min, angle_increment,
                           range_min, range_max, claimed, turned)

        if fitted is None:
            return

        self.fits.append((now,) + fitted)
        self._trim(self.fits, now)

    # ------------------------------------------------------------------

    def _fit(self, before, after, angle_min, angle_increment,
             range_min, range_max, claimed, turned):
        """
        Least-squares fraction of the claimed motion the view backs up.

        Returns (fit, information term), or None when too little of the
        world survived the surface checks to fit against.
        """
        count = len(after)

        # Roll the old scan back by the angle the GYRO says the robot turned,
        # so what is left to explain is translation only.
        shift = 0
        if angle_increment:
            shift = int(round(turned / angle_increment))

        # The old scan, de-rotated, with everything out of range marked
        # unusable - the surface direction below reads each beam's
        # neighbours, so validity has to be known for all of them up front.
        old = [float('nan')] * count
        for index in range(count):
            value = before[(index - shift) % count]
            if math.isfinite(value) and range_min <= value <= range_max:
                old[index] = value

        numerator = 0.0
        denominator = 0.0
        used = 0
        span = TANGENT_SPAN

        for index in range(count):
            here = old[index]
            new = after[index]

            if not (math.isfinite(here) and math.isfinite(new)):
                continue

            if not (range_min <= new <= range_max):
                continue

            left = old[(index - span) % count]
            right = old[(index + span) % count]

            # The neighbours must be the SAME surface, or the "wall
            # direction" below would be a line across an edge - and at an
            # edge the beam is about to slide off onto whatever is behind,
            # a jump of metres no motion model predicts.
            jump = 0.05 + 0.05 * here
            if not (math.isfinite(left) and math.isfinite(right)):
                continue
            if abs(left - here) > jump or abs(right - here) > jump:
                continue

            bearing = angle_min + index * angle_increment
            beam_x = math.cos(bearing)
            beam_y = math.sin(bearing)

            left_bearing = bearing - span * angle_increment
            right_bearing = bearing + span * angle_increment

            # Local wall direction through the neighbouring returns, and
            # from it the predicted range change for a forward travel of
            # `claimed`: -d * n_x / (n . b). A wall parallel to travel
            # predicts 0 and drops out of both sums; a wall dead ahead
            # predicts the full claimed motion and anchors them.
            tangent_x = (right * math.cos(right_bearing)
                         - left * math.cos(left_bearing))
            tangent_y = (right * math.sin(right_bearing)
                         - left * math.sin(left_bearing))
            length = math.hypot(tangent_x, tangent_y)

            if length < 1e-6:
                continue

            # n = (-tangent_y, tangent_x) / length
            incidence = (tangent_x * beam_y - tangent_y * beam_x) / length

            # Near-grazing: the division diverges and the return flickers.
            if abs(incidence) < MIN_INCIDENCE:
                continue

            predicted = claimed * (tangent_y / length) / incidence

            numerator += (new - here) * predicted
            denominator += predicted * predicted
            used += 1

        if used < self.min_beams or denominator <= 0.0:
            return None

        return numerator / denominator, denominator / (claimed * claimed)

    def _trim(self, series, now):
        cutoff = now - self.window

        if series and series[0][0] < cutoff:
            series[:] = [item for item in series if item[0] >= cutoff]

    # ------------------------------------------------------------------

    def has_scan(self, now):
        """Whether there is a live lidar to reason about at all."""
        if self.last_scan_time is None:
            return False

        return (now - self.last_scan_time) <= self.sensor_timeout

    def fit(self):
        """Median fraction of the claimed motion the world currently backs up."""
        if len(self.fits) < 3:
            return None

        return _median([value for _, value, _ in self.fits])

    def info(self):
        """
        Median information term: how loudly the scene can answer at all.

        sum(predicted^2) / claimed^2 - pure geometry, independent of speed.
        Below `min_info` the fit is a ratio of noise to nothing and poll()
        abstains rather than reading a stall into it.
        """
        if len(self.fits) < 3:
            return None

        return _median([value for _, _, value in self.fits])

    def turn_rate(self):
        """Mean absolute yaw rate the gyro is actually measuring."""
        if not self.gyro:
            return None

        return sum(abs(rate) for _, rate in self.gyro) / len(self.gyro)

    # ------------------------------------------------------------------

    def poll(self, now):
        """Return the current verdict, with hysteresis applied."""
        if not self.has_scan(now):
            self._reset()
            return Verdict(False, 'no lidar')

        reason = ''
        measured_fit = self.fit()

        driving = abs(self.wheel_speed) > self.min_speed
        turning = abs(self.wheel_rate) > self.min_rate

        if driving and measured_fit is not None:
            # A scene that carries no information about forward motion -
            # every surviving beam on walls parallel to travel - cannot
            # testify either way, and a fit taken over it is noise. Abstain,
            # in the same no-verdict family as 'no lidar': the caller can
            # show it, but it is never a crash. A genuine pin has its
            # obstacle dead ahead at short range, the highest-information
            # geometry there is (measured: pins carried 377+ against a
            # cutoff of 25), so this cannot swallow a real stall.
            measured_info = self.info()

            if measured_info is not None and measured_info < self.min_info:
                settled = self._settle(now, '', measured_fit)

                if settled.stalled:
                    return settled

                return Verdict(
                    False,
                    f'degenerate view (information {measured_info:.0f} < '
                    f'{self.min_info:.0f})',
                    measured_fit, self.fit_threshold)

            # MAGNITUDE, not the signed value. A stall is the room standing
            # still, which is a fit of ~0; a large NEGATIVE fit means the room
            # swept past several times faster than claimed and in the other
            # direction, which is a robot that is very much moving - being
            # shoved back off an obstacle, or reversing while the wheel
            # reading lags. Testing `fit < threshold` called those crashes
            # too, and they were the ones that fired at -4.14 and -8.57
            # against a +0.09 m/s claim.
            if abs(measured_fit) < self.fit_threshold:
                reason = (
                    f'wheels report {self.wheel_speed:+.2f} m/s but the view '
                    f'only backs {measured_fit:+.2f} of it '
                    f'(|fit| < {self.fit_threshold:.2f})'
                )

        if not reason and turning:
            measured_rate = self.turn_rate()

            if measured_rate is not None:
                if measured_rate < self.rotation_ratio * abs(self.wheel_rate):
                    reason = (
                        f'wheels report {self.wheel_rate:+.2f} rad/s but the '
                        f'gyro measures {measured_rate:.3f} rad/s'
                    )

        return self._settle(now, reason, measured_fit)

    def _settle(self, now, reason, measured_fit):
        """Apply the confirm and release windows to a raw reading."""
        if reason:
            self.agree_since = None

            if self.disagree_since is None:
                self.disagree_since = now

            if not self.stalled and now - self.disagree_since >= self.confirm:
                self.stalled = True
                self.last_reason = reason

                return Verdict(True, reason, measured_fit, self.fit_threshold)

        else:
            self.disagree_since = None

            if self.agree_since is None:
                self.agree_since = now

            if self.stalled and now - self.agree_since >= self.release:
                self.stalled = False
                self.last_reason = ''

        return Verdict(self.stalled, self.last_reason, measured_fit,
                       self.fit_threshold)

    def _reset(self):
        self.stalled = False
        self.disagree_since = None
        self.agree_since = None
        self.last_reason = ''
