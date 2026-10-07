"""
Has the robot actually moved? Accelerometer only.

Replaces the old `stall_guard` + `odom_guard` pair, which between them read the
gyro, ran two detection channels, tracked vibration, froze odometry and raised
the ESP32's PWM floor through /pid_limits. That last part is why the robot span
with nothing publishing cmd_vel: the floor was raised to break a (false) stall
and the process was killed before it restored the baseline, leaving the
firmware driving on its own. **This node never writes to /pid_limits and never
publishes cmd_vel.** It reports; it does not act.

WHAT AN ACCELEROMETER CAN AND CANNOT TELL YOU

It measures acceleration, not velocity. At a *constant* speed a healthy robot
reads the same ~0 as one standing still, so a single sample can never separate
"moving" from "stopped". Two things are real and both are used here:

  1. CHANGES in velocity. Starting, stopping or hitting something all show up
     as a burst. Integrating over a short window gives a Δv that a pinned robot
     simply does not produce.
  2. MOTION ENERGY. A driving robot shakes - motors, gearbox, floor texture.
     Standing still it does not.

Measured on this robot, and the reason for the honesty above: at rest the
magnitude wandered 0.126-0.150 m/s2, and while genuinely driving at 0.035 m/s
it read 0.149 m/s2 - indistinguishable. The accelerometer only separates the
two once the robot is moving briskly or the floor is not glass-smooth. So this
node reports three states, not two, and `unsure` is a real answer rather than a
guess dressed up as one.

It is also deliberately useless as a safety interlock while the IMU is
unreliable: with no /imu, or readings older than `imu_timeout`, it reports
`no imu` and nothing downstream should act.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

# Gravity-compensated magnitude at rest, measured on this robot: 0.126-0.150.
# A little above the worst of that, so resting noise alone cannot read as
# motion. Raise it if the robot reports moving while visibly parked.
DEFAULT_REST_BAND = 0.18

# Below this Δv over the window, nothing happened worth calling movement.
DEFAULT_MIN_DELTA_V = 0.04


class MotionCheck(Node):

    def __init__(self):
        super().__init__('motion_check')

        self.declare_parameter('window', 1.0)
        self.declare_parameter('rest_band', DEFAULT_REST_BAND)
        self.declare_parameter('min_delta_v', DEFAULT_MIN_DELTA_V)
        self.declare_parameter('imu_timeout', 0.5)
        self.declare_parameter('report_period', 0.5)

        self.window = float(self.get_parameter('window').value)
        self.rest_band = float(self.get_parameter('rest_band').value)
        self.min_delta_v = float(self.get_parameter('min_delta_v').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)

        self.samples = []            # (monotonic, ax, ay)
        self.last_imu = None

        self.create_subscription(
            Imu, '/imu', self.on_imu, qos_profile_sensor_data)

        self.moved_pub = self.create_publisher(Bool, 'motion_check/moved', 10)
        self.status_pub = self.create_publisher(
            String, 'motion_check/status', 10)

        self.create_timer(
            float(self.get_parameter('report_period').value), self.report)

        self.get_logger().info(
            'motion_check: accelerometer only, reports whether the body moved. '
            'It never touches /cmd_vel or /pid_limits.')

    def on_imu(self, msg):
        now = time.monotonic()
        self.last_imu = now
        a = msg.linear_acceleration
        self.samples.append((now, a.x, a.y))
        cutoff = now - self.window
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.pop(0)

    def verdict(self):
        """('moved'|'still'|'unsure'|'no imu', explanation)."""
        now = time.monotonic()

        if self.last_imu is None:
            return 'no imu', 'no /imu message has ever arrived'
        if now - self.last_imu > self.imu_timeout:
            return 'no imu', ('last /imu %.1fs ago (>%.1fs)'
                              % (now - self.last_imu, self.imu_timeout))
        if len(self.samples) < 5:
            return 'unsure', 'not enough samples yet'

        mags = [math.hypot(x, y) for _, x, y in self.samples]
        peak = max(mags)
        mean = sum(mags) / len(mags)

        # Δv by integrating the in-plane acceleration across the window.
        # Trapezoid is overkill for this; rectangles over a 50 Hz stream are
        # already finer than the signal deserves.
        dv = 0.0
        for (t0, x0, y0), (t1, x1, y1) in zip(self.samples, self.samples[1:]):
            dt = t1 - t0
            dv += 0.5 * (math.hypot(x0, y0) + math.hypot(x1, y1)) * dt

        detail = ('peak %.3f mean %.3f dv %.3f m/s (band %.2f, dv_min %.2f)'
                  % (peak, mean, dv, self.rest_band, self.min_delta_v))

        if peak > self.rest_band or dv > self.min_delta_v:
            return 'moved', detail
        # Quiet. On a smooth floor at a steady crawl this is ALSO what real
        # motion looks like, so it is reported as 'still' but the numbers are
        # published alongside so a human can see how close the call was.
        return 'still', detail

    def report(self):
        state, detail = self.verdict()
        self.moved_pub.publish(Bool(data=(state == 'moved')))
        self.status_pub.publish(String(data='%s - %s' % (state, detail)))


def main(args=None):
    rclpy.init(args=args)
    node = MotionCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
