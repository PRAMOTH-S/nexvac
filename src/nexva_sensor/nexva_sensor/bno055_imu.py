"""
BNO055 on the Raspberry Pi's I2C bus, published as sensor_msgs/Imu.

STARTED BY BRINGUP. `imu.launch.py` now defaults `enable` to true and
`bringup.launch.py` includes it, so the board comes up with the robot. A
missing board or missing libraries is logged and the node keeps running - see
`connect()` - so bringup never fails over an unwired IMU. See the launch file
for the wiring, install and bench-check sequence.

ACCEL-ONLY IS THE DEFAULT. `accel_only` (default true) publishes the
accelerometer and the gyro and marks the fused orientation unusable
(orientation_covariance[0] = -1, REP-145) whatever the calibration counters
say, and skips the magnetometer publisher entirely. The magnetometer is what
makes the fused heading confidently wrong when uncalibrated, and nothing on
this robot has calibrated it. `accel_only:=false` restores the previous
behaviour exactly: mag published, orientation trusted once the counters reach
`min_calibration`.

Ported from the vac_main1 simulation workspace, where it replaced a Gazebo IMU
plugin on the real robot. Wiring is documented in Docs/circuit.txt section 3:
four wires to the Pi header, address 0x28.

It does NOT feed an EKF. Fusing this IMU with the wheels was tried on
vac_main1 and measured far worse than the wheels alone - 109 deg mean heading
error against 12 - and was removed. The value here is an absolute heading that
does not drift, useful for noticing when the wheels are lying. Do not wire it
into robot_localization without re-measuring the map first.

What this adds over reading the board in a loop (`imutest.py`):

- **Every reading can come back None.** The Adafruit library returns None for
  a whole vector when an I2C transaction is disturbed, and on a Pi sharing the
  bus that happens. Printed in a terminal it flickers past; published into a
  ROS message it becomes a NaN quaternion that poisons every consumer. Each
  read is checked and a bad one is counted and skipped, not published.

- **Calibration is reported, not ignored.** The BNO055 fuses magnetometer
  into its absolute orientation, and until the magnetometer is calibrated
  that orientation is confidently wrong. The chip tells you this - four
  counters, 0 to 3 - and the node publishes them and refuses to claim a
  trustworthy heading below `min_calibration`. An uncalibrated BNO055 does not
  look broken; it looks fine and points the wrong way.

- **Axes are declared, not assumed.** The chip's own frame depends on how the
  board is physically mounted. ROS wants REP-103: x forward, y left, z up.
  `axis_remap` maps one to the other and defaults to the identity, which is
  correct only if the board is mounted with its x pointing forward. Verify it
  on the bench with `ros2 run nexva_sensor imu_monitor` - it is a thing to
  check with the robot in front of you, not to guess in a launch file.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String

# The IMU is a stream: the newest sample is the only one worth having, and a
# late one is worse than none. Best effort, depth 1, matching what odom_guard
# subscribes with.
IMU_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

# Datasheet figures, converted. These are what the covariance matrices are
# built from; they are the sensor's own noise, not a guess.
ORIENTATION_STDDEV = math.radians(2.0)      # +/- 2 deg absolute, fused
ANGULAR_STDDEV = math.radians(0.1)          # gyro noise
LINEAR_STDDEV = 0.05                        # m/s^2

CALIBRATION_NAMES = ('system', 'gyroscope', 'accelerometer', 'magnetometer')


class Bno055Imu(Node):
    """Reads a BNO055 over I2C and publishes it as a ROS IMU."""

    def __init__(self):
        super().__init__('bno055_imu')

        # base_link, not imu_link: nexva.urdf.xacro carries no imu_link, so a
        # message stamped with one would have no transform and every consumer
        # would silently drop it. Add the link with its measured mount offset,
        # then set this to imu_link.
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('address', 0x28)

        # Which physical I2C bus. 1 is the Pi's default header bus, the one
        # behind pins 3 and 5.
        self.declare_parameter('bus', 1)

        # Reorder the chip's axes into REP-103. Three signed 1-based indices:
        # [1, 2, 3] is the identity, [2, -1, 3] means "the chip's y is our x,
        # the chip's x is our -y". Verify it, do not guess it.
        self.declare_parameter('axis_remap', [1, 2, 3])

        # Below this the fused heading is not trustworthy. 3 is fully
        # calibrated; the chip reaches it for the gyro almost at once, and for
        # the magnetometer only after the robot has been turned around a bit.
        self.declare_parameter('min_calibration', 2)

        # Accelerometer (and gyro) only. The fused orientation is published
        # but permanently flagged unusable, and the magnetometer - the part
        # that poisons that orientation - is not published at all. This is the
        # default because no one has calibrated this board yet, and an
        # uncalibrated BNO055 does not look broken, it looks fine and points
        # the wrong way. stall_guard consumes exactly these two fields.
        self.declare_parameter('accel_only', True)

        self.declare_parameter('publish_mag', True)
        self.declare_parameter('publish_temperature', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.rate = float(self.get_parameter('rate').value)
        self.address = int(self.get_parameter('address').value)
        self.bus = int(self.get_parameter('bus').value)
        self.remap = list(self.get_parameter('axis_remap').value)
        self.min_calibration = int(self.get_parameter('min_calibration').value)
        self.accel_only = bool(self.get_parameter('accel_only').value)

        self.validate_remap()

        self.imu_pub = self.create_publisher(Imu, 'imu', IMU_QOS)
        self.status_pub = self.create_publisher(String, 'imu/status', 10)

        # accel_only wins over publish_mag: the whole point of the mode is
        # that the magnetometer is not to be relied on, and a topic nobody
        # should read is better absent than present.
        self.mag_pub = (
            self.create_publisher(MagneticField, 'imu/mag', IMU_QOS)
            if self.get_parameter('publish_mag').value
            and not self.accel_only else None)
        self.temp_pub = (
            self.create_publisher(Temperature, 'imu/temperature', 10)
            if self.get_parameter('publish_temperature').value else None)

        self.sensor = None
        self.published = 0
        self.dropped = 0
        self.last_calibration = None
        self.warned_uncalibrated = False

        self.connect()

        self.create_timer(1.0 / self.rate, self.read_once)
        self.create_timer(5.0, self.report)

    # ------------------------------------------------------------------

    def validate_remap(self):
        """Reject an axis_remap that is not a signed permutation of 1..3."""
        ok = (len(self.remap) == 3
              and sorted(abs(int(v)) for v in self.remap) == [1, 2, 3]
              and all(int(v) != 0 for v in self.remap))

        if not ok:
            self.get_logger().error(
                f'axis_remap {self.remap} is not a signed permutation of '
                f'1,2,3 - falling back to the identity. Valid examples: '
                f'[1, 2, 3], [2, -1, 3], [-1, -2, 3]'
            )
            self.remap = [1, 2, 3]

    def connect(self):
        """
        Open the board, saying precisely what is missing when it fails.

        The three ways this fails look identical from a stack trace - no
        library, no bus, no device - and need three different fixes, so they
        are separated here.
        """
        try:
            import board
            import busio
            import adafruit_bno055
        except ImportError as exc:
            self.get_logger().error(
                f'BNO055 libraries not installed ({exc}). On the Pi:\n'
                f'    sudo apt install -y python3-pip i2c-tools\n'
                f'    pip3 install --break-system-packages '
                f'adafruit-circuitpython-bno055 adafruit-blinka'
            )
            return

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=self.address)
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().error(
                f'Cannot reach a BNO055 at 0x{self.address:02x} on bus '
                f'{self.bus}: {exc}\n'
                f'    check wiring:  i2cdetect -y {self.bus}\n'
                f'    the board should show as 28 (or 29 if ADR is pulled '
                f'high)\n'
                f'    if the bus itself is missing, enable it: '
                f'raspi-config > Interface Options > I2C'
            )
            return

        self.get_logger().info(
            f'BNO055 at 0x{self.address:02x} on i2c-{self.bus}, publishing '
            f'imu at {self.rate:.0f} Hz in frame {self.frame_id}'
            + (' (accel_only: orientation flagged unusable, no magnetometer)'
               if self.accel_only else '')
        )

    # ------------------------------------------------------------------

    def apply_remap(self, vector):
        """Reorder and sign one xyz triple according to axis_remap."""
        out = []
        for entry in self.remap:
            index = abs(int(entry)) - 1
            value = vector[index]
            out.append(-value if entry < 0 else value)
        return out

    def read_once(self):
        """Read the board and publish one sample, or count a failure."""
        if self.sensor is None:
            return

        try:
            quaternion = self.sensor.quaternion
            gyro = self.sensor.gyro
            accel = self.sensor.linear_acceleration
            calibration = self.sensor.calibration_status
        except OSError as exc:
            # An I2C read that failed outright rather than returning None.
            self.dropped += 1
            self.get_logger().warn(
                f'I2C read failed: {exc}', throttle_duration_sec=5.0)
            return

        # The library returns None - for the whole tuple or for a component -
        # when a transaction is disturbed. Publishing that as a quaternion
        # gives every consumer a NaN.
        if not self.usable(gyro, 3) or not self.usable(accel, 3):
            self.dropped += 1
            return

        # The fused quaternion is only load-bearing when it is being trusted.
        # In accel_only it is already flagged unusable, so a missing one is no
        # reason to throw away a perfectly good acceleration sample - which is
        # the one thing stall_guard needs. Identity + covariance -1 says
        # "no orientation here" without a NaN.
        orientation_ok = self.usable(quaternion, 4)
        if not orientation_ok:
            if not self.accel_only:
                self.dropped += 1
                return
            quaternion = (1.0, 0.0, 0.0, 0.0)

        self.track_calibration(calibration)

        now = self.get_clock().now().to_msg()

        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        # Adafruit returns (w, x, y, z); ROS wants x, y, z, w.
        w, x, y, z = quaternion
        rx, ry, rz = self.apply_remap([x, y, z])
        msg.orientation.x = float(rx)
        msg.orientation.y = float(ry)
        msg.orientation.z = float(rz)
        msg.orientation.w = float(w)

        gx, gy, gz = self.apply_remap(list(gyro))
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        ax, ay, az = self.apply_remap(list(accel))
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        # If the fused orientation is not calibrated yet, say so in the
        # message rather than only in a log line nothing reads. -1 in the
        # first element is REP-145's "do not use this field".
        #
        # In accel_only this is never trusted, however good the counters look.
        # The heading comes out of a fusion that includes a magnetometer the
        # operator has not calibrated, and a consumer that reads a covariance
        # instead of a README must be told no.
        trusted = (
            not self.accel_only
            and orientation_ok
            and self.calibration_ok(calibration))
        msg.orientation_covariance[0] = (
            ORIENTATION_STDDEV ** 2 if trusted else -1.0)
        if trusted:
            msg.orientation_covariance[4] = ORIENTATION_STDDEV ** 2
            msg.orientation_covariance[8] = ORIENTATION_STDDEV ** 2

        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = ANGULAR_STDDEV ** 2
            msg.linear_acceleration_covariance[i] = LINEAR_STDDEV ** 2

        self.imu_pub.publish(msg)
        self.published += 1

        self.publish_extras(now)

    @staticmethod
    def usable(reading, length):
        """Check a reading is present and every component is a real number."""
        if reading is None or len(reading) != length:
            return False
        return all(v is not None and math.isfinite(v) for v in reading)

    def publish_extras(self, stamp):
        """Magnetometer and temperature, if anyone asked for them."""
        if self.mag_pub is not None:
            mag = self.sensor.magnetic
            if self.usable(mag, 3):
                mx, my, mz = self.apply_remap(list(mag))
                msg = MagneticField()
                msg.header.stamp = stamp
                msg.header.frame_id = self.frame_id
                # The board reports microtesla; the message wants tesla.
                msg.magnetic_field.x = float(mx) * 1e-6
                msg.magnetic_field.y = float(my) * 1e-6
                msg.magnetic_field.z = float(mz) * 1e-6
                self.mag_pub.publish(msg)

        if self.temp_pub is not None:
            temperature = self.sensor.temperature
            if temperature is not None:
                msg = Temperature()
                msg.header.stamp = stamp
                msg.header.frame_id = self.frame_id
                msg.temperature = float(temperature)
                self.temp_pub.publish(msg)

    def track_calibration(self, calibration):
        """Announce calibration changes, and say what to do about them."""
        if calibration is None or len(calibration) != 4:
            return

        calibration = tuple(int(v) for v in calibration)
        if calibration == self.last_calibration:
            return

        self.last_calibration = calibration

        message = String()
        message.data = ' '.join(
            f'{name}={value}/3'
            for name, value in zip(CALIBRATION_NAMES, calibration))
        self.status_pub.publish(message)

        if self.calibration_ok(calibration):
            if self.warned_uncalibrated:
                self.get_logger().info(f'IMU calibrated: {message.data}')
                self.warned_uncalibrated = False
            return

        if not self.warned_uncalibrated:
            self.warned_uncalibrated = True
            if self.accel_only:
                self.get_logger().warn(
                    f'IMU not calibrated yet ({message.data}). accel_only is '
                    f'on, so the heading is already published as unusable and '
                    f'the magnetometer counter can be ignored - but gyro and '
                    f'accelerometer must still reach {self.min_calibration} '
                    f'or the acceleration carries an unsettled zero-g offset '
                    f'and stall_guard is arguing with a bias. Gyro: hold the '
                    f'robot still for a few seconds. Accelerometer: rest it '
                    f'on each of a few different faces.'
                )
                return

            self.get_logger().warn(
                f'IMU not calibrated yet ({message.data}). The heading it '
                f'reports is confidently wrong until it is. Gyro: hold the '
                f'robot still for a few seconds. Accelerometer: rest it on '
                f'each of a few different faces. Magnetometer: turn it slowly '
                f'through a figure of eight. orientation_covariance stays -1 '
                f'until all three reach {self.min_calibration}.'
            )

    def calibration_ok(self, calibration):
        """
        Are the counters this mode depends on at min_calibration?

        accel_only never uses the fused heading, so the magnetometer counter
        is not its business - demanding it would leave the node warning for
        ever about a sensor it has deliberately switched off. Gyro and
        accelerometer still matter: the accelerometer's own counter is what
        says its zero-g offsets have settled, and stall_guard's whole argument
        rests on that offset being stable.
        """
        if calibration is None or len(calibration) != 4:
            return False
        wanted = calibration[1:3] if self.accel_only else calibration[1:]
        return min(wanted) >= self.min_calibration

    def report(self):
        """Periodic health line, so a silent IMU is visible."""
        if self.sensor is None:
            self.get_logger().warn(
                'No BNO055 - imu is not being published',
                throttle_duration_sec=30.0)
            return

        total = self.published + self.dropped
        if total == 0:
            return

        rate = self.published / 5.0
        if self.dropped:
            self.get_logger().warn(
                f'imu {rate:.1f} Hz, {self.dropped} of {total} reads dropped '
                f'({100.0 * self.dropped / total:.1f}%) - check the I2C '
                f'wiring and that nothing else is hammering the bus'
            )

        self.published = 0
        self.dropped = 0


def main(args=None):
    rclpy.init(args=args)
    node = Bno055Imu()

    # ExternalShutdownException is what a SIGTERM looks like from in here -
    # `timeout`, a launch file tearing down, systemd. Without catching it the
    # node exits through a traceback every time it is stopped normally, and
    # the shutdown below then fails a second time on an already-dead context.
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
