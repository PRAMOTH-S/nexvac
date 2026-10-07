"""
Live BNO055 readout in a terminal, from the ROS topics.

`i2cdetect -y 1` on the Pi answers "is the board wired up at all". This is the
other half: the same picture taken from `/imu`, so it can be run **from the
laptop** while the robot is doing something, and so what you are looking at is
what the rest of the system is actually receiving rather than a second
independent read of the chip. If this disagrees with a direct I2C read, the
problem is in the node or the axis remap, not the wiring.

This is the tool for steps 5 and 6 of the bring-up sequence in
nexva_sensor/launch/imu.launch.py: turning the robot left must raise yaw, and
the calibration line is the one to watch. A BNO055 that is not calibrated does
not look broken - it looks fine and points the wrong way.

    ros2 run nexva_sensor imu_monitor
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String

IMU_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)


class ImuMonitor(Node):
    """One screen of live IMU state."""

    def __init__(self):
        super().__init__('imu_monitor')

        self.declare_parameter('refresh', 0.2)
        self.declare_parameter('plain', False)

        self.plain = bool(self.get_parameter('plain').value)

        self.imu = None
        self.mag = None
        self.temperature = None
        self.status = 'waiting'

        self.count = 0
        self.last_count = 0
        self.rate = 0.0
        self.last_stamp = None

        self.create_subscription(Imu, 'imu', self.on_imu, IMU_QOS)
        self.create_subscription(String, 'imu/status', self.on_status, 10)
        self.create_subscription(MagneticField, 'imu/mag', self.on_mag, IMU_QOS)
        self.create_subscription(
            Temperature, 'imu/temperature', self.on_temperature, 10)

        refresh = float(self.get_parameter('refresh').value)
        self.create_timer(refresh, self.draw)
        self.create_timer(1.0, self.tick_rate)

    def on_imu(self, msg):
        self.imu = msg
        self.count += 1
        self.last_stamp = self.get_clock().now()

    def on_status(self, msg):
        self.status = msg.data

    def on_mag(self, msg):
        self.mag = msg

    def on_temperature(self, msg):
        self.temperature = msg.temperature

    def tick_rate(self):
        self.rate = float(self.count - self.last_count)
        self.last_count = self.count

    @staticmethod
    def euler(q):
        """Quaternion to roll, pitch, yaw in degrees."""
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = (math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0
                 else math.asin(sinp))

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def draw(self):
        lines = []
        add = lines.append

        add('  nexva - BNO055')
        add('  ' + '-' * 54)

        if self.imu is None:
            add('  waiting for /imu ...')
            add('')
            add('  nothing is publishing. The driver is off by default:')
            add('    ros2 launch nexva_sensor imu.launch.py enable:=true')
            add('  Check the board itself with:   i2cdetect -y 1')
            self.render(lines)
            return

        silent = ''
        if self.last_stamp is not None:
            age = (self.get_clock().now() - self.last_stamp).nanoseconds / 1e9
            if age > 1.0:
                silent = f'   SILENT for {age:.1f} s'

        add(f'  link      {self.rate:.0f} Hz   {self.count} samples{silent}')
        add('')

        roll, pitch, yaw = self.euler(self.imu.orientation)
        trusted = self.imu.orientation_covariance[0] >= 0.0
        add(f'  heading   yaw {yaw:+8.2f} deg'
            + ('' if trusted else '   NOT TRUSTED, see calibration'))
        add(f'            roll {roll:+7.2f}   pitch {pitch:+7.2f}')
        add('')

        g = self.imu.angular_velocity
        add(f'  gyro      x {g.x:+7.3f}  y {g.y:+7.3f}  z {g.z:+7.3f}  rad/s')

        a = self.imu.linear_acceleration
        add(f'  accel     x {a.x:+7.3f}  y {a.y:+7.3f}  z {a.z:+7.3f}  m/s2')

        if self.mag is not None:
            m = self.mag.magnetic_field
            add(f'  mag       x {m.x * 1e6:+7.1f}  y {m.y * 1e6:+7.1f}'
                f'  z {m.z * 1e6:+7.1f}  uT')

        if self.temperature is not None:
            add(f'  board     {self.temperature:.0f} C')

        add('')
        add(f'  calib     {self.status}')

        if not trusted:
            add('')
            add('  gyro: hold it still.  accel: rest it on a few faces.')
            add('  mag: turn it slowly through a figure of eight.')

        self.render(lines)

    def render(self, lines):
        if self.plain:
            print(' | '.join(line.strip() for line in lines if line.strip()),
                  flush=True)
            return

        print('\033[2J\033[H', end='')
        print('\n'.join(lines), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = ImuMonitor()

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
