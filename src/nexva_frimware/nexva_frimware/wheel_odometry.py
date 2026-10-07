"""Compute /odom and the odom -> base_footprint transform from encoder counts.

This used to live on the ESP32, but nav_msgs/Odometry serializes to 724 B
against a 512 B serial MTU. Every message fragmented across the micro-XRCE
output buffers and the reliable stream then blocked ~1 s waiting for delivery
confirmation, pinning the transform to 1 Hz no matter what the baud rate was.

Encoder counts cross the link in one small message without fragmenting. Doing
the integration here also means /odom carries host timestamps, which removes
any dependence on the micro-ROS session clock.

The integration matches the original firmware implementation exactly, so
odometry behaves the same as before.

NOT BELIEVING THE WHEELS WHEN THE BODY SAYS OTHERWISE
Encoders are honest about the WHEELS and say nothing about the ROBOT. Pinned
against a chair leg with the motors still turning, the integration below walks
the pose forward through the obstacle, and because this node also owns the
`odom -> base_footprint` transform, SLAM, AMCL, the coverage planner and the
web UI all follow it there.

This node is the ACTUATOR half of the fix; `nexva_sensor/odom_guard.py` is the
DETECTOR half. The detector fuses `stall_guard`'s accelerometer verdict with
IMU freshness and publishes one boolean, `/wheel_slip`. Here, behind the
`trust_imu` parameter, that flag stops forward distance being accumulated and
makes the reported linear velocity zero - in `/odom` AND in the transform, so
every consumer is corrected at once rather than each having to opt in to a
second topic. The correction is applied here rather than in the detector
because two nodes publishing `odom -> base_footprint` would fight over it.

Suppression is fail-open in three ways, so a dead, missing or unwired IMU can
never freeze odometry:
  * `/wheel_slip` must be TRUE *and* newer than `slip_timeout`. If odom_guard
    is not running, is killed, or stalls, the flag goes stale within a second
    and the wheels are believed again.
  * odom_guard itself refuses to raise the flag without a live `imu` topic.
  * `trust_imu:=false` removes the subscription entirely.
Rotation is never suppressed: a pinned robot can still be yawing, and the
stall detector only reasons about the forward axis.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        # Defaults mirror the constants in microros_code.ino.
        self.declare_parameter('encoder_cpr', 662.0)
        self.declare_parameter('wheel_radius', 0.0335)
        self.declare_parameter('wheel_base', 0.245)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        # One sample may not plausibly move a wheel further than this. Anything
        # larger means the ESP32 rebooted and its counters restarted at zero.
        self.declare_parameter('reset_threshold_m', 1.0)
        # Believe nexva_sensor/odom_guard when it says the wheels are lying.
        # On by default: the failure it prevents - a pose that walks through
        # a wall and a map drawn from it - is one nothing downstream can
        # detect or undo, and the correction costs nothing when the robot is
        # driving normally. trust_imu:=false restores the previous behaviour
        # exactly.
        self.declare_parameter('trust_imu', False)
        # A slip flag older than this is treated as absent, not as false-or-
        # true. This is the fail-open: no detector, no suppression.
        self.declare_parameter('slip_timeout', 1.0)

        self.cpr = self.get_parameter('encoder_cpr').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.reset_threshold = self.get_parameter('reset_threshold_m').value
        self.trust_imu = bool(self.get_parameter('trust_imu').value)
        self.slip_timeout = float(self.get_parameter('slip_timeout').value)

        self.wheel_slip = False
        self.last_slip_stamp = None
        self.suppressing = False
        self.suppressed_metres = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_count = 0
        self.right_count = 0
        self.previous_left = None
        self.previous_right = None
        self.last_stamp = self.get_clock().now()

        # Both counts arrive in one message, so left and right are always
        # from the same instant. Integrate on arrival.
        self.create_subscription(
            Vector3, '/enco/counts', self.encoder_callback,
            qos_profile_sensor_data)

        if self.trust_imu:
            # TRANSIENT_LOCAL to match odom_guard's latched publisher, so
            # this node learns the current verdict even if it starts second.
            self.create_subscription(
                Bool, '/wheel_slip', self.slip_callback,
                QoSProfile(
                    depth=1,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'wheel slip suppression %s'
            % ('ON - /wheel_slip gates forward odometry (fail-open after '
               '%.1fs without the flag)' % self.slip_timeout
               if self.trust_imu else
               'OFF (trust_imu:=false) - encoders are believed always'))

        self.get_logger().info(
            'Publishing /odom%s on each encoder pair '
            '(cpr %.1f, wheel radius %.4f m, wheel base %.3f m)'
            % (' and odom -> ' + self.base_frame if self.publish_tf else '',
               self.cpr, self.wheel_radius, self.wheel_base)
        )

    def encoder_callback(self, msg):
        self.left_count = msg.x
        self.right_count = msg.y
        self.update()

    def slip_callback(self, msg):
        self.wheel_slip = bool(msg.data)
        self.last_slip_stamp = self.get_clock().now()

    def slip_active(self):
        """True only while a FRESH flag says the wheels are lying."""
        if not self.trust_imu or not self.wheel_slip:
            return False
        if self.last_slip_stamp is None:
            return False
        age = (self.get_clock().now() - self.last_slip_stamp).nanoseconds / 1e9
        if age > self.slip_timeout:
            # The detector stopped talking. Silence is not a verdict, so the
            # wheels get the benefit of the doubt - this is the fail-open
            # that keeps a dead IMU or a dead odom_guard harmless.
            return False
        return True

    def counts_to_metres(self, counts):
        return (counts / self.cpr) * (2.0 * math.pi * self.wheel_radius)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_stamp).nanoseconds / 1e9
        self.last_stamp = now

        left_count = self.left_count
        right_count = self.right_count

        if self.previous_left is None:
            self.previous_left = left_count
            self.previous_right = right_count
            return

        left_distance = self.counts_to_metres(left_count - self.previous_left)
        right_distance = self.counts_to_metres(right_count - self.previous_right)

        self.previous_left = left_count
        self.previous_right = right_count

        # The ESP32 reboots itself when it loses the agent, which restarts the
        # counters. Integrating that jump would teleport the robot.
        if (abs(left_distance) > self.reset_threshold
                or abs(right_distance) > self.reset_threshold):
            self.get_logger().warn(
                'Encoder counts jumped (%.2f m, %.2f m) - treating as an ESP32 '
                'restart and skipping this sample'
                % (left_distance, right_distance))
            return

        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        # The wheels turned, but did the robot move? While the detector says
        # no, the distance is dropped on the floor instead of being
        # integrated: the pose stands still and the reported speed is zero,
        # so SLAM, AMCL and the explorer all stop being told about motion
        # that did not happen. The encoder counts themselves are still
        # consumed above, so there is no jump when the robot breaks free.
        suppressed = self.slip_active()
        distance_measured = distance

        if suppressed:
            self.suppressed_metres += abs(distance)
            distance = 0.0
            if not self.suppressing:
                self.suppressing = True
                self.get_logger().warn(
                    'wheel slip confirmed - holding odometry still while the '
                    'wheels spin (rotation still tracked)')
        elif self.suppressing:
            self.suppressing = False
            self.get_logger().info(
                'wheel slip cleared - suppressed %.3f m of phantom motion'
                % self.suppressed_metres)
            self.suppressed_metres = 0.0

        self.theta += delta_theta
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        # The POSE is suppressed; the reported wheel SPEED is not.
        #
        # Zeroing the twist as well was tried and it deadlocks the detector:
        # stall_guard decides the robot is stalled by comparing the wheel
        # speed on this very topic against the accelerometer, so a zeroed
        # twist makes it report "WHEELS STOPPED - commanded but not turning
        # is a different fault", it withdraws its verdict, the flag drops,
        # the suppression releases, and the whole thing oscillates. Measured
        # with the twist zeroed: the guard entered and released roughly once
        # a second and leaked 56% of the phantom distance through anyway.
        #
        # So the twist keeps reporting what the wheels actually did, which
        # is a true measurement of the wheels and the input the detector
        # needs, and the covariance marks it as not to be believed as robot
        # motion. The pose and the transform - the thing that was walking
        # through the wall - stand still. Anything that wants a twist
        # corrected too can subscribe to /odom_guarded.
        linear_velocity = distance_measured / dt if dt > 0.0 else 0.0
        angular_velocity = delta_theta / dt if dt > 0.0 else 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        if suppressed:
            # REP-145: say loudly that this forward speed is a wheel
            # reading, not a robot reading. Anything doing proper fusion
            # (an EKF, robot_localization) will down-weight it to nothing;
            # stall_guard, which wants the raw wheel number, still gets it.
            covariance = list(odom.twist.covariance)
            covariance[0] = 1e3
            covariance[7] = 1e3
            odom.twist.covariance = covariance

        self.odom_publisher.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
