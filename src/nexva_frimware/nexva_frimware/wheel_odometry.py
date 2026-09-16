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
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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

        self.cpr = self.get_parameter('encoder_cpr').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.reset_threshold = self.get_parameter('reset_threshold_m').value

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

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

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

        self.theta += delta_theta
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        linear_velocity = distance / dt if dt > 0.0 else 0.0
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
