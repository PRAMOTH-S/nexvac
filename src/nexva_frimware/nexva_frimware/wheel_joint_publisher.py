"""Turn the ESP32's raw encoder counts into /joint_states.

robot_state_publisher needs positions for the two continuous wheel joints;
without them it never emits base_link -> left_wheel_1 / right_wheel_1, so the
wheels are missing from the robot model and RViz logs TF errors for them.

Deriving the joint states here rather than publishing them from the ESP32
keeps them off the serial link, which is already the bottleneck for /odom.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3


class WheelJointPublisher(Node):

    def __init__(self):
        super().__init__('wheel_joint_publisher')

        # Must match ENCODER_CPR in the ESP32 firmware.
        self.declare_parameter('encoder_cpr', 662.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')

        self.cpr = self.get_parameter('encoder_cpr').value
        self.left_name = self.get_parameter('left_joint').value
        self.right_name = self.get_parameter('right_joint').value
        rate = self.get_parameter('publish_rate').value

        self.left_count = 0
        self.right_count = 0
        self.left_position = 0.0
        self.right_position = 0.0
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.last_stamp = self.get_clock().now()

        self.create_subscription(
            Vector3, '/enco/counts', self.encoder_callback,
            qos_profile_sensor_data)

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / rate, self.publish_joint_states)

        self.get_logger().info(
            'Publishing /joint_states for %s, %s at %.1f Hz (cpr %.1f)'
            % (self.left_name, self.right_name, rate, self.cpr)
        )

    def encoder_callback(self, msg):
        self.left_count = msg.x
        self.right_count = msg.y

    def counts_to_radians(self, counts):
        return (counts / self.cpr) * 2.0 * math.pi

    def publish_joint_states(self):
        now = self.get_clock().now()
        dt = (now - self.last_stamp).nanoseconds / 1e9
        self.last_stamp = now

        left = self.counts_to_radians(self.left_count)
        right = self.counts_to_radians(self.right_count)

        if dt > 0.0:
            self.left_velocity = (left - self.left_position) / dt
            self.right_velocity = (right - self.right_position) / dt

        self.left_position = left
        self.right_position = right

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [self.left_name, self.right_name]
        msg.position = [left, right]
        msg.velocity = [self.left_velocity, self.right_velocity]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
