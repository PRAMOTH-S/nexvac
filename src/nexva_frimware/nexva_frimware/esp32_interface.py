import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class ESP32Interface(Node):

    def __init__(self):
        super().__init__('esp32_interface')

        # Encoder data coming from ESP32
        self.left_encoder_sub = self.create_subscription(
            Int32,
            '/left_enco',
            self.left_encoder_callback,
            10
        )

        self.right_encoder_sub = self.create_subscription(
            Int32,
            '/right_enco',
            self.right_encoder_callback,
            10
        )

        # Odometry coming from ESP32
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Velocity command going to ESP32
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.left_ticks = 0
        self.right_ticks = 0

        self.get_logger().info('Nexva ESP32 interface started')

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

        self.get_logger().info(
            f'Left encoder: {self.left_ticks}'
        )

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

        self.get_logger().info(
            f'Right encoder: {self.right_ticks}'
        )

    def odom_callback(self, msg):
        self.get_logger().debug(
            f'Odom: x={msg.pose.pose.position.x:.3f}, '
            f'y={msg.pose.pose.position.y:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = ESP32Interface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
