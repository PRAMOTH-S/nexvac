#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


# ============================================================
# SETTINGS
# ============================================================

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200


# ============================================================
# NODE
# ============================================================

class ESP32Bridge(Node):

    def __init__(self):

        super().__init__("esp32_bridge")


        # ----------------------------------------------------
        # SERIAL
        # ----------------------------------------------------

        try:

            self.serial = serial.Serial(
                SERIAL_PORT,
                BAUD_RATE,
                timeout=0.01
            )

            self.get_logger().info(
                f"ESP32 connected: {SERIAL_PORT}"
            )

        except Exception as e:

            self.serial = None

            self.get_logger().error(
                f"ESP32 connection failed: {e}"
            )


        # ----------------------------------------------------
        # ODOM PUBLISHER
        # ----------------------------------------------------

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10
        )


        # ----------------------------------------------------
        # CMD VEL PUBLISHER
        # ----------------------------------------------------

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )


        # ----------------------------------------------------
        # TF
        # ----------------------------------------------------

        self.tf_broadcaster = TransformBroadcaster(self)


        # ----------------------------------------------------
        # ROBOT POSITION
        # ----------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0


        # ----------------------------------------------------
        # VELOCITY
        # ----------------------------------------------------

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0


        # ----------------------------------------------------
        # TIME
        # ----------------------------------------------------

        self.last_time = self.get_clock().now()


        # ----------------------------------------------------
        # UPDATE TIMER
        # ----------------------------------------------------

        self.timer = self.create_timer(
            0.02,
            self.update
        )


        self.get_logger().info(
            "ESP32 RViz virtual controller READY"
        )


    # ========================================================
    # READ ESP32
    # ========================================================

    def read_serial(self):

        if self.serial is None:
            return


        try:

            while self.serial.in_waiting:

                line = self.serial.readline().decode(
                    "utf-8",
                    errors="ignore"
                ).strip()


                if not line:
                    continue


                # ------------------------------------------------
                # VEL,linear,angular
                # ------------------------------------------------

                if line.startswith("VEL,"):

                    parts = line.split(",")


                    if len(parts) == 3:

                        self.linear_velocity = float(parts[1])

                        self.angular_velocity = float(parts[2])


        except Exception as e:

            self.get_logger().error(
                f"Serial read error: {e}"
            )


    # ========================================================
    # UPDATE
    # ========================================================

    def update(self):

        self.read_serial()


        # ----------------------------------------------------
        # TIME
        # ----------------------------------------------------

        current_time = self.get_clock().now()


        dt = (
            current_time - self.last_time
        ).nanoseconds / 1e9


        self.last_time = current_time


        if dt <= 0:
            return


        # ----------------------------------------------------
        # UPDATE POSITION
        # ----------------------------------------------------

        self.x += (
            self.linear_velocity
            * math.cos(self.theta)
            * dt
        )


        self.y += (
            self.linear_velocity
            * math.sin(self.theta)
            * dt
        )


        self.theta += (
            self.angular_velocity * dt
        )


        # Normalize angle

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )


        # ----------------------------------------------------
        # CMD VEL
        # ----------------------------------------------------

        twist = Twist()

        twist.linear.x = self.linear_velocity

        twist.angular.z = self.angular_velocity

        self.cmd_vel_pub.publish(twist)


        # ----------------------------------------------------
        # ODOM
        # ----------------------------------------------------

        self.publish_odom(current_time)


    # ========================================================
    # ODOM + TF
    # ========================================================

    def publish_odom(self, current_time):

        # ----------------------------------------------------
        # QUATERNION
        # ----------------------------------------------------

        qz = math.sin(self.theta / 2.0)

        qw = math.cos(self.theta / 2.0)


        # ----------------------------------------------------
        # ODOM
        # ----------------------------------------------------

        odom = Odometry()


        odom.header.stamp = current_time.to_msg()

        odom.header.frame_id = "odom"

        odom.child_frame_id = "base_link"


        odom.pose.pose.position.x = self.x

        odom.pose.pose.position.y = self.y

        odom.pose.pose.position.z = 0.0


        odom.pose.pose.orientation.z = qz

        odom.pose.pose.orientation.w = qw


        odom.twist.twist.linear.x = self.linear_velocity

        odom.twist.twist.angular.z = self.angular_velocity


        self.odom_pub.publish(odom)


        # ----------------------------------------------------
        # TF
        # ----------------------------------------------------

        tf = TransformStamped()


        tf.header.stamp = current_time.to_msg()

        tf.header.frame_id = "odom"

        tf.child_frame_id = "base_link"


        tf.transform.translation.x = self.x

        tf.transform.translation.y = self.y

        tf.transform.translation.z = 0.0


        tf.transform.rotation.z = qz

        tf.transform.rotation.w = qw


        self.tf_broadcaster.sendTransform(tf)


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = ESP32Bridge()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass


    finally:

        if node.serial is not None:

            node.serial.close()


        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
