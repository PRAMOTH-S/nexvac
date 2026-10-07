#!/usr/bin/env python3
"""
Wait until bringup is actually up: /scan, /odom and odom -> base_footprint.

Replaces three `ros2 topic echo --once` / `ros2 run tf2_ros tf2_echo` calls per
second. That approach had two faults, and both of them looked like "the script
is stuck":

1. **Startup cost.** Each `ros2 ...` call is a fresh Python process: rclpy init,
   then DDS discovery, before it can receive anything. Measured at 1-2 s on a
   laptop against a `timeout 3`; a Pi 5 under the load of a just-started bringup
   is slower still, so the check times out while the topic is publishing
   perfectly well. Three of those per loop meant most of the wait was spent
   starting and killing processes rather than listening.

2. **QoS.** The RPLIDAR publishes /scan BEST_EFFORT. `ros2 topic echo`
   subscribes RELIABLE by default, and a RELIABLE subscriber never connects to
   a BEST_EFFORT publisher - no error, no data, forever. That one is not flaky;
   it simply never succeeds.

This is one process, one discovery, three subscriptions with the right QoS on
each, held open until all three have been seen. It talks DDS directly, so a
stale `ros2 daemon` cannot hide the graph from it either.

    wait_for_bringup.py [--timeout S] [--quiet]

Exit 0 when everything is live, 1 on timeout (naming what is missing), 2 if
the ROS graph could not be joined at all.
"""

import argparse
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

TARGET_FRAME = 'odom'
SOURCE_FRAME = 'base_footprint'

MISSING_HELP = {
    'scan': '/scan silent - is the RPLIDAR plugged in and spinning? '
            '(check the rplidar lines in the bringup output)',
    'odom': '/odom silent - is the ESP32 on /dev/esp? '
            'check the micro-ROS agent lines in the bringup output',
    'tf': 'no odom -> base_footprint - wheel_odometry is not publishing TF',
}


class BringupWaiter(Node):

    def __init__(self, quiet):
        super().__init__('wait_for_bringup')
        self.quiet = quiet
        self.seen = {'scan': False, 'odom': False, 'tf': False}

        # Sensor QoS on BOTH: the lidar is BEST_EFFORT, and wheel_odometry
        # publishes /odom RELIABLE. A BEST_EFFORT subscriber receives from
        # either, so it is the safe side to be on for a liveness check.
        self.create_subscription(
            LaserScan, '/scan', lambda _: self.mark('scan'),
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odom', lambda _: self.mark('odom'),
            qos_profile_sensor_data)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(0.2, self.check_tf)

    def mark(self, key):
        if not self.seen[key]:
            self.seen[key] = True
            self.say(f'/{key}   live' if key != 'tf' else None)

    def say(self, text):
        if text and not self.quiet:
            print(f'[wait]     {text}', flush=True)

    def check_tf(self):
        if self.seen['tf']:
            return
        if self.buffer.can_transform(
                TARGET_FRAME, SOURCE_FRAME, rclpy.time.Time()):
            self.seen['tf'] = True
            self.say(f'{TARGET_FRAME} -> {SOURCE_FRAME}  live')

    def ready(self):
        return all(self.seen.values())


def main():
    parser = argparse.ArgumentParser(prog='wait_for_bringup')
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()

    try:
        rclpy.init()
    except Exception as exc:                                  # noqa: BLE001
        print(f'[wait] cannot join the ROS graph: {exc}', file=sys.stderr)
        return 2

    node = BringupWaiter(args.quiet)
    deadline = time.monotonic() + args.timeout

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.ready():
                break
    except KeyboardInterrupt:
        pass

    ready = node.ready()
    missing = [key for key, ok in node.seen.items() if not ok]

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if ready:
        return 0

    if not args.quiet:
        for key in missing:
            print(f'[wait]   {MISSING_HELP[key]}', flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
