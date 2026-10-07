#!/usr/bin/env python3
"""
Wait until SLAM has actually localized the robot: /map and a FRESH
map -> base_footprint transform.

Same reasoning as wait_for_bringup.py, one level up the stack. A mission
script that follows `ros2 launch ... slam` with `sleep 10` is guessing how
long slam_toolbox needs. On a cold Pi 5 with a first scan to match that guess
is routinely short, and whatever runs next - Nav2, an explorer - comes up
against a map frame that does not exist yet.

Two conditions, both required, and the second one is the one a naive check
gets wrong:

1. **/map received.** TRANSIENT_LOCAL + RELIABLE, depth 1: slam_toolbox
   latches the map, and a VOLATILE subscriber that arrives after the
   publication never sees it. That is the /scan BEST_EFFORT mistake again,
   in the opposite direction.

2. **map -> base_footprint resolves AND is fresh.** A lookup at
   `rclpy.time.Time()` returns the NEWEST transform in the buffer however old
   it is, so a slam_toolbox that published once and died still answers - and
   `can_transform` still says yes - forever. The transform is therefore aged
   by its own header stamp against --max-age, so a stale cached transform
   reads as not ready, which is what it is.

    wait_for_slam.py [--timeout S] [--max-age S] [--quiet]

Exit 0 when the robot is localized, 1 on timeout (naming what is missing),
2 if the ROS graph could not be joined at all.
"""

import argparse
import sys
import time

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

MAP_FRAME = 'map'
BASE_FRAME = 'base_footprint'

# slam_toolbox latches /map. Match it or a late subscriber hears nothing.
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class SlamWaiter(Node):

    def __init__(self, quiet, max_age):
        super().__init__('wait_for_slam')
        self.quiet = quiet
        self.max_age = max_age
        self.map_seen = False
        self.tf_live = False
        self.why = f'waiting for {MAP_FRAME} -> {BASE_FRAME}'

        self.create_subscription(
            OccupancyGrid, '/map', self.on_map, MAP_QOS)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(0.1, self.check_tf)

    def on_map(self, msg):
        if not self.map_seen:
            self.map_seen = True
            self.say(f'/map  live ({msg.info.width}x{msg.info.height})')

    def say(self, text):
        if text and not self.quiet:
            print(f'[slam]     {text}', flush=True)

    def check_tf(self):
        if self.tf_live:
            return

        try:
            tf = self.buffer.lookup_transform(MAP_FRAME, BASE_FRAME, Time())
        except tf2_ros.TransformException:
            self.why = (f'no {MAP_FRAME} -> {BASE_FRAME} - slam_toolbox has '
                        f'not matched a scan yet')
            return

        age = (self.get_clock().now()
               - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > self.max_age:
            self.why = (f'{MAP_FRAME} -> {BASE_FRAME} is {age:.1f} s old '
                        f'(limit {self.max_age:.1f} s) - a stale cached '
                        f'transform, not a live pose')
            return

        self.tf_live = True
        self.say(f'{MAP_FRAME} -> {BASE_FRAME}  live ({age:.2f} s old)')

    def ready(self):
        return self.map_seen and self.tf_live


def main():
    parser = argparse.ArgumentParser(prog='wait_for_slam')
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--max-age', type=float, default=2.0,
                        help='how fresh the transform stamp must be')
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()

    try:
        rclpy.init()
    except Exception as exc:                                  # noqa: BLE001
        print(f'[slam] cannot join the ROS graph: {exc}', file=sys.stderr)
        return 2

    node = SlamWaiter(args.quiet, args.max_age)
    deadline = time.monotonic() + args.timeout
    nagged = 0.0

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                break

            now = time.monotonic()
            if not args.quiet and now - nagged >= 2.0:
                nagged = now
                left = deadline - now
                missing = node.why if not node.tf_live else 'waiting for /map'
                print(f'[slam]     {missing} ({left:.0f}s left)', flush=True)
    except KeyboardInterrupt:
        pass

    ready = node.ready()
    why = node.why if not node.tf_live else 'no /map published'

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if ready:
        return 0

    if not args.quiet:
        print(f'[slam] NOT LOCALIZED after {args.timeout:.0f}s: {why}',
              flush=True)
        print('[slam]   is slam_toolbox running, and is bringup publishing '
              '/scan and odom -> base_footprint?', flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
