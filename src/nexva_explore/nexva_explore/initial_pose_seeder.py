"""
Seed AMCL with the pose the map was saved at, then exit.

Run by clean.launch.py alongside Nav2. Exploring ends by saving the map AND the
robot's 2D pose + heading in that map's frame (<name>.pose.yaml); loading the
map later and telling AMCL exactly where the robot is beats letting it guess.

GUARDS. A wrong seed is worse than none (AMCL converges on the wrong place and
the robot sweeps into walls), so the pose is used only if
`pose_store.check_pose_for_map` accepts it: file present and parseable, same
map name, same source, and not older than the map file by more than 10 min.
Otherwise this logs a clear warning and does nothing - AMCL starts the way it
always did (the operator seeds it from the web UI).

The message copies nav_client's /initialpose rules: header.stamp is left ZERO
("use the latest transform"). Stamping it "now" loses the race against TF and
AMCL silently drops it with "would require extrapolation into the future". And
/initialpose is volatile, so publishing before AMCL subscribes vanishes with no
error - hence waiting for a subscriber, and confirming by /amcl_pose.

`min_runtime` keeps this process alive that long regardless, so the launch
(which starts the cleaner when this exits) never starts it earlier than it
would have without seeding.
"""

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)

from nexva_explore import map_registry, pose_store

# Seeded from a stored pose, so tight - not RViz's "somewhere around here".
COV_XY = 0.05 ** 2
COV_YAW = math.radians(3.0) ** 2


class InitialPoseSeeder(Node):
    def __init__(self):
        super().__init__('initial_pose_seeder')
        self.declare_parameter('map_name', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_source', map_registry.DEFAULT_SOURCE)
        self.declare_parameter('pose_file', '')
        self.declare_parameter('amcl_wait', 45.0)
        self.declare_parameter('min_runtime', 9.0)
        self.declare_parameter('stale_after', pose_store.STALE_AFTER_MAP_S)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.amcl_seen = False
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.VOLATILE))

    def amcl_cb(self, _msg):
        self.amcl_seen = True

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        started = time.monotonic()
        param = lambda n: self.get_parameter(n).value            # noqa: E731
        name = param('map_name')
        path = pose_store.pose_path(name, param('pose_file') or None)

        pose = pose_store.load_pose(path)
        ok, why = pose_store.check_pose_for_map(
            pose, name, param('map_yaml') or None,
            source=param('map_source'), stale_after=param('stale_after'))

        if not ok:
            self.get_logger().warn(
                f'NOT seeding AMCL from {path}: {why}. Falling back to the '
                'default start - set the initial pose from the web UI.')
        else:
            self.seed(pose, path, param('amcl_wait'))

        # Never return sooner than min_runtime, see module docstring.
        self.spin_for(max(0.0, param('min_runtime') - (time.monotonic() - started)))

    def seed(self, pose, path, wait):
        deadline = time.monotonic() + wait
        while rclpy.ok() and self.count_subscribers('/initialpose') == 0:
            if time.monotonic() > deadline:
                self.get_logger().warn(
                    'AMCL never subscribed to /initialpose - pose NOT seeded.')
                return
            self.spin_for(0.2)

        yaw = pose['yaw_rad']
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = pose.get('frame') or 'map'   # stamp stays zero
        msg.pose.pose.position.x = pose['x']
        msg.pose.pose.position.y = pose['y']
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = COV_XY
        cov[35] = COV_YAW
        msg.pose.covariance = cov

        for attempt in range(1, 6):
            self.pub.publish(msg)
            self.spin_for(3.0)
            if self.amcl_seen:
                self.get_logger().info(
                    f'AMCL seeded from {path}: x={pose["x"]:.2f} y={pose["y"]:.2f} '
                    f'yaw={math.degrees(yaw):.1f} deg (attempt {attempt})')
                return
        self.get_logger().warn(
            'published the saved pose 5 times but AMCL never answered with '
            '/amcl_pose - it may not have taken it. Check localisation in the UI.')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSeeder()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
