"""
Keep the map growing while cleaning a previously saved map.

The saved-map cleaning run localises with AMCL against a fixed map served by
map_server, which by itself never changes - so anything that moved since the
map was made would never be noticed. This node loads that saved map as its
starting point, folds every new lidar scan into it, and republishes the result
on /map, so the cleaner plans against current reality and the autosaver keeps
writing it to disk.

map_server keeps serving the original on /static_map for AMCL. Localisation
stays anchored to the map it was tuned on, while /map moves with the world.

Cadence: every scan is folded in, and the result is published once a second.
Those are deliberately different numbers. Integrating is what turns lidar into
belief and it is cheap, so no scan is thrown away - the node used to sample
one scan every 0.5 s and discard the four in between, which at 10 Hz meant 80%
of the evidence never reached the map. Publishing is what costs: a whole
OccupancyGrid on the wire, and the autosaver writing it to disk behind it. One
a second is enough to plan and to watch, and it is the same interval
slam_toolbox uses for the global map, so both paths refresh at the same rate.

What is *not* folded in is anything far away. See scan_grid for why clearing a
cell needs the beams to still be close enough together to have covered it.

Where the beams start from is looked up through TF, not assumed. The scan is
placed from the pose of the frame in its own header (`laser` on Nexva), which
the URDF mounts on base_link at z=0.157 turned +90 degrees about z. The sim's
explorer got away with using the base pose because Gazebo's lidar sat at the
origin facing forward; on the real robot that would rotate every scan a
quarter turn and tear the map.
"""

import math
import os

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import tf2_ros

from nexva_explore.scan_grid import auto_clear_range, scan_cells

MAP_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# Log-odds, PER SECOND rather than per scan.
#
# This is the important part. These used to be per-scan, tuned when the node
# sampled one scan every 0.5 s. Folding in every scan instead took the rate
# from 2 Hz to 10 Hz without changing them, so every cell accumulated evidence
# five times faster than intended: a single beam was enough to flip a cell to
# occupied (+0.85 against a +0.5 threshold), and the map churned - cells
# changing their mind 8.8 times per publish, walls only 84.8% accurate.
#
# Expressed per second and divided by the measured scan rate, the map behaves
# the same whether the lidar runs at 5 Hz, 10 Hz or 20 Hz. A faster lidar
# gives a smoother map rather than a twitchier one, which is what you would
# expect from more information.
FREE_PER_SECOND = -0.8
OCCUPIED_PER_SECOND = 1.7

L_LIMIT = 4.0
L_PRIOR = 2.0

# Wide enough that no single scan can move a cell across. At 10 Hz a cell
# needs 8 consistent hits (0.8 s) to become occupied and 15 clear passes
# (1.5 s) to become free. That is slow for obstacle avoidance and completely
# fine here, because avoidance does not read this map - frontier_explorer
# checks the live scan in publish_safe(), 10 times a second. This map is for
# PLANNING, and a planner wants a map that has made up its mind.
OCCUPIED_AT = 1.2
FREE_AT = -1.2

# If scans arrive faster or slower than this, the per-second values above are
# divided by the real rate instead. Only used before enough scans have arrived
# to measure it.
NOMINAL_HZ = 10.0


class MapUpdater(Node):
    """Publishes the saved map with live lidar folded into it."""

    def __init__(self):
        super().__init__('map_updater')

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('max_range', 8.0)

        # How far out a beam is still allowed to mark a cell FREE. Echoes are
        # taken out to max_range; clearing stops here. 0.0 works it out from
        # the scan itself - the range where two beams are one cell apart, so
        # past it they comb the ground rather than sweep it. See scan_grid.
        self.declare_parameter('clear_range', 0.0)

        # And how far out an echo may MARK a cell occupied. Past this an echo
        # is still a real return, but the beam is wider than a cell by then,
        # so which cell it belongs in is a guess - and a guess that could not
        # be taken back, because clearing stops at `clear_range` and anything
        # marked beyond that is never revisited by a free vote. 0.0 works it
        # out from the scan, same figure as the clearing radius.
        self.declare_parameter('mark_range', 0.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_range = self.get_parameter('max_range').value
        self.clear_range = self.get_parameter('clear_range').value
        self.mark_range = self.get_parameter('mark_range').value
        self.announced_clear_range = False

        map_yaml = self.get_parameter('map_yaml').value

        self.resolution = 0.05
        self.origin = (0.0, 0.0)
        self.log_odds = None
        self.scan = None

        # The cleaner raises this while the robot is crashing. Its pose is
        # untrustworthy then, so anything folded in would land in the wrong
        # place and tear the map.
        self.frozen = False

        if map_yaml and os.path.isfile(map_yaml):
            self.load_map(map_yaml)
            self.get_logger().info(
                f'Loaded {map_yaml} as the starting point '
                f'({self.log_odds.shape[1]}x{self.log_odds.shape[0]} '
                f'@ {self.resolution:.3f} m/px)'
            )
        else:
            self.get_logger().error(f'Cannot read map yaml: {map_yaml!r}')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_pub = self.create_publisher(OccupancyGrid, 'map', MAP_QOS)

        # BEST_EFFORT, because that is what the RPLIDAR driver publishes. A
        # RELIABLE subscriber is simply never matched to a BEST_EFFORT
        # publisher - no error, no warning, no scans, and this node sits
        # there doing nothing. qos_profile_sensor_data is the standard
        # profile for exactly this: best effort, volatile, a short queue.
        # Integration is cheap enough that the few scans it may hold are
        # folded in before the pose has moved far.
        self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, 'map_freeze', self.freeze_callback, 10)

        # Integrate on arrival, publish on a clock. Every scan counts; the
        # grid goes out once a second.
        self.publish_period = self.get_parameter('publish_period').value
        self.create_timer(self.publish_period, self.publish_map)

        self.scans_folded = 0
        self.scans_since_publish = 0
        self.publishes = 0

        # Measured scan rate, so the per-second log-odds above can be divided
        # by it. Held as an interval because that is what arrives.
        self.scan_interval = 1.0 / NOMINAL_HZ
        self.last_scan_time = None
        self.announced_rate = False

        # Said once, so a scan frame that TF has never heard of is reported
        # rather than silently dropping every scan.
        self.warned_tf = False

        self.get_logger().info(
            f'Folding every scan into {self.log_odds.shape[1]}x'
            f'{self.log_odds.shape[0]} cells, publishing /map every '
            f'{self.publish_period:.1f} s'
            if self.log_odds is not None else
            'No map loaded - nothing to update'
        )

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------

    def load_map(self, yaml_path):
        """Read the .yaml plus its .pgm into a log-odds grid."""
        fields = {}

        with open(yaml_path, encoding='ascii') as handle:
            for line in handle:
                if ':' not in line:
                    continue
                key, _, value = line.partition(':')
                fields[key.strip()] = value.strip()

        self.resolution = float(fields.get('resolution', 0.05))

        origin = fields.get('origin', '[0.0, 0.0, 0.0]')
        parts = origin.strip('[]').split(',')
        self.origin = (float(parts[0]), float(parts[1]))

        image = fields.get('image', '')
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(yaml_path), image)

        grid = self.read_pgm(image)

        # .pgm is written top row first, an OccupancyGrid starts bottom-left.
        grid = np.flipud(grid)

        # Trinary .pgm: 0 occupied, 205 unknown, 254 free. The free test has
        # to sit above 205 or unknown space loads as free and the robot
        # believes it has already seen the whole world.
        self.log_odds = np.zeros(grid.shape, dtype=np.float32)
        self.log_odds[grid <= 100] = L_PRIOR
        self.log_odds[grid >= 230] = -L_PRIOR
        # 205 stays 0, i.e. still unknown

    @staticmethod
    def read_pgm(path):
        """Read a binary P5 .pgm into a uint8 array."""
        with open(path, 'rb') as handle:
            data = handle.read()

        fields = []
        index = 0

        # magic, width, height, maxval - skipping comments and any whitespace
        while len(fields) < 4:
            while index < len(data) and data[index:index + 1].isspace():
                index += 1
            if data[index:index + 1] == b'#':
                while index < len(data) and data[index:index + 1] != b'\n':
                    index += 1
                continue
            start = index
            while index < len(data) and not data[index:index + 1].isspace():
                index += 1
            fields.append(data[start:index])

        index += 1
        width = int(fields[1])
        height = int(fields[2])

        pixels = np.frombuffer(data[index:index + width * height], dtype=np.uint8)
        return pixels.reshape(height, width)

    # ------------------------------------------------------------------
    # Updating
    # ------------------------------------------------------------------

    def scan_callback(self, msg):
        self.scan = msg
        self.track_rate()
        self.integrate()

    def track_rate(self):
        """Keep a smoothed estimate of how often scans arrive."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_scan_time is not None:
            gap = now - self.last_scan_time
            # Ignore absurd gaps: a pause while something else hogged the CPU
            # is not the lidar changing rate, and letting it in would make one
            # stall permanently weaken every future update.
            if 0.005 < gap < 1.0:
                self.scan_interval += 0.1 * (gap - self.scan_interval)
        self.last_scan_time = now

        if not self.announced_rate and self.scans_folded > 20:
            self.announced_rate = True
            self.get_logger().info(
                f'scans at {1.0 / self.scan_interval:.1f} Hz; a cell needs '
                f'{math.ceil(OCCUPIED_AT / (OCCUPIED_PER_SECOND * self.scan_interval))} '
                f'hits to read occupied, '
                f'{math.ceil(abs(FREE_AT) / abs(FREE_PER_SECOND * self.scan_interval))} '
                f'clear passes to read free'
            )

    def freeze_callback(self, msg):
        if msg.data != self.frozen:
            self.get_logger().info(
                'Mapping held - robot is crashing' if msg.data
                else 'Mapping resumed'
            )

        self.frozen = msg.data

    def scan_pose(self, scan):
        """
        Where this scan's beams start from, in the map frame: (x, y, yaw).

        The frame comes from the scan itself, so the lidar's real mount on the
        chassis - height, offset and the quarter turn the URDF gives it - is
        applied by TF rather than assumed away. Only if the scan carries no
        frame at all does this fall back to the base frame.
        """
        frame = scan.header.frame_id.lstrip('/') or self.base_frame

        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, frame, Time())
        except tf2_ros.TransformException as exc:
            if not self.warned_tf:
                self.warned_tf = True
                self.get_logger().warn(
                    f'No transform {self.map_frame} -> {frame} yet, scans are '
                    f'not being folded in: {exc}'
                )
            return None

        self.warned_tf = False

        t = tf.transform.translation
        q = tf.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        return t.x, t.y, yaw

    def trust_radius(self, scan):
        """How far this scan may clear cells."""
        if self.clear_range > 0.0:
            return self.clear_range

        return min(auto_clear_range(scan, self.resolution), self.max_range)

    def mark_radius(self, scan):
        """How far this scan may mark cells occupied."""
        if self.mark_range > 0.0:
            return min(self.mark_range, self.max_range)

        return min(auto_clear_range(scan, self.resolution), self.max_range)

    def announce_ranges(self, scan):
        """Say once what the two radii worked out to, and why."""
        if self.announced_clear_range:
            return

        self.announced_clear_range = True
        clearing = self.trust_radius(scan)
        marking = self.mark_radius(scan)

        self.get_logger().info(
            f'Clearing free space out to {clearing:.2f} m, marking obstacles '
            f'out to {marking:.2f} m '
            f'({math.degrees(abs(scan.angle_increment)):.2f} deg between '
            f'beams, one {self.resolution * 100:.0f} cm cell at that range); '
            f'anything beyond is read but not written'
        )

        if marking > clearing:
            self.get_logger().warn(
                f'mark_range {marking:.2f} m is beyond the clearing radius '
                f'{clearing:.2f} m: a cell marked out there can never be '
                'cleared again, because no free vote ever reaches it'
            )

    def integrate(self):
        """Fold the latest scan into the grid."""
        if self.log_odds is None or self.scan is None or self.frozen:
            return

        scan = self.scan

        pose = self.scan_pose(scan)
        if pose is None:
            return

        x, y, yaw = pose
        height, width = self.log_odds.shape

        self.announce_ranges(scan)

        free_rows, free_cols, hit_rows, hit_cols = scan_cells(
            scan, x, y, yaw, self.resolution,
            self.origin[0], self.origin[1], width, height,
            self.trust_radius(scan), self.max_range, self.mark_radius(scan),
        )

        # One vote per cell per scan, so the weight a cell carries reflects
        # how many scans agreed about it rather than how many beams happened
        # to graze it. np.add.at is not needed: scan_cells already returns
        # each cell once, so plain fancy indexing cannot double-count.
        #
        # Scaled by the real gap between scans, so the map fills at the same
        # speed in seconds whatever the lidar rate is.
        free_step = FREE_PER_SECOND * self.scan_interval
        occupied_step = OCCUPIED_PER_SECOND * self.scan_interval

        if free_rows.size:
            self.log_odds[free_rows, free_cols] += free_step

        if hit_rows.size:
            self.log_odds[hit_rows, hit_cols] += occupied_step

        np.clip(self.log_odds, -L_LIMIT, L_LIMIT, out=self.log_odds)

        self.scans_folded += 1
        self.scans_since_publish += 1

    def publish_map(self):
        """Publish the current belief as an OccupancyGrid, once a second."""
        if self.log_odds is None:
            return

        # Every 30th publish, say how many scans went in. This is the number
        # that tells you the map is keeping up: it should be the lidar rate
        # times the publish period, and a drop means scans are being missed.
        self.publishes += 1
        if self.publishes % 30 == 0:
            self.get_logger().info(
                f'{self.scans_folded} scans folded in, '
                f'{self.scans_folded / self.publishes:.1f} per /map '
                f'({self.scans_since_publish} since the last one)'
            )

        self.scans_since_publish = 0

        height, width = self.log_odds.shape

        grid = np.full((height, width), -1, dtype=np.int8)
        grid[self.log_odds >= OCCUPIED_AT] = 100
        grid[self.log_odds <= FREE_AT] = 0

        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = self.resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.reshape(-1).tolist()

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapUpdater()

    # ExternalShutdownException is what a SIGTERM looks like from in here -
    # `timeout`, a launch file tearing down, systemd. Without catching it the
    # node exits through a traceback every time it is stopped normally, and the
    # unguarded shutdown below then raises a SECOND time on a context rclpy has
    # already torn down ("rcl_shutdown already called").
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
