"""
Persist the SLAM map every time it changes.

slam_toolbox republishes /map each time it folds new lidar scans into the
occupancy grid; this node writes that grid straight out as .pgm + .yaml on
every one of those updates, so the files on disk always match what SLAM
currently believes.

The grid is written directly rather than by shelling out to map_saver_cli:
that spawns a whole ROS node per save, which is far too heavy to run on every
update and races against the next update when saves come close together.

Runs on the wall clock. Nothing here reads the ROS clock, so it behaves the
same on the robot (use_sim_time false) as it did under Gazebo.
"""

import hashlib
import os
import time

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from slam_toolbox.srv import SerializePoseGraph

from nexva_explore import map_registry

# How many saves to let pass before complaining that slam_toolbox's
# serialise service never turned up. A few is normal - the autosaver is
# publishing before slam_toolbox has finished starting.
SKIPS_BEFORE_WARNING = 10

# Pixel values map_server expects in a trinary .pgm
FREE_PIXEL = 254
UNKNOWN_PIXEL = 205
OCCUPIED_PIXEL = 0

# Map topics are latched; match that so a map published before this node
# starts is still delivered.
MAP_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# slam_toolbox registers its services relative to its own node name, and
# nexva_slam's slam.launch.py runs it as `slam_toolbox` with namespace='',
# so the service lives at exactly this absolute name. Absolute on purpose:
# it must not move if this node is ever pushed into a namespace.
SERIALIZE_SERVICE = '/slam_toolbox/serialize_map'


class MapAutosaver(Node):
    """Writes the live SLAM map to disk on every update."""

    AUTOSAVE_DEFAULT = True

    def __init__(self, node_name='map_autosaver'):
        super().__init__(node_name)

        # Which robot this map belongs to: 'hardware' or 'sim'. It keeps the
        # two apart in map.md, so a run on the real robot cannot overwrite
        # a map Gazebo built, and each one reuses its own. Declared first
        # because the default folder is named after it.
        self.declare_parameter('map_source', map_registry.DEFAULT_SOURCE)
        self.map_source = self.get_parameter('map_source').value

        self.declare_parameter('map_dir', os.path.join(
            map_registry.maps_root(), self.map_source))
        self.declare_parameter('map_name', 'home')

        # False turns the node into a passive holder of the latest /map: it
        # writes nothing on its own. map_saver (a subclass) runs this way and
        # saves only when asked.
        self.declare_parameter('autosave', self.AUTOSAVE_DEFAULT)
        self.autosave = self.get_parameter('autosave').value
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh', 0.196)

        # 0.0 means "write every update". Raise it only if the map is huge and
        # the writes start costing more than they are worth.
        self.declare_parameter('min_save_interval', 0.0)

        # Where to record what was saved. Empty means "beside the maps",
        # ~/nexva_maps/map.md, or wherever NEXVA_MAP_REGISTRY points.
        self.declare_parameter('map_registry', '')

        self.map_dir = self.get_parameter('map_dir').value
        self.map_name = self.get_parameter('map_name').value
        self.occupied_thresh = self.get_parameter('occupied_thresh').value
        self.free_thresh = self.get_parameter('free_thresh').value
        self.min_save_interval = self.get_parameter('min_save_interval').value
        self.registry_file = map_registry.registry_path(
            self.get_parameter('map_registry').value or None
        )

        os.makedirs(self.map_dir, exist_ok=True)

        self.pgm_path = os.path.join(self.map_dir, self.map_name + '.pgm')
        self.yaml_path = os.path.join(self.map_dir, self.map_name + '.yaml')

        self.last_saved_hash = None
        self.last_save_time = 0.0
        self.saves = 0

        # slam_toolbox publishes /map absolutely, ignoring its namespace
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, MAP_QOS)

        # The .pgm/.yaml pair is a picture of the map. slam_toolbox cannot
        # resume from a picture - to carry on mapping where it left off it
        # needs its own pose graph. Saving one alongside is what lets the
        # cleaning run start from the map instead of from nothing, while
        # still being the same mapper that built it.
        #
        # Optional on purpose: if the service is not there (a cleaning run
        # driven by something else, or slam_toolbox not up yet) the .pgm and
        # .yaml are still written and everything that reads those still works.
        #
        # The name matters. In the simulator both nodes sat in a namespace and
        # a relative name was the only one that matched; here slam_toolbox is
        # launched un-namespaced, so the service is /slam_toolbox/serialize_map
        # and nothing else. Get it wrong and service_is_ready() is false
        # forever, serialise_graph() returns at its first line, and NO POSE
        # GRAPH IS EVER WRITTEN - which is why the skip below is counted and
        # eventually reported instead of staying silent.
        self.serialize_client = self.create_client(
            SerializePoseGraph, SERIALIZE_SERVICE)
        self.graph_path = os.path.join(self.map_dir, self.map_name)
        self.serialise_failed = False
        self.serialise_skipped = 0

        self.latest_map = None

        if self.autosave:
            self.get_logger().info(
                f'Autosaving the {self.map_source} map to {self.pgm_path} on '
                f'every SLAM update; recording it in {self.registry_file}'
            )

    def map_callback(self, msg):
        """Write the map out whenever the grid actually changed."""
        self.latest_map = msg

        if not self.autosave:
            return

        # The geometry has to be part of the digest, not just the cells: as
        # SLAM explores outward the grid is re-anchored, and an origin shift
        # changes the .yaml even when the cell bytes happen to be unchanged.
        info = msg.info
        digest = hashlib.md5(bytes(msg.data))
        digest.update(
            f'{info.width}x{info.height}@{info.resolution}'
            f':{info.origin.position.x},{info.origin.position.y}'.encode('ascii')
        )
        digest = digest.hexdigest()

        if digest == self.last_saved_hash:
            return

        # Wall clock: this is a disk-write throttle and has nothing to do
        # with the ROS clock.
        now = time.monotonic()
        if self.min_save_interval > 0.0:
            if now - self.last_save_time < self.min_save_interval:
                return

        try:
            self.write_map(msg)
        except OSError as exc:
            self.get_logger().warn(f'Could not write map: {exc}')
            return

        self.record_save()
        self.serialise_graph()

        self.last_saved_hash = digest
        self.last_save_time = now
        self.saves += 1

        info = msg.info
        self.get_logger().info(
            f'Map saved #{self.saves} ({info.width}x{info.height} @ '
            f'{info.resolution:.3f} m/px)'
        )

    def serialise_graph(self):
        """
        Ask slam_toolbox to write its pose graph next to the .pgm.

        Fire and forget. The result is not waited on: this runs inside the map
        callback and blocking there would stall the subscription. A failure is
        reported once and then left alone - the map itself is already on disk
        by this point, so the run is not damaged by the graph being missing.
        """
        if not self.serialize_client.service_is_ready():
            # Silence here is what hid the bug above: every save looked fine
            # and the graph was never written. Say it, once, and only after
            # enough saves that slam_toolbox has plainly had time to come up.
            self.serialise_skipped += 1

            if self.serialise_skipped == SKIPS_BEFORE_WARNING:
                self.get_logger().warn(
                    f'{self.serialize_client.srv_name} has not appeared after '
                    f'{self.serialise_skipped} map saves, so no pose graph is '
                    'being written. The .pgm and .yaml are saved, but a '
                    'cleaning run cannot resume slam_toolbox from this map '
                    'and will fall back to map_server + AMCL. Check that '
                    'slam_toolbox is running, un-namespaced, as '
                    '`slam_toolbox` (nexva_slam slam.launch.py).'
                )

            return

        self.serialise_skipped = 0

        request = SerializePoseGraph.Request()
        request.filename = self.graph_path

        future = self.serialize_client.call_async(request)
        future.add_done_callback(self.serialise_done)

    def serialise_done(self, future):
        try:
            result = future.result()
        except Exception as exc:                            # noqa: BLE001
            self.warn_serialise(str(exc))
            return

        # RESULT_SUCCESS is 0 in slam_toolbox's reply.
        if result is None or getattr(result, 'result', 0) != 0:
            self.warn_serialise(f'slam_toolbox returned {result}')
            return

        self.serialise_failed = False

    def warn_serialise(self, detail):
        """Say it once. A failure here does not cost the map, only the resume."""
        if self.serialise_failed:
            return

        self.serialise_failed = True
        self.get_logger().warn(
            f'Could not serialise the pose graph to {self.graph_path}: '
            f'{detail}. The .pgm and .yaml are saved and everything that '
            f'reads those still works - but a cleaning run will have to '
            f'rebuild the map as it goes rather than starting from this one. '
            f'Check the directory exists and is writable.'
        )

    def record_save(self):
        """Update map.md so the cleaning run knows what exists and when."""
        try:
            map_registry.write_registry(
                self.registry_file,
                map_name=self.map_name,
                yaml_path=self.yaml_path,
                pgm_path=self.pgm_path,
                source=self.map_source,
            )
        except OSError as exc:
            self.get_logger().warn(f'Could not update {self.registry_file}: {exc}')

    def write_map(self, msg):
        """Render the occupancy grid to .pgm and .yaml, atomically."""
        height = msg.info.height
        width = msg.info.width

        grid = np.array(msg.data, dtype=np.int16).reshape(height, width)

        image = np.full((height, width), UNKNOWN_PIXEL, dtype=np.uint8)
        image[(grid >= 0) & (grid <= self.free_thresh * 100)] = FREE_PIXEL
        image[grid >= self.occupied_thresh * 100] = OCCUPIED_PIXEL

        # An OccupancyGrid starts at the bottom-left; a .pgm starts top-left.
        image = np.flipud(image)

        header = f'P5\n{width} {height}\n255\n'.encode('ascii')
        self.atomic_write(self.pgm_path, header + image.tobytes())

        origin = msg.info.origin.position
        yaml_text = (
            f'image: {os.path.basename(self.pgm_path)}\n'
            f'mode: trinary\n'
            f'resolution: {msg.info.resolution:.6f}\n'
            f'origin: [{origin.x:.6f}, {origin.y:.6f}, 0.0]\n'
            f'negate: 0\n'
            f'occupied_thresh: {self.occupied_thresh}\n'
            f'free_thresh: {self.free_thresh}\n'
        )
        self.atomic_write(self.yaml_path, yaml_text.encode('ascii'))

    @staticmethod
    def atomic_write(path, payload):
        """Write via a temp file so a reader never sees a half-written map."""
        # pid in the name: map_saver may write the same map from another
        # process (clean mode runs both), and a shared .tmp would race.
        tmp = f'{path}.{os.getpid()}.tmp'

        with open(tmp, 'wb') as handle:
            handle.write(payload)

        os.replace(tmp, path)


def main(args=None):
    rclpy.init(args=args)
    node = MapAutosaver()

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
