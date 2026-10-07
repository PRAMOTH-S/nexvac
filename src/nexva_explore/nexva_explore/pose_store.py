"""
The robot's 2D pose, kept in a writable file.

    ~/nexva_maps/<map_name>.pose.yaml

    map_name: kitchen
    source: hardware
    frame: map
    x: 1.234
    y: -0.5
    yaw_rad: 1.5708
    yaw_deg: 90.0
    saved_at: '2026-10-05 12:00:00'
    saved_unix: 1791201600.123

Why a file: the pose a map was finished at is the pose the next clean run must
start from, and that has to survive a reboot. Exploring ends with the robot
standing somewhere in the map's own frame; loading that map later and asking
AMCL to guess where it is is exactly the step that goes wrong.

The module is two layers. The functions at the top (`save_pose`, `load_pose`,
`pose_path`, `check_pose_for_map`) are plain Python with no ROS import, so the
saver, the clean launch's seeder and the tests all use them without a robot.
`PoseStore` below is the node that keeps the file current.

ATOMIC: a write goes to a temp file in the SAME directory and is then
`os.replace`d over the target. Same directory matters - os.replace is only
atomic within one filesystem. A crash or a pulled battery mid-write leaves
either the old file or the new one, never half of one.

THROTTLED: this is an SD card. The node samples at `write_period` (1 s) and
skips the write if the robot has not moved, with a slow heartbeat so the
timestamp does not go stale on a robot standing still.

PINNED ON SAVE: the pose is only meaningful together with the map it was
taken in. Once a map is saved (a /save_map request) the file is pinned to that
snapshot and the live tracking stops, otherwise a robot that carried on
cleaning would drag the pose away from the map on disk.
"""

import json
import math
import os
import tempfile
import time

import yaml

from nexva_explore import map_registry

ENV_POSE_FILE = 'NEXVA_POSE_FILE'
POSE_SUFFIX = '.pose.yaml'

# A pose this much older than the map it names is not from the same run.
# The saver writes both within a second of each other, so this is generous.
STALE_AFTER_MAP_S = 600.0

# Rewrite an unchanged pose at least this often.
HEARTBEAT_S = 30.0
MIN_MOVE_M = 0.01
MIN_TURN_RAD = math.radians(1.0)


# --------------------------------------------------------------------------
# Plain functions (no ROS)
# --------------------------------------------------------------------------

def pose_path(map_name, explicit=None):
    """
    Where the pose file for a map lives.

    Precedence: an explicit path (the `pose_file` parameter), then the
    NEXVA_POSE_FILE environment variable, then ~/nexva_maps/<map_name>.pose.yaml
    (NEXVA_MAPS moves that root).
    """
    if explicit:
        return os.path.expanduser(explicit)

    env = os.environ.get(ENV_POSE_FILE)
    if env:
        return os.path.expanduser(env)

    return os.path.join(map_registry.maps_root(), f'{map_name}{POSE_SUFFIX}')


def yaw_from_quat(z, w, x=0.0, y=0.0):
    """Yaw about +Z from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def save_pose(path, x, y, yaw, map_name, source=None, frame='map'):
    """
    Write the pose atomically. Raises OSError / ValueError; never half-writes.

    `yaw` is radians, normalised to (-pi, pi]. Degrees are written as well for
    a person reading the file; load_pose trusts only the radians.
    """
    x, y, yaw = float(x), float(y), float(yaw)
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError(f'refusing to save a non-finite pose ({x}, {y}, {yaw})')

    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    now = time.time()

    data = {
        'map_name': str(map_name),
        'source': source or map_registry.DEFAULT_SOURCE,
        'frame': frame,
        'x': round(x, 4),
        'y': round(y, 4),
        'yaw_rad': round(yaw, 5),
        'yaw_deg': round(math.degrees(yaw), 2),
        'saved_at': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now)),
        'saved_unix': round(now, 3),
    }

    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)

    # Same directory as the target: os.replace is atomic only on one filesystem.
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.pose-', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            yaml.safe_dump(data, handle, default_flow_style=False, sort_keys=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(tmp, 0o644)        # mkstemp makes it 0600
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise

    return data


def load_pose(path):
    """
    Read a pose file. Returns the dict, or None if it is missing, unreadable,
    not a mapping, or has a non-finite x / y / yaw_rad. Never raises.
    """
    try:
        with open(path, encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError, UnicodeDecodeError):
        return None

    if not isinstance(data, dict):
        return None

    try:
        x = float(data['x'])
        y = float(data['y'])
        yaw = float(data['yaw_rad'])
    except (KeyError, TypeError, ValueError):
        return None

    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return None

    data['x'], data['y'], data['yaw_rad'] = x, y, yaw

    try:
        data['saved_unix'] = float(data.get('saved_unix', 0.0))
    except (TypeError, ValueError):
        data['saved_unix'] = 0.0

    return data


def check_pose_for_map(pose, map_name, map_yaml=None, source=None,
                       stale_after=STALE_AFTER_MAP_S):
    """
    Decide whether a loaded pose may seed localisation on this map.

    Returns (ok, reason). Never ok on: no pose, a different map name, a
    different source, or a pose written much longer ago than the map file was
    (the map has since been re-saved by another run, so the pose belongs to
    an older version of it).
    """
    if pose is None:
        return False, 'no usable pose file'

    if pose.get('map_name') != map_name:
        return False, (f'pose file is for map {pose.get("map_name")!r}, '
                       f'not {map_name!r}')

    if source and pose.get('source') and pose['source'] != source:
        return False, f'pose file is from source {pose["source"]!r}, not {source!r}'

    if map_yaml:
        try:
            map_time = os.path.getmtime(map_yaml)
        except OSError:
            return False, f'map {map_yaml} is not readable'

        if pose['saved_unix'] < map_time - stale_after:
            return False, (
                f'pose is stale: saved {time.strftime("%F %T", time.localtime(pose["saved_unix"]))}'
                f', map written {time.strftime("%F %T", time.localtime(map_time))}')

    return True, 'ok'


def pose_summary(map_name, explicit=None):
    """Small dict for the web UI: the stored pose, or None."""
    pose = load_pose(pose_path(map_name, explicit))

    if pose is None or pose.get('map_name') != map_name:
        return None

    return {
        'x': pose['x'], 'y': pose['y'],
        'yaw_deg': round(math.degrees(pose['yaw_rad']), 1),
        'saved_at': pose.get('saved_at', ''),
    }


# --------------------------------------------------------------------------
# The node
# --------------------------------------------------------------------------

def main(args=None):
    # ROS imports live here so the functions above work with no ROS installed.
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.time import Time
    from std_msgs.msg import String
    import tf2_ros

    class PoseStore(Node):
        """Keeps <map_name>.pose.yaml current with the robot's map pose."""

        def __init__(self):
            super().__init__('pose_store')
            self.declare_parameter('map_name', 'explore')
            self.declare_parameter('map_source', map_registry.DEFAULT_SOURCE)
            self.declare_parameter('pose_file', '')
            self.declare_parameter('write_period', 1.0)
            self.declare_parameter('map_frame', 'map')
            self.declare_parameter('base_frame', 'base_footprint')
            self.declare_parameter('pose_timeout', 2.0)
            self.declare_parameter('pin_on_save', True)

            get = lambda n: self.get_parameter(n).value          # noqa: E731
            self.map_name = get('map_name')
            self.source = get('map_source')
            self.path = pose_path(self.map_name, get('pose_file') or None)
            self.map_frame = get('map_frame')
            self.base_frame = get('base_frame')
            self.pose_timeout = get('pose_timeout')
            self.pin_on_save = get('pin_on_save')

            self.last_written = None      # (x, y, yaw)
            self.last_write_time = 0.0
            self.pinned = False
            self.writes = 0
            self.amcl = None              # (x, y, yaw, monotonic)

            # Same reasoning as the web bridge: map->base_footprint from TF is
            # the live signal. /amcl_pose only publishes when the filter
            # updates, so a robot standing still stops sending it entirely.
            # It is only the fallback, for when TF cannot answer.
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)

            # A save request pins the file to the snapshot; a failed one frees it.
            self.create_subscription(String, '/save_map', self.save_request_cb, 10)
            self.create_subscription(String, '/save_map_result', self.save_result_cb, 10)

            self.create_timer(float(get('write_period')), self.tick)
            self.get_logger().info(
                f'pose store: {self.map_frame}->{self.base_frame} -> {self.path} '
                f'every {get("write_period")} s (when moved)')

        def amcl_cb(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.amcl = (p.x, p.y, yaw_from_quat(q.z, q.w), time.monotonic())

        def save_request_cb(self, _msg):
            if self.pin_on_save:
                self.pinned = True

        def save_result_cb(self, msg):
            try:
                result = json.loads(msg.data)
            except ValueError:
                return
            if isinstance(result, dict) and not result.get('ok') and self.pinned:
                self.pinned = False
                self.get_logger().warn('map save failed - pose tracking resumed')

        def current_pose(self):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, Time())
            except tf2_ros.TransformException:
                tf = None

            if tf is not None:
                # Time() returns the newest transform however old it is, so a
                # dead publisher never raises. Judge by the stamp instead.
                age = (self.get_clock().now()
                       - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
                if age <= self.pose_timeout:
                    t, q = tf.transform.translation, tf.transform.rotation
                    return (t.x, t.y, yaw_from_quat(q.z, q.w, q.x, q.y))

            if self.amcl and time.monotonic() - self.amcl[3] < 10.0:
                return self.amcl[:3]

            return None

        def tick(self):
            if self.pinned:
                return

            pose = self.current_pose()
            if pose is None:
                self.get_logger().warn(
                    f'no live {self.map_frame}->{self.base_frame} pose; '
                    'file left as it was', throttle_duration_sec=10.0)
                return

            now = time.monotonic()
            if self.last_written is not None and now - self.last_write_time < HEARTBEAT_S:
                lx, ly, lyaw = self.last_written
                turn = abs(math.atan2(math.sin(pose[2] - lyaw), math.cos(pose[2] - lyaw)))
                if math.hypot(pose[0] - lx, pose[1] - ly) < MIN_MOVE_M and turn < MIN_TURN_RAD:
                    return

            try:
                save_pose(self.path, *pose, self.map_name, source=self.source,
                          frame=self.map_frame)
            except (OSError, ValueError) as exc:
                self.get_logger().warn(f'could not write {self.path}: {exc}',
                                       throttle_duration_sec=10.0)
                return

            self.last_written = pose
            self.last_write_time = now
            self.writes += 1

    rclpy.init(args=args)
    node = PoseStore()
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
