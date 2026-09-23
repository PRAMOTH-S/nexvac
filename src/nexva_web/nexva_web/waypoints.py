"""Load and validate the named-waypoint file.

A waypoint is only meaningful against the map it was recorded on. The two maps
in this workspace have different origins (sep23map1 at [-3.232, -7.293],
sep23map2 at [-3.512, -5.449]), so the same x/y is roughly 1.8 m apart between
them. The file therefore names its map and the loader refuses to hand out poses
when a different one is loaded, rather than quietly sending the robot to the
wrong place.
"""

import os

import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# Matches what RViz's "2D Pose Estimate" tool sends. Zeros would tell AMCL the
# pose is exact, which collapses the particle filter.
INITIAL_POSE_COVARIANCE = [
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787,
]


class WaypointError(Exception):
    """Raised for anything malformed in the waypoint file."""


class Waypoint:

    def __init__(self, name, position, orientation):
        self.name = name
        self.x, self.y, self.z = position
        self.qx, self.qy, self.qz, self.qw = orientation

    def to_pose_stamped(self, frame_id, stamp):
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        msg.pose.orientation.x = self.qx
        msg.pose.orientation.y = self.qy
        msg.pose.orientation.z = self.qz
        msg.pose.orientation.w = self.qw
        return msg

    def to_initial_pose(self, frame_id):
        """Build the /initialpose message.

        header.stamp is deliberately left at zero. AMCL transforms the incoming
        pose using this stamp, and stamping it with "now" loses the race against
        TF often enough to matter - the symptom is AMCL logging "Failed to
        transform initial pose in time ... would require extrapolation into the
        future" and silently ignoring it. Zero means "use the latest transform".
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        msg.pose.pose.orientation.x = self.qx
        msg.pose.pose.orientation.y = self.qy
        msg.pose.pose.orientation.z = self.qz
        msg.pose.pose.orientation.w = self.qw
        msg.pose.covariance = list(INITIAL_POSE_COVARIANCE)
        return msg

    def as_dict(self):
        return {
            'name': self.name,
            'x': self.x, 'y': self.y, 'z': self.z,
            'qx': self.qx, 'qy': self.qy, 'qz': self.qz, 'qw': self.qw,
        }


class WaypointSet:

    def __init__(self, map_name, frame_id, waypoints, source_path):
        self.map_name = map_name
        self.frame_id = frame_id
        self.waypoints = waypoints
        self.source_path = source_path

    @property
    def names(self):
        return [w.name for w in self.waypoints]

    def get(self, name):
        for w in self.waypoints:
            if w.name == name:
                return w
        raise WaypointError(
            'no waypoint named %r (have: %s)' % (name, ', '.join(self.names)))

    def matches_map(self, loaded_map_yaml):
        """True if `loaded_map_yaml` (a path from map_server) is our map."""
        if not loaded_map_yaml:
            return False
        return os.path.splitext(os.path.basename(loaded_map_yaml))[0] == self.map_name

    def as_dict(self):
        return {
            'map': self.map_name,
            'frame_id': self.frame_id,
            'items': [w.as_dict() for w in self.waypoints],
        }


def _require(d, key, ctx):
    if key not in d:
        raise WaypointError('%s: missing %r' % (ctx, key))
    return d[key]


def _xyz(d, ctx):
    return (float(_require(d, 'x', ctx)),
            float(_require(d, 'y', ctx)),
            float(d.get('z', 0.0)))


def _quat(d, ctx):
    q = (float(d.get('x', 0.0)), float(d.get('y', 0.0)),
         float(d.get('z', 0.0)), float(d.get('w', 1.0)))
    norm = sum(v * v for v in q) ** 0.5
    if abs(norm - 1.0) > 0.01:
        raise WaypointError(
            '%s: orientation is not a unit quaternion (norm %.4f)' % (ctx, norm))
    return q


def load(path):
    """Parse a waypoint file, raising WaypointError on anything malformed."""
    if not os.path.isfile(path):
        raise WaypointError('waypoint file not found: %s' % path)
    with open(path) as fh:
        try:
            doc = yaml.safe_load(fh)
        except yaml.YAMLError as exc:
            raise WaypointError('%s is not valid YAML: %s' % (path, exc))

    if not isinstance(doc, dict):
        raise WaypointError('%s: top level must be a mapping' % path)

    map_name = _require(doc, 'map', path)
    frame_id = doc.get('frame_id', 'map')
    raw = _require(doc, 'waypoints', path)
    if not isinstance(raw, list) or not raw:
        raise WaypointError('%s: "waypoints" must be a non-empty list' % path)

    seen, out = set(), []
    for i, entry in enumerate(raw):
        ctx = '%s: waypoint %d' % (path, i)
        if not isinstance(entry, dict):
            raise WaypointError('%s: must be a mapping' % ctx)
        name = str(_require(entry, 'name', ctx))
        if name in seen:
            raise WaypointError('%s: duplicate name %r' % (ctx, name))
        seen.add(name)
        pose = _require(entry, 'pose', '%s (%s)' % (ctx, name))
        position = _require(pose, 'position', '%s (%s)' % (ctx, name))
        orientation = pose.get('orientation', {})
        out.append(Waypoint(name,
                            _xyz(position, '%s (%s) position' % (ctx, name)),
                            _quat(orientation, '%s (%s) orientation' % (ctx, name))))

    return WaypointSet(str(map_name), str(frame_id), out, path)
