"""Check waypoints against the occupancy grid before the robot is asked to go.

A waypoint whose cell is occupied, or which sits closer to an obstacle than the
robot's own radius, cannot be reached. What happens instead is subtle and wastes
an afternoon: NavFn's `tolerance` lets it return a path that stops short of the
goal, the controller follows that path to its end, and SimpleGoalChecker then
refuses to declare success because the robot is not within xy_goal_tolerance of
the pose that was actually asked for. The run ends in recoveries and an abort
with no message naming the real cause.

Cheaper to catch it here. PGM parsing is done by hand so this needs no imaging
library.
"""

import math
import os

import yaml

ROBOT_RADIUS = 0.22      # matches robot_radius in nav2_params.yaml
SEARCH_LIMIT = 1.5       # how far out to look for the nearest obstacle


class MapError(Exception):
    pass


def _tokens(fh, count):
    """Read `count` whitespace-separated tokens, skipping '#' comments."""
    out = []
    token = b''
    while len(out) < count:
        ch = fh.read(1)
        if not ch:
            raise MapError('unexpected end of PGM header')
        if ch == b'#':
            while ch and ch not in b'\r\n':
                ch = fh.read(1)
            continue
        if ch.isspace():
            if token:
                out.append(token)
                token = b''
            continue
        token += ch
    return out


class OccupancyMap:

    def __init__(self, map_yaml_path):
        if not os.path.isfile(map_yaml_path):
            raise MapError('map yaml not found: %s' % map_yaml_path)
        with open(map_yaml_path) as fh:
            meta = yaml.safe_load(fh)

        self.name = os.path.splitext(os.path.basename(map_yaml_path))[0]
        self.resolution = float(meta['resolution'])
        self.origin_x, self.origin_y = float(meta['origin'][0]), float(meta['origin'][1])
        self.occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        self.free_thresh = float(meta.get('free_thresh', 0.196))
        self.negate = bool(meta.get('negate', 0))

        image = meta['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(map_yaml_path), image)
        with open(image, 'rb') as fh:
            magic = _tokens(fh, 1)[0]
            if magic != b'P5':
                raise MapError('only binary PGM (P5) maps are supported, got %r'
                               % magic.decode('ascii', 'replace'))
            w, h, maxval = (int(t) for t in _tokens(fh, 3))
            self.width, self.height, self.maxval = w, h, maxval
            self.data = fh.read(w * h)
        if len(self.data) < self.width * self.height:
            raise MapError('PGM is truncated (%d of %d bytes)'
                           % (len(self.data), self.width * self.height))

    @property
    def extent(self):
        return (self.origin_x,
                self.origin_x + self.width * self.resolution,
                self.origin_y,
                self.origin_y + self.height * self.resolution)

    def _cell_index(self, x, y):
        i = int((x - self.origin_x) / self.resolution)
        # PGM row 0 is the top of the image; the map origin is bottom-left.
        j = self.height - 1 - int((y - self.origin_y) / self.resolution)
        return i, j

    def _occupancy(self, i, j):
        """Probability that cell (i, j) is occupied, or None if off-map."""
        if not (0 <= i < self.width and 0 <= j < self.height):
            return None
        value = self.data[j * self.width + i]
        p = value / self.maxval if self.negate else (self.maxval - value) / self.maxval
        return p

    def classify(self, x, y):
        p = self._occupancy(*self._cell_index(x, y))
        if p is None:
            return 'off-map'
        if p > self.occupied_thresh:
            return 'occupied'
        if p < self.free_thresh:
            return 'free'
        return 'unknown'

    def nearest_obstacle(self, x, y, limit=SEARCH_LIMIT):
        """(distance, bearing_deg) of the closest occupied cell, or None."""
        i0, j0 = self._cell_index(x, y)
        n = int(limit / self.resolution)
        best = None
        for dj in range(-n, n + 1):
            for di in range(-n, n + 1):
                p = self._occupancy(i0 + di, j0 + dj)
                if p is None or p <= self.occupied_thresh:
                    continue
                d = math.hypot(di, dj) * self.resolution
                if best is None or d < best[0]:
                    best = (d, math.degrees(math.atan2(-dj, di)))
        return best


def check_waypoint(grid, wp, radius=ROBOT_RADIUS):
    """Return a dict describing whether `wp` is reachable on `grid`."""
    x0, x1, y0, y1 = grid.extent
    here = grid.classify(wp.x, wp.y)
    in_bounds = x0 <= wp.x <= x1 and y0 <= wp.y <= y1

    blocked = 0
    total = 0
    for angle in range(0, 360, 10):
        for r in (radius * 0.5, radius):
            total += 1
            c = grid.classify(wp.x + r * math.cos(math.radians(angle)),
                              wp.y + r * math.sin(math.radians(angle)))
            if c == 'occupied':
                blocked += 1

    nearest = grid.nearest_obstacle(wp.x, wp.y)
    yaw = math.degrees(2.0 * math.atan2(wp.qz, wp.qw))
    yaw = (yaw + 180.0) % 360.0 - 180.0

    problems = []
    if not in_bounds:
        problems.append('outside the map')
    if here == 'occupied':
        problems.append('pose is in an occupied cell')
    elif here == 'unknown':
        problems.append('pose is in unknown space')
    if blocked:
        problems.append('%d of %d footprint samples are occupied'
                        % (blocked, total))
    if nearest and nearest[0] < radius:
        problems.append('obstacle %.2f m away, inside the %.2f m footprint'
                        % (nearest[0], radius))

    return {
        'name': wp.name, 'x': wp.x, 'y': wp.y, 'yaw': yaw,
        'cell': here, 'in_bounds': in_bounds,
        'blocked': blocked, 'samples': total,
        'nearest': nearest, 'problems': problems,
        'ok': not problems,
    }
