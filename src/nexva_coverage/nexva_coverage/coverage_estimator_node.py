#!/usr/bin/env python3
# Copyright 2026 PRAMOTH-S
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Belief-based coverage estimator for the Nexva vacuum robot.

coverage_planner_node deliberately does not track what it has cleaned: it
expects an external estimator, and without one its waypoint skipping, gap-fill
and stop_at_target are inert. In the upstream project that input came from a
Gazebo ground-truth meter. This node is the real-robot equivalent.

What it does: follows the robot pose, stamps a disk of `cleaning_radius` into a
grid at every step, and publishes both the grid and the fraction of reachable
floor covered so far.

  subscribes  map              nav_msgs/OccupancyGrid   (transient_local)
  subscribes  amcl_pose        (fallback only; TF is the primary pose source)
  subscribes  coverage_planner/cleaning_active  (optional gate)
  publishes   covered_grid     nav_msgs/OccupancyGrid   (transient_local, 0/100)
  publishes   coverage_ratio   std_msgs/Float32
  service     ~/reset          std_srvs/Empty           (clear before a new run)

Two details that matter for the numbers to mean anything:

1. The denominator must be the planner's definition of reachable floor, not
   simply "cells marked free". The planner inflates obstacles AND unknown space
   by robot_radius before deciding what it can reach, so this node replicates
   `_build_masks` exactly (same 4-connected dilation). Count a larger free area
   than the planner plans over and the ratio can never reach coverage_target,
   so the sweep would never believe it finished.

2. Coverage is accumulated along the path between samples, not just at them.
   At 10 Hz and 0.2 m/s the robot moves 2 cm per tick, under one 5 cm cell, but
   during a fast transit it can cross several cells between samples and leave a
   dotted trail that reads as unclean floor.

This is an estimate of where the robot BELIEVES it has been: it inherits AMCL's
error and assumes the brush actually cleans its full swath. It is not proof the
floor is clean.
"""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

# Must match coverage_planner_node.py
FREE = 0
UNKNOWN = -1
OCC_THRESH = 50
COVERED = 100          # planner's _covered_at tests `>= 100`


def latched_qos() -> QoSProfile:
    """QoS matching a transient-local map publisher (map_server / SLAM)."""
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    """4-connected dilation, repeated `radius` times.

    Copied deliberately from coverage_planner_node so both nodes agree on what
    counts as reachable floor. A different kernel here would silently shift the
    denominator of every ratio this node publishes.
    """
    out = mask.copy()
    for _ in range(radius):
        shifted = out.copy()
        shifted[1:, :] |= out[:-1, :]
        shifted[:-1, :] |= out[1:, :]
        shifted[:, 1:] |= out[:, :-1]
        shifted[:, :-1] |= out[:, 1:]
        out = shifted
    return out


class CoverageEstimator(Node):

    def __init__(self) -> None:
        super().__init__('coverage_estimator')

        # cleaning_radius MUST match the planner's, or the planner will skip
        # waypoints this node never claimed to have cleaned (or vice versa).
        self.declare_parameter('cleaning_radius', 0.13)
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('stamp_hz', 10.0)
        self.declare_parameter('publish_hz', 2.0)
        # Only accumulate while the planner says it is cleaning. Default off so
        # the node is useful standalone (e.g. during a manual teleop sweep).
        self.declare_parameter('only_when_active', False)
        # Guard against a teleport: AMCL relocalising mid-run would otherwise
        # paint a clean stripe across everything between the old and new pose.
        self.declare_parameter('max_jump_m', 1.0)

        self.cleaning_radius = self.get_parameter('cleaning_radius').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('robot_base_frame').value
        self.only_when_active = self.get_parameter('only_when_active').value
        self.max_jump_m = self.get_parameter('max_jump_m').value

        self.map_msg: Optional[OccupancyGrid] = None
        self.free_mask: Optional[np.ndarray] = None
        self.covered: Optional[np.ndarray] = None
        self.total_free = 0
        self.disk: Optional[np.ndarray] = None
        self.last_xy: Optional[Tuple[float, float]] = None
        self.amcl_xy: Optional[Tuple[float, float]] = None
        self.cleaning_active = True
        self.logged_ready = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            OccupancyGrid, 'map', self._on_map, latched_qos())
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, latched_qos())
        self.create_subscription(
            Bool, '/coverage_planner/cleaning_active', self._on_active,
            latched_qos())

        self.grid_pub = self.create_publisher(
            OccupancyGrid, 'covered_grid', latched_qos())
        self.ratio_pub = self.create_publisher(Float32, 'coverage_ratio', 10)
        self.create_service(Empty, '~/reset', self._on_reset)

        self.create_timer(1.0 / float(self.get_parameter('stamp_hz').value),
                          self._stamp_tick)
        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value),
                          self._publish_tick)

        self.get_logger().info(
            'coverage_estimator up (cleaning_radius %.3f m); waiting for /map'
            % self.cleaning_radius)

    # ----------------------------------------------------------------- inputs

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self.map_msg is not None:
            return
        self.map_msg = msg
        info = msg.info
        h, w = info.height, info.width
        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)

        # Identical to the planner's _build_masks, minus keep-out zones (the
        # planner will not drive there, so those cells simply stay uncovered).
        obstacle = grid >= OCC_THRESH
        unknown = grid == UNKNOWN
        infl = max(1, int(round(self.robot_radius / info.resolution)))
        blocked = _dilate(obstacle | unknown, infl)
        self.free_mask = (grid == FREE) & ~blocked
        self.total_free = int(self.free_mask.sum())

        self.covered = np.zeros((h, w), dtype=np.int8)

        r = max(1, int(round(self.cleaning_radius / info.resolution)))
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        self.disk = (xx * xx + yy * yy) <= r * r

        self.get_logger().info(
            'map %dx%d @ %.3f m; %d reachable free cells; '
            'cleaning disk radius %d cells'
            % (w, h, info.resolution, self.total_free, r))

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_active(self, msg: Bool) -> None:
        self.cleaning_active = bool(msg.data)

    def _on_reset(self, request, response):
        if self.covered is not None:
            self.covered[:] = 0
        self.last_xy = None
        self.get_logger().info('coverage reset')
        return response

    def _robot_xy(self) -> Optional[Tuple[float, float]]:
        """TF first — AMCL only republishes on motion, so a still robot has none."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return self.amcl_xy

    # ------------------------------------------------------------ accumulate

    def _stamp_disk(self, x: float, y: float) -> None:
        info = self.map_msg.info
        res = info.resolution
        cx = int((x - info.origin.position.x) / res)
        cy = int((y - info.origin.position.y) / res)
        h, w = self.covered.shape
        r = self.disk.shape[0] // 2

        y0, y1 = cy - r, cy + r + 1
        x0, x1 = cx - r, cx + r + 1
        # clip both the destination window and the disk the same way, so a
        # robot near the map edge stamps a partial disk instead of throwing
        gy0, gy1 = max(0, y0), min(h, y1)
        gx0, gx1 = max(0, x0), min(w, x1)
        if gy0 >= gy1 or gx0 >= gx1:
            return
        dy0, dy1 = gy0 - y0, self.disk.shape[0] - (y1 - gy1)
        dx0, dx1 = gx0 - x0, self.disk.shape[1] - (x1 - gx1)

        sub = self.disk[dy0:dy1, dx0:dx1]
        window = self.covered[gy0:gy1, gx0:gx1]
        window[sub] = COVERED

    def _stamp_tick(self) -> None:
        if self.covered is None:
            return
        if self.only_when_active and not self.cleaning_active:
            return
        xy = self._robot_xy()
        if xy is None:
            return
        x, y = xy

        if self.last_xy is None:
            self._stamp_disk(x, y)
            self.last_xy = (x, y)
            if not self.logged_ready:
                self.logged_ready = True
                self.get_logger().info('tracking coverage from (%.2f, %.2f)'
                                       % (x, y))
            return

        lx, ly = self.last_xy
        dist = math.hypot(x - lx, y - ly)

        # A relocalisation jump is not travel: stamping the line between the
        # two poses would paint clean floor the robot never visited.
        if dist > self.max_jump_m:
            self.get_logger().warn(
                'pose jumped %.2f m (> max_jump_m %.2f) - not filling the gap'
                % (dist, self.max_jump_m))
            self._stamp_disk(x, y)
            self.last_xy = (x, y)
            return

        # Fill along the path so fast transits leave a stripe, not dots.
        step = self.map_msg.info.resolution * 0.5
        n = max(1, int(dist / step))
        for i in range(1, n + 1):
            f = i / n
            self._stamp_disk(lx + (x - lx) * f, ly + (y - ly) * f)
        self.last_xy = (x, y)

    # --------------------------------------------------------------- outputs

    def _publish_tick(self) -> None:
        if self.covered is None or self.total_free <= 0:
            return

        # Only floor the planner can actually reach counts, in numerator and
        # denominator alike. Stamps that land on walls or inflation are real
        # (the disk overhangs) but must not inflate the score.
        covered_free = int(np.count_nonzero(
            (self.covered >= COVERED) & self.free_mask))
        ratio = covered_free / float(self.total_free)

        self.ratio_pub.publish(Float32(data=float(ratio)))

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.global_frame
        out.info = self.map_msg.info
        out.data = self.covered.reshape(-1).tolist()
        self.grid_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageEstimator()
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
