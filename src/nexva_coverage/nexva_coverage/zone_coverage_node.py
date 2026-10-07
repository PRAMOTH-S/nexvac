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
Zone cleaning executor: plan a marked area, then drive it through Nav2.

Operating model matches a commercial floor scrubber rather than a domestic
robot: an operator marks a zone (a rectangle dragged on the web UI, or a
polygon in zones.yaml), names it, and starts a job on it. Planning is pure
geometry and lives in zone_planner; this node owns the ROS surface and the
execution loop.

  action srv  ~/clean_zone      nexva_coverage: started via the topic API below
  subscribes  map               nav_msgs/OccupancyGrid  (transient_local)
  subscribes  ~/zone            geometry_msgs/PolygonStamped  (draw-and-go)
  subscribes  ~/command         std_msgs/String  ('start <name>' | 'stop' | 'plan <name>')
  publishes   ~/plan            nav_msgs/Path           (the sweep, for RViz)
  publishes   ~/state           std_msgs/String         (idle/planning/cleaning)
  publishes   ~/progress        std_msgs/Float32        (0..1 through the passes)
  action clnt navigate_to_pose  nav2_msgs/NavigateToPose

Navigation is NOT reimplemented here. Every waypoint is handed to the existing
Nav2 stack, so all of the tuning already done on this robot - MPPI, costmap
inflation, AMCL, recoveries - applies unchanged.
"""

import math
import os
import threading
from typing import List, Optional

from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Float32, String
import yaml

from .zone_planner import (
    CoverageParams, MapInfo, Pass, Waypoint, densify, plan_zone,
)


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class ZoneCoverage(Node):

    def __init__(self) -> None:
        super().__init__('zone_coverage')

        self.declare_parameter('zones_file', '')
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('swath', 0.26)
        self.declare_parameter('overlap', 0.15)
        self.declare_parameter('edge_pass', True)
        self.declare_parameter('min_segment_len', 0.25)
        self.declare_parameter('min_region_area', 0.40)
        self.declare_parameter('waypoint_spacing', 0.50)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('goal_timeout_sec', 45.0)
        # A waypoint that fails twice is usually blocked by something that was
        # not on the map. Skipping beats burning the whole job on one spot.
        self.declare_parameter('max_retries', 1)
        # A waypoint blocked by something transient - a person, a trolley, a
        # door - is worth another attempt once the rest of the zone is done and
        # the obstruction has probably moved. Deferring beats both giving up on
        # it and standing there waiting for it.
        self.declare_parameter('retry_deferred', True)
        # 'through_poses' hands Nav2 a whole sweep row as ONE goal, so it drives
        # the row continuously. 'per_waypoint' sends a separate NavigateToPose
        # for every point, which makes the robot stop, settle inside the goal
        # tolerance and re-handshake every waypoint_spacing metres - the robot
        # spends more time standing still than sweeping. Keep through_poses.
        self.declare_parameter('drive_mode', 'through_poses')
        self.declare_parameter('auto_start', '')

        self.params = CoverageParams(
            robot_radius=self.get_parameter('robot_radius').value,
            swath=self.get_parameter('swath').value,
            overlap=self.get_parameter('overlap').value,
            edge_pass=self.get_parameter('edge_pass').value,
            min_segment_len=self.get_parameter('min_segment_len').value,
            min_region_area=self.get_parameter('min_region_area').value,
            waypoint_spacing=self.get_parameter('waypoint_spacing').value,
        )
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('robot_base_frame').value
        self.goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.retry_deferred = bool(self.get_parameter('retry_deferred').value)
        self.drive_mode = str(self.get_parameter('drive_mode').value)
        self.deferred: List[int] = []     # blocked waypoints, revisited at the end
        self.in_retry_sweep = False       # one retry pass only, never a loop

        self.grid: Optional[np.ndarray] = None
        self.info: Optional[MapInfo] = None
        self.zones = {}
        self.waypoints: List[Waypoint] = []
        self.legs: List[List[Waypoint]] = []   # one entry per sweep pass
        self.index = 0
        self.retries = 0
        self.state = 'idle'
        self.goal_handle = None
        self._lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, 'map', self._on_map,
                                 latched_qos(), callback_group=cb)
        self.create_subscription(PolygonStamped, '~/zone', self._on_zone,
                                 10, callback_group=cb)
        self.create_subscription(String, '~/command', self._on_command,
                                 10, callback_group=cb)

        self.plan_pub = self.create_publisher(Path, '~/plan', latched_qos())
        self.state_pub = self.create_publisher(String, '~/state', latched_qos())
        self.progress_pub = self.create_publisher(Float32, '~/progress', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                callback_group=cb)
        self.nav_through = ActionClient(self, NavigateThroughPoses,
                                        'navigate_through_poses',
                                        callback_group=cb)

        self._load_zones()
        self._publish_state()
        self.get_logger().info(
            'zone_coverage up: %d zone(s) loaded, swath %.2f m, row spacing '
            '%.3f m' % (len(self.zones), self.params.swath,
                        self.params.row_spacing))

    # ------------------------------------------------------------ zone file

    def _load_zones(self) -> None:
        path = self.get_parameter('zones_file').value
        if not path or not os.path.isfile(path):
            return
        try:
            doc = yaml.safe_load(open(path)) or {}
        except yaml.YAMLError as exc:
            self.get_logger().error('zones file is not valid YAML: %s' % exc)
            return
        for z in doc.get('zones', []):
            name = z.get('name')
            pts = z.get('points')
            if not name or not pts or len(pts) < 3:
                self.get_logger().warn('skipping malformed zone %r' % name)
                continue
            self.zones[name] = [(float(p[0]), float(p[1])) for p in pts]
        self.get_logger().info('zones: %s' % ', '.join(self.zones) or '(none)')

    # -------------------------------------------------------------- inputs

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self.grid is not None:
            return
        i = msg.info
        self.info = MapInfo(i.resolution, i.origin.position.x,
                            i.origin.position.y, i.width, i.height)
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape(
            i.height, i.width)
        self.get_logger().info('map %dx%d @ %.3f m' % (i.width, i.height,
                                                       i.resolution))
        auto = self.get_parameter('auto_start').value
        if auto:
            self._start(auto)

    def _on_zone(self, msg: PolygonStamped) -> None:
        """A zone drawn live (web UI rectangle). Stored as 'drawn'."""
        pts = [(p.x, p.y) for p in msg.polygon.points]
        if len(pts) < 3:
            self.get_logger().warn('drawn zone has < 3 points, ignoring')
            return
        self.zones['drawn'] = pts
        self.get_logger().info('zone "drawn" set from %d points' % len(pts))
        self._plan('drawn')

    def _on_command(self, msg: String) -> None:
        parts = msg.data.strip().split(None, 1)
        if not parts:
            return
        verb = parts[0].lower()
        arg = parts[1] if len(parts) > 1 else ''
        if verb == 'stop':
            self._stop()
        elif verb == 'plan':
            self._plan(arg or 'drawn')
        elif verb == 'start':
            self._start(arg or 'drawn')
        else:
            self.get_logger().warn('unknown command %r' % msg.data)

    # ------------------------------------------------------------ planning

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _plan(self, name: str) -> bool:
        if self.grid is None:
            self.get_logger().warn('no map yet')
            return False
        zone = self.zones.get(name)
        if zone is None and name not in ('', 'whole_map'):
            self.get_logger().error(
                'no zone %r (have: %s)' % (name, ', '.join(self.zones) or 'none'))
            return False

        self._set_state('planning')
        passes, stats = plan_zone(self.grid, self.info, self.params,
                                  zone=zone, start=self._robot_xy())
        self.waypoints = densify(passes, self.params)
        # Group the waypoints back into their passes. A pass is one continuous
        # run - a sweep row, or the perimeter loop - and is what gets handed to
        # Nav2 as a single goal.
        self.legs = []
        for p in passes:
            leg = densify([p], self.params)
            if leg:
                self.legs.append(leg)
        self.index = 0
        self.retries = 0

        self.get_logger().info(
            'planned %r: %d regions, %.1f m2, %d passes (%d edge / %d fill), '
            '%.1f m sweep, %d waypoints, angles %s'
            % (name or 'whole_map', stats['regions'], stats['drivable_area_m2'],
               stats['passes'], stats['edge_passes'], stats['fill_passes'],
               stats['sweep_length_m'], len(self.waypoints),
               stats['sweep_angles_deg']))
        self._publish_plan()
        self._set_state('idle')
        return bool(self.waypoints)

    def _publish_plan(self) -> None:
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for w in self.waypoints:
            path.poses.append(self._pose(w))
        self.plan_pub.publish(path)

    def _pose(self, w: Waypoint) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.global_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = w.x
        p.pose.position.y = w.y
        p.pose.orientation.z = math.sin(w.yaw / 2.0)
        p.pose.orientation.w = math.cos(w.yaw / 2.0)
        return p

    # ----------------------------------------------------------- execution

    def _start(self, name: str) -> None:
        if self.state == 'cleaning':
            self.get_logger().warn('already cleaning; send "stop" first')
            return
        if not self._plan(name):
            return
        client = (self.nav_through if self.drive_mode == 'through_poses'
                  else self.nav)
        name = ('navigate_through_poses' if self.drive_mode == 'through_poses'
                else 'navigate_to_pose')
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('%s unavailable - is Nav2 up?' % name)
            return
        self._set_state('cleaning')
        self._send_next()

    def _stop(self) -> None:
        with self._lock:
            handle, self.goal_handle = self.goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()
        self._set_state('idle')
        self.get_logger().info('stopped')

    def _units(self):
        """What the executor iterates over: whole passes, or single waypoints."""
        return self.legs if self.drive_mode == 'through_poses' else self.waypoints

    def _send_next(self) -> None:
        if self.state != 'cleaning':
            return
        units = self._units()
        if self.index >= len(units):
            if self.retry_deferred and self.deferred and not self.in_retry_sweep:
                retry = [units[i] for i in self.deferred]
                self.get_logger().info(
                    'first sweep done; retrying %d deferred %s'
                    % (len(retry),
                       'pass(es)' if self.drive_mode == 'through_poses'
                       else 'waypoint(s)'))
                if self.drive_mode == 'through_poses':
                    self.legs = retry
                else:
                    self.waypoints = retry
                self.deferred = []
                self.index = 0
                self.retries = 0
                self.in_retry_sweep = True
                self._send_next()
                return
            missed = len(self.deferred)
            self.get_logger().info(
                'zone complete: %d %s driven%s'
                % (len(units), 'passes' if self.drive_mode == 'through_poses'
                   else 'waypoints',
                   ', %d still unreachable' % missed if missed else ''))
            self.in_retry_sweep = False
            self.deferred = []
            self._set_state('idle')
            self.progress_pub.publish(Float32(data=1.0))
            return

        self.progress_pub.publish(Float32(data=self.index / float(len(units))))

        if self.drive_mode == 'through_poses':
            leg = units[self.index]
            goal = NavigateThroughPoses.Goal()
            goal.poses = [self._pose(w) for w in leg]
            self.get_logger().info(
                'pass %d/%d: %d poses (%s)'
                % (self.index + 1, len(units), len(leg), leg[0].kind))
            fut = self.nav_through.send_goal_async(goal)
        else:
            goal = NavigateToPose.Goal()
            goal.pose = self._pose(units[self.index])
            fut = self.nav.send_goal_async(goal)

        fut.add_done_callback(self._on_accepted)

    def _on_accepted(self, fut) -> None:
        try:
            handle = fut.result()
        except Exception as exc:
            self.get_logger().error('goal send failed: %s' % exc)
            self._advance(failed=True)
            return
        if handle is None or not handle.accepted:
            self._advance(failed=True)
            return
        with self._lock:
            self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut) -> None:
        with self._lock:
            self.goal_handle = None
        try:
            status = fut.result().status
        except Exception:
            self._advance(failed=True)
            return
        # 4 == STATUS_SUCCEEDED
        self._advance(failed=(status != 4))

    def _advance(self, failed: bool) -> None:
        if self.state != 'cleaning':
            return
        if failed and self.retries < self.max_retries:
            self.retries += 1
            self.get_logger().warn('waypoint %d failed, retry %d/%d'
                                   % (self.index, self.retries, self.max_retries))
            self._send_next()
            return
        if failed:
            if self.retry_deferred and not self.in_retry_sweep:
                self.deferred.append(self.index)
                self.get_logger().warn(
                    '%s %d blocked, deferred to the end of the sweep'
                    % ('pass' if self.drive_mode == 'through_poses'
                       else 'waypoint', self.index))
            else:
                self.get_logger().warn('waypoint %d unreachable, skipping'
                                       % self.index)
        self.retries = 0
        self.index += 1
        self._send_next()

    # -------------------------------------------------------------- status

    def _set_state(self, s: str) -> None:
        self.state = s
        self._publish_state()

    def _publish_state(self) -> None:
        self.state_pub.publish(String(data=self.state))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZoneCoverage()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
