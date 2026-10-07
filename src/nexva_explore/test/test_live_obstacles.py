"""
Live lidar returns reach the planning grid before SLAM draws them.

`live_obstacles` is lifted out of `frontier_explorer.py` by AST and bound to a
plain rig, as test_recovery does, so nothing needs a ROS context.
"""

import ast
import math
import os
import types

import numpy as np

SOURCE = os.path.join(
    os.path.dirname(__file__), '..', 'nexva_explore', 'frontier_explorer.py')

LIFTED = {'live_obstacles', 'world_to_cell', 'scan_is_stale', 'scan_age'}


def _lift():
    tree = ast.parse(open(SOURCE).read())
    out = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name in LIFTED:
            node.decorator_list = []
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'math': math, 'np': np, 'time': Clock}
            exec(compile(mod, SOURCE, 'exec'), ns)
            out[node.name] = ns[node.name]
    assert set(out) == LIFTED, set(out) ^ LIFTED
    return out


class Clock:
    t = 1000.0

    @classmethod
    def monotonic(cls):
        return cls.t


FUNCS = _lift()


def rig(points, scan_age=0.05, live_range=1.0):
    r = types.SimpleNamespace()
    for name, fn in FUNCS.items():
        setattr(r, name, types.MethodType(fn, r))
    r.scan_points = [(x, y, math.hypot(x, y)) for x, y in points]
    r.scan_time = Clock.t - scan_age
    r.scan_timeout = 0.5
    r.live_obstacle_range = live_range
    return r


def grid_msg(res=0.05, ox=-5.0, oy=-5.0):
    info = types.SimpleNamespace(
        resolution=res,
        origin=types.SimpleNamespace(position=types.SimpleNamespace(x=ox, y=oy)))
    return types.SimpleNamespace(info=info)


def cell(msg, x, y):
    return (int((y - msg.info.origin.position.y) / msg.info.resolution),
            int((x - msg.info.origin.position.x) / msg.info.resolution))


def test_obstacle_ahead_lands_in_map_frame_using_the_pose():
    msg = grid_msg()
    # robot at (1, 2) facing +y (yaw 90 deg); a return 0.5 m straight ahead
    # in the robot frame is at (1, 2.5) in the map.
    r = rig([(0.5, 0.0)])
    mask = r.live_obstacles(msg, (1.0, 2.0, math.pi / 2), 200, 200)
    assert mask.sum() == 1
    assert mask[cell(msg, 1.0, 2.5)]


def test_far_returns_are_ignored():
    r = rig([(0.5, 0.0), (3.0, 0.0), (0.0, -1.5)])
    mask = r.live_obstacles(grid_msg(), (0.0, 0.0, 0.0), 200, 200)
    assert mask.sum() == 1


def test_stale_scan_contributes_nothing():
    # a lidar that went quiet must not leave a phantom wall in the plan
    r = rig([(0.5, 0.0)], scan_age=2.0)
    assert not r.live_obstacles(grid_msg(), (0.0, 0.0, 0.0), 200, 200).any()


def test_disabled_by_zero_range():
    r = rig([(0.5, 0.0)], live_range=0.0)
    assert not r.live_obstacles(grid_msg(), (0.0, 0.0, 0.0), 200, 200).any()


def test_points_off_the_grid_are_dropped_not_wrapped():
    msg = grid_msg(ox=0.0, oy=0.0)
    # robot at the grid corner; a return behind it would be a negative index,
    # which numpy would silently wrap to the far side of the map.
    r = rig([(-0.5, 0.0)])
    mask = r.live_obstacles(msg, (0.0, 0.0, 0.0), 100, 100)
    assert not mask.any()
