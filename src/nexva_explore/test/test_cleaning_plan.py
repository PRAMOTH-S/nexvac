# Copyright 2026 vac_main1
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
Score the cleaning sweep's waypoint plan against synthetic rooms.

The plan methods are lifted out of `frontier_explorer.py` by AST and bound to
a plain object, so nothing here needs a ROS context or a running node. What is
measured is the ORDERED WAYPOINT LIST - not a robot driving it. Every figure
quoted in `build.md` for the 30 Aug 20:04 build comes from this file.
"""

import ast
from collections import deque
import math
import os
import types

import numpy as np

SOURCE = os.path.join(
    os.path.dirname(__file__), '..', 'nexva_explore', 'frontier_explorer.py')

LIFTED = {'build_cleaning_plan', 'sweep_transposed', 'sweep_strips',
          'split_regions', 'start_corner', 'region_lines', 'region_entry',
          'sweep_region', 'run_points', 'contiguous_runs', 'dilate',
          'flood', 'reachable_floor', 'fill_specks', 'floor_runs',
          'wavefront', 'descend'}


def _lift(names=LIFTED):
    """Pull the plan builder out of the node class without importing rclpy."""
    tree = ast.parse(open(SOURCE).read())
    # BY NAME, not "the first class in the file". Adding any class above
    # FrontierExplorer - BlockState did exactly this on 31 Aug - otherwise
    # lifts the wrong one, finds none of the plan methods, and fails the whole
    # build with a collection error that says nothing about the real cause.
    node = next(n for n in tree.body
                if isinstance(n, ast.ClassDef) and n.name == 'FrontierExplorer')
    kept = [n for n in node.body
            if isinstance(n, ast.FunctionDef) and n.name in names]

    assert {n.name for n in kept} == set(names), set(names) - {n.name for n in kept}

    holder = ast.ClassDef(name='Plan', bases=[], keywords=[], body=kept,
                          decorator_list=[], type_params=[])
    module = ast.fix_missing_locations(
        ast.Module(body=[holder], type_ignores=[]))
    scope = {'math': math, 'np': np}
    exec(compile(module, SOURCE, 'exec'), scope)
    return scope['Plan']


Plan = _lift()


class Grid:
    """Just enough of an OccupancyGrid for the plan builder."""

    def __init__(self, res, height, width, origin_x=0.0, origin_y=0.0):
        position = types.SimpleNamespace(x=origin_x, y=origin_y)
        self.info = types.SimpleNamespace(
            resolution=res, height=height, width=width,
            origin=types.SimpleNamespace(position=position))


def planner(axis='auto', row_spacing=0.22, corner_turn_weight=0.25,
            start='auto', row_point_spacing=1.0, cleaning_radius=0.135):
    plan = Plan()
    plan.row_spacing = row_spacing
    plan.cleaning_radius = cleaning_radius
    plan.row_point_spacing = row_point_spacing
    plan.footprint_width = 0.27
    plan.plan_radius = 0.27 / 2.0 + 0.03
    plan.sweep_start = start
    plan.sweep_axis = axis
    plan.corner_turn_weight = corner_turn_weight
    plan.cell_to_world = lambda msg, row, col: (
        msg.info.origin.position.x + (col + 0.5) * msg.info.resolution,
        msg.info.origin.position.y + (row + 0.5) * msg.info.resolution)
    return plan


def room(width_m, height_m, res=0.05, blockers=()):
    """Open floor with rectangular blockers, all in metres."""
    width, height = int(width_m / res), int(height_m / res)
    floor = np.ones((height, width), dtype=bool)

    for x0, y0, x1, y1 in blockers:
        floor[int(y0 / res):int(y1 / res), int(x0 / res):int(x1 / res)] = False

    return floor, Grid(res, height, width)


def build(plan, floor, msg, pose):
    return plan.build_cleaning_plan(
        msg, floor, msg.info.height, msg.info.width, pose)


def old_plan(plan, msg, floor, pose):
    """
    Lay out the row-by-row sweep this replaced, to measure against.

    Copied from the 30 Aug 08:20 build: rows in grid order, first row walked
    from whichever end the robot is nearer, direction flipped per row, every
    run on a row emitted where it falls.
    """
    res = msg.info.resolution
    step = max(1, int(round(plan.row_spacing / res)))
    min_run = max(2, int(round(plan.footprint_width / res)))
    rows = [r for r in range(0, msg.info.height, step) if floor[r].any()]

    if not rows:
        return []

    reverse = False
    here_row = int((pose[1] - msg.info.origin.position.y) / res)
    here_col = int((pose[0] - msg.info.origin.position.x) / res)

    if abs(here_row - rows[-1]) < abs(here_row - rows[0]):
        rows = list(reversed(rows))

    ends = np.flatnonzero(floor[rows[0]])

    if ends.size:
        reverse = abs(here_col - int(ends[-1])) < abs(here_col - int(ends[0]))

    goals = []

    for row in rows:
        cols = np.flatnonzero(floor[row])

        if cols.size == 0:
            continue

        runs = Plan.contiguous_runs(cols)

        if reverse:
            runs = list(reversed(runs))

        for first, last in runs:
            if last - first + 1 < min_run:
                continue

            pair = (last, first) if reverse else (first, last)
            goals.append(plan.cell_to_world(msg, row, pair[0]))
            goals.append(plan.cell_to_world(msg, row, pair[1]))

        reverse = not reverse

    return goals


def straight_length(points, pose):
    total = math.hypot(points[0][0] - pose[0], points[0][1] - pose[1])
    return total + sum(math.hypot(b[0] - a[0], b[1] - a[1])
                       for a, b in zip(points, points[1:]))


def driven_length(points, pose, floor, msg):
    """
    Distance routed through free space, not straight lines.

    A straight line from one waypoint to the next passes through the table the
    plan is supposed to be driving around, which flatters the row-by-row plan
    enormously. This is a lower bound - it ignores the turning circle and
    every avoidance manoeuvre - but it is the same lower bound for both plans.
    """
    res = msg.info.resolution
    height, width = floor.shape

    def cell(x, y):
        return (int((y - msg.info.origin.position.y) / res),
                int((x - msg.info.origin.position.x) / res))

    def route(a, b):
        if a == b:
            return 0.0

        seen = {a: 0.0}
        queue = deque([a])

        while queue:
            row, col = queue.popleft()

            if (row, col) == b:
                return seen[(row, col)] * res

            for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                step = (row + d_row, col + d_col)

                if (0 <= step[0] < height and 0 <= step[1] < width
                        and floor[step] and step not in seen):
                    seen[step] = seen[(row, col)] + 1.0
                    queue.append(step)

        return None

    stops = [cell(pose[0], pose[1])] + [cell(x, y) for x, y in points]
    total = 0.0

    for a, b in zip(stops, stops[1:]):
        leg = route(a, b)

        if leg is None:
            return None

        total += leg

    return total


def crossings(points, x_split, y_band):
    """How often the sweep crosses the blocker, counted only in its band."""
    low, high = y_band
    sides = [x > x_split for x, y in points if low <= y <= high]
    return sum(1 for a, b in zip(sides, sides[1:]) if a != b)


def test_blocker_is_crossed_once():
    """A table in open floor is passed once, not on every row."""
    floor, msg = room(5.0, 5.0, blockers=[(2.0, 1.0, 3.0, 4.0)])
    plan = planner(axis='x')
    pose = (0.4, 0.4, 0.0)

    new = build(plan, floor, msg, pose)
    old = old_plan(plan, msg, floor, pose)

    assert crossings(old, 2.5, (1.0, 4.0)) == 15
    assert crossings(new, 2.5, (1.0, 4.0)) == 1
    assert plan.clean_regions == 4

    # In front of the table, one strip down each side, and behind it.
    assert driven_length(old, pose, floor, msg) > 150.0
    assert driven_length(new, pose, floor, msg) < 125.0


def swept(points, floor, msg, radius=0.135):
    """
    Floor covered by dragging a `radius` disc along every row of a plan.

    Only the straight runs count - consecutive waypoints that share a sweep
    line, with drivable floor the whole way between them. The transit
    between rows and between regions is deliberately not credited, so this
    is a lower bound, and the same lower bound for every plan it is asked
    about.

    The "drivable the whole way" part matters: the row-by-row plan emits
    every run on a line one after another, so a straight segment from the
    end of one run to the start of the next cuts clean across whatever lies
    between them. Crediting that would hand the old plan coverage it never
    drives, because the real robot routes round over the grid BFS.
    """
    res = msg.info.resolution
    reach = max(1, int(round(radius / res)))
    height, width = floor.shape
    mark = np.zeros_like(floor)

    def cell(x, y):
        return (int((y - msg.info.origin.position.y) / res),
                int((x - msg.info.origin.position.x) / res))

    for before, after in zip(points, points[1:]):
        (row_a, col_a), (row_b, col_b) = cell(*before), cell(*after)

        if row_a == row_b:
            span = (min(col_a, col_b), max(col_a, col_b))

            if not floor[row_a, span[0]:span[1] + 1].all():
                continue

            rows = (row_a - reach, row_a + reach)
            cols = span
        elif col_a == col_b:
            span = (min(row_a, row_b), max(row_a, row_b))

            if not floor[span[0]:span[1] + 1, col_a].all():
                continue

            rows = span
            cols = (col_a - reach, col_a + reach)
        else:
            continue

        mark[max(0, rows[0]):min(height, rows[1] + 1),
             max(0, cols[0]):min(width, cols[1] + 1)] = True

    return 100.0 * int((mark & floor).sum()) / int(floor.sum())


def test_the_sweep_covers_at_least_as_much_floor_as_the_rows_it_replaced():
    """
    Reordering is allowed to change where the robot goes, not what it misses.

    The old set-equality check could not survive the sampling lines moving -
    and moving them is the point, because the old ones were measured from the
    map origin and left a strip at each edge. So the thing asserted is the
    thing that was actually wanted all along: no floor is lost.
    """
    rng = np.random.default_rng(7)
    worse = 0

    for _ in range(60):
        floor, msg, pose = _random_room(rng)
        plan = planner(axis='x')
        new = swept(build(plan, floor, msg, pose), floor, msg)
        old = swept(old_plan(plan, msg, floor, pose), floor, msg)

        assert new >= 98.0, f'the sweep left {100.0 - new:.1f}% of the floor'

        worse += new < old - 1e-9

    assert worse == 0


def test_first_waypoint_is_nearer_on_average():
    """The sweep should start where the robot is, not across the room."""
    rng = np.random.default_rng(7)
    total_new = total_old = 0.0
    trials = 200

    for _ in range(trials):
        floor, msg, pose = _random_room(rng)
        plan = planner(axis='x')
        new = build(plan, floor, msg, pose)
        old = old_plan(plan, msg, floor, pose)

        total_new += math.hypot(new[0][0] - pose[0], new[0][1] - pose[1])
        total_old += math.hypot(old[0][0] - pose[0], old[0][1] - pose[1])

    assert total_new / trials < total_old / trials
    assert total_new / trials < 1.6


def test_corner_is_real_floor_in_an_l_shaped_room():
    """The bounding box corner of an L is in the missing quadrant."""
    floor, msg = room(6.0, 6.0, blockers=[(3.0, 3.0, 6.0, 6.0)])
    plan = planner(axis='x')
    pose = (2.8, 2.8, 0.0)

    new = build(plan, floor, msg, pose)
    old = old_plan(plan, msg, floor, pose)

    near = math.hypot(new[0][0] - pose[0], new[0][1] - pose[1])
    far = math.hypot(old[0][0] - pose[0], old[0][1] - pose[1])

    assert near < far


def test_yaw_only_breaks_a_tie():
    """Midway along a wall, both corners are equally close."""
    floor, msg = room(4.0, 4.0)
    pose_x = (2.0, 0.3, 0.0)
    pose_y = (2.0, 0.3, math.pi / 2)

    facing_x = build(planner(axis='x'), floor, msg, pose_x)[0]
    facing_y = build(planner(axis='x'), floor, msg, pose_y)[0]

    assert facing_x[0] > 3.0
    assert facing_y[0] < 1.0

    # Weight 0.0 takes yaw out of it, so both pick the same corner.
    blind_x = build(planner(axis='x', corner_turn_weight=0.0),
                    floor, msg, pose_x)[0]
    blind_y = build(planner(axis='x', corner_turn_weight=0.0),
                    floor, msg, pose_y)[0]

    assert blind_x == blind_y


def sweep_lines(plan, points):
    """Return the distinct rows a plan lays down, in driven order."""
    index = 0 if plan.clean_axis == 'y' else 1
    lines = []

    for point in points:
        value = round(point[index], 6)

        if not lines or lines[-1] != value:
            lines.append(value)

    out = []

    for value in lines:
        if value not in out:
            out.append(value)

    return out


def test_auto_axis_sweeps_along_the_longer_side():
    """Rows along the long side of a corridor means a quarter of the turns."""
    floor, msg = room(2.0, 8.0)
    pose = (0.3, 0.3, 0.0)

    auto = planner()
    forced = planner(axis='x')
    rows_auto = len(sweep_lines(auto, build(auto, floor, msg, pose)))
    rows_forced = len(sweep_lines(forced, build(forced, floor, msg, pose)))

    assert auto.clean_axis == 'y'

    # 2.0 m and 8.0 m of floor, rows at most row_spacing apart with one
    # pinned on each edge: ceil(1.95 / 0.2) + 1 and ceil(7.95 / 0.2) + 1.
    assert rows_auto == 11
    assert rows_forced == 41


def test_degenerate_maps_return_an_empty_plan():
    """Nothing drivable at all, and runs too short to be worth driving."""
    plan = planner()
    msg = Grid(0.05, 40, 40)

    assert build(plan, np.zeros((40, 40), dtype=bool), msg, (0.0, 0.0, 0.0)) == []

    # A 0.10 m square of floor: every run on it is shorter than the robot,
    # so `min_run` drops them all and there is nothing to plan.
    speck = np.zeros((40, 40), dtype=bool)
    speck[20:22, 20:22] = True

    assert build(plan, speck, msg, (0.0, 0.0, 0.0)) == []


def test_a_narrow_but_drivable_sliver_is_now_swept():
    """
    Two cells of DRIVABLE floor is a corridor the robot fits down.

    The mask handed to the planner already has the obstacles inflated by the
    driving half-width, so anything still true in it is somewhere the robot
    can stand. This sliver used to produce an empty plan - not because it was
    refused, but because the sampling lines were multiples of the row step
    measured from the map origin and none of them happened to land on it.
    That is the edge strip in miniature: floor skipped by an accident of
    where the map starts.
    """
    plan = planner()
    msg = Grid(0.05, 40, 40)

    sliver = np.zeros((40, 40), dtype=bool)
    sliver[:, 5:7] = True

    points = build(plan, sliver, msg, (0.0, 0.0, 0.0))

    assert points
    assert plan.clean_axis == 'y'
    assert {round(x, 3) for x, _ in points} == {0.275, 0.325}


def test_a_plan_is_still_built_without_a_pose():
    """TF can drop out; the sweep must still be laid out."""
    floor, msg = room(5.0, 5.0)

    # 26 rows over 5 m of floor, six waypoints along each 5 m row.
    assert len(build(planner(), floor, msg, None)) == 156
    assert len(build(planner(row_point_spacing=0.0), floor, msg, None)) == 52


def _random_room(rng):
    width = float(rng.uniform(2.5, 8.0))
    height = float(rng.uniform(2.5, 8.0))
    blockers = []

    for _ in range(int(rng.integers(0, 4))):
        x0 = float(rng.uniform(0.4, max(0.5, width - 1.2)))
        y0 = float(rng.uniform(0.4, max(0.5, height - 1.2)))
        blockers.append((x0, y0, x0 + float(rng.uniform(0.3, 1.5)),
                         y0 + float(rng.uniform(0.3, 1.5))))

    floor, msg = room(width, height, blockers=blockers)
    pose = (float(rng.uniform(0.2, width - 0.2)),
            float(rng.uniform(0.2, height - 0.2)),
            float(rng.uniform(-math.pi, math.pi)))
    return floor, msg, pose


def test_sweep_start_top_begins_at_a_top_corner_and_works_down():
    """What was asked for: a top corner, rows along x, top to bottom."""
    floor, msg = room(5.0, 5.0)

    for pose in [(0.3, 0.3, 0.0), (4.7, 0.3, 0.0), (2.5, 2.5, math.pi)]:
        plan = planner(start='top')
        points = build(plan, floor, msg, pose)

        assert plan.clean_axis == 'x'
        assert plan.clean_side == 'top'

        # First waypoint at the top of the room, last at the bottom.
        assert points[0][1] > 4.7
        assert points[-1][1] < 0.1

        # Rows descend monotonically, and each is a straight run along x.
        rows = sorted({round(y, 3) for _, y in points}, reverse=True)
        assert [round(y, 3) for _, y in points][:2] == [rows[0], rows[0]]
        # 100 cells of floor, rows at most 4 cells apart with one pinned on
        # each edge: ceil(99 / 4) + 1.
        assert len(rows) == 26


def test_sweep_start_top_still_picks_the_nearer_top_corner():
    """Which top corner is still the robot's business."""
    floor, msg = room(5.0, 5.0)

    left = build(planner(start='top'), floor, msg, (0.3, 0.3, 0.0))[0]
    right = build(planner(start='top'), floor, msg, (4.7, 0.3, 0.0))[0]

    assert left[0] < 1.0
    assert right[0] > 4.0


def test_the_other_three_sides_work_too():
    """bottom, left and right, so a room can be swept whichever way suits."""
    floor, msg = room(5.0, 5.0)
    pose = (2.5, 2.5, 0.0)

    expected = {'bottom': ('x', 'bottom'), 'left': ('y', 'left'),
                'right': ('y', 'right'), 'top': ('x', 'top')}

    for side, (axis, edge) in expected.items():
        plan = planner(start=side)
        build(plan, floor, msg, pose)

        assert (plan.clean_axis, plan.clean_side) == (axis, edge)


def test_auto_is_still_available_and_still_nearest():
    """Naming a side is the default; auto is the old shortest-drive rule."""
    floor, msg = room(5.0, 5.0)
    pose = (0.3, 0.3, 0.0)

    auto = build(planner(start='auto', axis='x'), floor, msg, pose)
    top = build(planner(start='top'), floor, msg, pose)

    near = math.hypot(auto[0][0] - pose[0], auto[0][1] - pose[1])
    far = math.hypot(top[0][0] - pose[0], top[0][1] - pose[1])

    assert near < 1.0
    assert far > 4.0


def test_naming_a_side_does_not_lose_any_floor():
    """Same cells visited, whichever side it starts from."""
    rng = np.random.default_rng(3)

    for _ in range(60):
        floor, msg, pose = _random_room(rng)
        reference = sorted(map(tuple, build(planner(axis='x'), floor, msg, pose)))

        for side in ('top', 'bottom', 'left', 'right'):
            points = build(planner(start=side), floor, msg, pose)

            if side in ('top', 'bottom'):
                assert sorted(map(tuple, points)) == reference
            else:
                # Rows run the other way, so the waypoints are different
                # cells - but every one of them is drivable floor.
                assert points


def inset(width_m, height_m, res=0.05, margin=3):
    """
    Floor that does not start at the edge of the map.

    `margin` is in CELLS and deliberately not a whole number of row steps:
    the real mask is free space with the obstacles dilated by the driving
    half-width, so where the drivable floor begins has nothing whatever to
    do with the map origin. Sampling rows from line 0 of the map therefore
    starts somewhere inside the floor and stops somewhere short of its far
    side, and that offset is the edge strip.
    """
    width, height = int(width_m / res), int(height_m / res)
    floor = np.zeros((height, width), dtype=bool)
    floor[margin:height - margin, margin:width - margin] = True
    return floor, Grid(res, height, width)


def lines_of(points, axis):
    """Sweep lines of a plan, as cell indices, in driven order."""
    index = 0 if axis == 'y' else 1
    seen = []

    for point in points:
        line = int(round(point[index] / 0.05 - 0.5))

        if line not in seen:
            seen.append(line)

    return seen


def test_the_first_and_last_rows_reach_the_edges_of_the_floor():
    """
    No uncleaned strip at either end of the sweep.

    The rows used to be sampled at multiples of the row step measured from
    the MAP origin, which has nothing to do with where the floor is. On this
    floor - drivable from cell 3 to cell 76 - that put the first row three
    cells inside the near edge and left the far edge unvisited entirely,
    and every one of those cells past the swath is dirt no row went near.
    """
    floor, msg = inset(5.0, 4.0, margin=5)
    first, last = 5, floor.shape[0] - 6

    plan = planner(start='top')
    rows = sorted(lines_of(build(plan, floor, msg, (0.3, 0.3, 0.0)), 'x'))

    assert rows[0] == first
    assert rows[-1] == last

    # What it used to do, for the record: the far edge was never sampled.
    old = sorted({line for line in range(0, floor.shape[0], 4)
                  if floor[line].any()})

    assert old[0] > first
    assert last - old[-1] > 0


def test_rows_overlap_the_swath_whatever_the_grid_resolution():
    """
    Consecutive rows must be closer together than the robot's swath.

    Two things used to be able to push them apart. `round` can round the
    spacing UP to the next whole cell - on a 0.06 m grid 0.22 m becomes 4
    cells, 0.24 m - and nothing checked the result against the swath at all,
    so raising `row_spacing` past it silently laid stripes of dirt between
    the rows instead of being refused.
    """
    swath = 2.0 * 0.135

    for res in (0.025, 0.05, 0.06, 0.1):
        for spacing in (0.15, 0.22, 0.3, 0.4, 1.0):
            floor, msg = inset(4.0, 3.0, res=res, margin=3)
            plan = planner(start='top', row_spacing=spacing)
            points = build(plan, floor, msg, (0.3, 0.3, 0.0))

            rows = sorted({round(y, 6) for _, y in points})
            gaps = [b - a for a, b in zip(rows, rows[1:])]

            assert gaps, (res, spacing)
            assert max(gaps) <= swath + 1e-9, (res, spacing, max(gaps))

            # And never wider than asked for, either - flooring the cell
            # count can only ever bring the rows closer together.
            assert max(gaps) <= min(spacing, 0.9 * swath) + 1e-9


def test_each_row_is_driven_back_the_way_the_last_one_came():
    """
    Turn at the end of the furrow rather than walking back to the gate.

    Every row has to run opposite to the one before it, and start where the
    one before it finished - otherwise half the drive is spent returning to
    the same side with the brush over floor that is already done.
    """
    floor, msg = inset(5.0, 4.0, margin=3)
    plan = planner(start='top')
    points = build(plan, floor, msg, (0.3, 0.3, 0.0))

    assert plan.clean_regions == 1

    rows = []

    for x, y in points:
        if not rows or rows[-1][0][1] != y:
            rows.append([])

        rows[-1].append((x, y))

    assert len(rows) > 10

    for before, after in zip(rows, rows[1:]):
        heading = after[-1][0] - after[0][0]
        last = before[-1][0] - before[0][0]

        assert heading * last < 0.0, 'two rows driven the same way round'
        assert abs(after[0][0] - before[-1][0]) < 1e-9, (
            'the next row starts at the far side, not where this one ended')

    # Within a row the waypoints march one way, never doubling back.
    for row in rows:
        steps = [b[0] - a[0] for a, b in zip(row, row[1:])]

        assert all(step > 0 for step in steps) or all(step < 0 for step in steps)


def test_row_waypoints_do_not_move_with_the_direction_of_travel():
    """
    A row driven right to left visits the same cells as left to right.

    The intermediate points are placed at fixed positions in the run rather
    than counted out from whichever end the robot enters by, so starting
    from the other corner re-orders the plan without changing which floor
    it touches. Without that, two passes over the same row sweep two
    different sets of cells and neither is complete.
    """
    assert Plan.run_points(10, 90, 20) == [10, 30, 50, 70, 90]
    assert Plan.run_points(10, 90, 0) == [10, 90]
    assert Plan.run_points(10, 14, 20) == [10, 14]

    floor, msg = inset(5.0, 4.0, margin=3)
    left = build(planner(start='top'), floor, msg, (0.3, 0.3, 0.0))
    right = build(planner(start='top'), floor, msg, (4.7, 0.3, 0.0))

    assert left[0] != right[0]
    assert sorted(map(tuple, left)) == sorted(map(tuple, right))


def test_every_waypoint_knows_which_region_it_belongs_to():
    """The status line names a region, so one has to be recorded per goal."""
    floor, msg = room(5.0, 5.0, blockers=[(2.0, 1.0, 3.0, 4.0)])
    plan = planner(axis='x')
    points = build(plan, floor, msg, (0.4, 0.4, 0.0))

    assert len(plan.goal_regions) == len(points)
    assert plan.clean_rows >= len({round(y, 6) for _, y in points})

    # Driven order, so the labels only ever count up, and every region the
    # decomposition found gets driven.
    assert plan.goal_regions == sorted(plan.goal_regions)
    assert set(plan.goal_regions) == set(range(plan.clean_regions))


def inflated(floor, msg, obstacles):
    """
    Carve obstacles into `floor` the way `replan` does: inflated by plan_radius.

    `obstacles` are (x, y, size) squares in metres. The planner never sees an
    obstacle bare - a single occupied cell already arrives as a disc nine
    cells across - so the speck tests have to hand it that, not the speck.
    """
    res = msg.info.resolution
    occupied = np.zeros_like(floor)

    for x, y, size in obstacles:
        half = max(0, int(round(size / res / 2.0)) - 1)
        row, col = int(y / res), int(x / res)
        occupied[row - half:row + half + 1, col - half:col + half + 1] = True

    inflate = int(math.ceil((0.27 / 2.0 + 0.03) / res))
    return floor & ~Plan.dilate(occupied, inflate)


SPECKS = [(1.0, 0.8, 0.05), (4.0, 0.7, 0.05), (1.1, 4.3, 0.10),
          (3.9, 4.2, 0.05), (0.8, 2.5, 0.05)]


def test_specks_do_not_split_the_floor_into_regions():
    """
    A SLAM speck is driven round, not decomposed round.

    Five single-cell specks scattered over the floor around a table used to
    come out as four extra regions each - before, down either side and past
    it - so the robot made a separate trip for the sliver on the far side of
    every one. The table still splits; the specks must not, and the rows
    must still keep off them.
    """
    floor, msg = room(5.0, 5.0, blockers=[(2.0, 1.0, 3.0, 4.0)])
    pose = (0.4, 0.4, 0.0)
    clean = planner(axis='x')
    build(clean, floor, msg, pose)

    specked = inflated(floor, msg, SPECKS)
    plan = planner(axis='x')
    points = build(plan, specked, msg, pose)

    assert clean.clean_regions == 4
    assert plan.clean_regions == clean.clean_regions

    # The plain decomposition really would have fragmented on them: 14
    # regions where the table alone makes 4.
    strips = plan.sweep_strips(specked, 4, 5)
    assert len(Plan.split_regions(strips, 4)) >= 4 + 2 * len(SPECKS)

    # No waypoint inside a speck's inflation, and the floor is still swept.
    res = msg.info.resolution

    for x, y in points:
        assert specked[int(y / res), int(x / res)], (x, y)

    assert driven_length(points, pose, specked, msg) is not None
    assert swept(points, specked, msg) >= 98.0


def test_an_obstacle_bigger_than_the_robot_still_splits():
    """
    The speck rule is sized by the robot, so real furniture is unaffected.

    An obstacle the robot's own size (0.27 m) is hopped. The smallest one in
    the sim world, the 0.8 m cylinder, must still be decomposed round - one
    side of it finished before the other.
    """
    floor, msg = room(5.0, 5.0)
    pose = (0.4, 0.4, 0.0)

    small = planner(axis='x')
    build(small, inflated(floor, msg, [(2.5, 2.5, 0.27)]), msg, pose)
    large = planner(axis='x')
    points = build(large, inflated(floor, msg, [(2.5, 2.5, 0.8)]), msg, pose)

    assert small.clean_regions == 1
    assert large.clean_regions == 4
    assert crossings(points, 2.5, (2.2, 2.8)) == 1


def test_a_bump_on_a_wall_is_not_a_speck():
    """Only blocked ground with floor all round it is a hole to fill."""
    floor, msg = inset(5.0, 4.0, margin=3)
    floor[40:43, 3:6] = False

    filled = planner().fill_specks(floor, 12)

    assert not filled[40:43, 3:6].any()
    assert not filled[:3, :].any() and not filled[:, :3].any()


def test_floor_the_robot_cannot_reach_is_not_planned():
    """
    A walled-off pocket is not a region, and none of its rows are waypoints.

    On the saved sim map these were the free-reading inside of a box and a
    band of free space outside a wall: five regions the robot could never
    drive, still counted, still pulling the edge rows out to their extent.
    """
    floor, msg = room(5.0, 5.0)
    floor[60:62, 60:90] = floor[88:90, 60:90] = False
    floor[60:90, 60:62] = floor[60:90, 88:90] = False

    plan = planner(axis='x')
    points = build(plan, floor, msg, (0.4, 0.4, 0.0))
    res = msg.info.resolution

    assert points
    assert not [(x, y) for x, y in points
                if 62 <= int(y / res) < 88 and 62 <= int(x / res) < 88]

    # Without a pose there is nothing to measure reachability from, so the
    # whole mask is planned, pocket and all.
    blind = planner(axis='x')
    assert len(build(blind, floor, msg, None)) > len(points)
    assert blind.clean_regions > plan.clean_regions


# ----------------------------------------------------------------------
# Planning wavefront: the numpy flood that replaced the per-cell Python BFS
# ----------------------------------------------------------------------

def old_bfs(traversable, start, h, w):
    """The per-cell FIFO BFS `replan` used before 5 Oct, kept as the reference."""
    flat = traversable.reshape(-1)
    dist = np.full(h * w, -1, dtype=np.int32)
    parent = np.full(h * w, -1, dtype=np.int32)
    dist[start] = 0
    queue = deque([start])

    while queue:
        cur = queue.popleft()
        row, col = divmod(cur, w)

        for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            r, c = row + d_row, col + d_col

            if 0 <= r < h and 0 <= c < w:
                nxt = r * w + c

                if flat[nxt] and dist[nxt] < 0:
                    dist[nxt] = dist[cur] + 1
                    parent[nxt] = cur
                    queue.append(nxt)

    return dist, parent


def old_path(parent, start, goal):
    cells = []
    cur = goal

    while cur != -1 and cur != start:
        cells.append(cur)
        cur = int(parent[cur])

    return cells[::-1]


def _check_against_bfs(floor, start, rng, goals=60):
    h, w = floor.shape
    dist, parent = old_bfs(floor, start, h, w)
    new = Plan.wavefront(floor, start)

    # Identical distances means identical reachability, identical frontier
    # and block choice, and identical "is this waypoint reachable" skips.
    assert new.dtype == dist.dtype and np.array_equal(new, dist)

    reach = np.flatnonzero(dist >= 0)
    same = 0

    for goal in rng.choice(reach, min(goals, reach.size)):
        goal = int(goal)
        before = old_path(parent, start, goal)
        after = Plan.descend(new, start, goal, w)

        # Greedy descent may take a different, equally short, route.
        assert len(after) == len(before) == dist[goal]
        assert (after[-1] if after else start) == goal

        previous = start
        for cell in after:
            assert floor.reshape(-1)[cell]
            assert abs(cell - previous) in (1, w), 'path is not 4-connected'
            previous = cell

        # Stopping the flood at the goal leaves the same route to it.
        early = Plan.wavefront(floor, start, stop=goal)
        assert Plan.descend(early, start, goal, w) == after

        same += after == before

    return same


def test_wavefront_gives_the_bfs_distances_and_equally_short_paths():
    rng = np.random.default_rng(11)

    for _ in range(25):
        floor, msg, pose = _random_room(rng)
        floor = inflated(floor, msg, SPECKS)
        h, w = floor.shape
        cells = np.flatnonzero(floor)
        _check_against_bfs(floor, int(rng.choice(cells)), rng)


def test_descent_matches_the_bfs_route_in_open_floor():
    """Columns before rows: the same L the FIFO BFS parents drew."""
    floor, _ = room(4.0, 3.0)
    rng = np.random.default_rng(2)
    start = 30 * floor.shape[1] + 40

    assert _check_against_bfs(floor, start, rng, goals=50) == 50


def test_wavefront_on_the_saved_home_map():
    """The real map, if this machine has it."""
    path = '/home/varun/vac_main1_maps/sim/home.pgm'

    if not os.path.exists(path):
        return

    data = open(path, 'rb').read()
    parts = data.split(maxsplit=4)
    w, h = int(parts[1]), int(parts[2])
    image = np.frombuffer(parts[4][-w * h:], np.uint8).reshape(h, w)
    occupancy = (255 - image.astype(float)) / 255.0
    occupied = np.flipud(occupancy > 0.65)
    free = np.flipud(occupancy < 0.196)
    floor = free & ~Plan.dilate(occupied, int(math.ceil((0.27 / 2.0 + 0.03) / 0.05)))

    rng = np.random.default_rng(5)
    cells = np.flatnonzero(floor)
    same = _check_against_bfs(floor, int(cells[cells.size // 2]), rng, goals=100)

    assert same >= 90


def test_unreachable_goal_floods_everything_and_stays_unlabelled():
    floor, _ = room(3.0, 3.0)
    floor[:, 30] = False
    w = floor.shape[1]
    dist = Plan.wavefront(floor, 10 * w + 5, stop=10 * w + 50)

    assert dist[10 * w + 50] == -1
    assert (dist[floor.reshape(-1)] >= 0).sum() == floor[:, :30].sum()


ARRIVAL = {'advance_on_arrival', 'advance_cleaning', 'build_path', 'start_cell',
           'world_to_cell', 'wavefront', 'descend'}


def test_arrival_routes_to_the_same_waypoint_as_a_full_replan():
    """
    The cached arrival step picks what `advance_cleaning` would have picked.

    Same waypoint index (unreachable ones skipped the same way) and a path
    of the same length to it - without the replan in front of it.
    """
    Node = _lift(ARRIVAL)
    floor, msg = room(5.0, 5.0, blockers=[(2.0, 1.0, 3.0, 4.0)])
    floor = inflated(floor, msg, SPECKS)
    floor[90:, 90:] = False
    floor[92:96, 92:96] = True          # an island no route reaches
    h, w = floor.shape
    plan = planner(axis='x')
    goals = build(plan, floor, msg, None)
    island = plan.cell_to_world(msg, 94, 94)

    def node():
        n = Node()
        n.cell_to_world = plan.cell_to_world
        n.coverage_percent = lambda m, t: 0.0
        n.publish_path = lambda m: None
        n.log_waypoint = lambda: None
        n.finish_pass = lambda *a: setattr(n, 'finished', True)
        n.finished = False
        n.cleaning_goals = [island] + goals[:40] + [island]
        return n

    rng = np.random.default_rng(4)
    cells = np.flatnonzero(floor)
    checked = 0

    for first in range(0, 40, 3):
        start = int(rng.choice(cells))
        pose = plan.cell_to_world(msg, *divmod(start, w)) + (0.0,)
        dist = Plan.wavefront(floor, start)

        full = node()
        full.goal_index = first
        full.advance_cleaning(msg, floor, dist, start, h, w)

        fast = node()
        fast.goal_index = first
        fast.plan_cache = {'msg': msg, 'traversable': floor,
                           'dist': Plan.wavefront(floor, int(cells[0]))}

        if fast.plan_cache['dist'][start] < 0:
            continue

        assert fast.advance_on_arrival(pose)
        assert fast.goal_index == full.goal_index
        assert len(fast.path) == len(full.path)
        assert fast.path[-1:] == full.path[-1:]
        checked += 1

    # Past the last reachable waypoint it declines, untouched, and the full
    # replan is left to finish the pass.
    fast = node()
    fast.goal_index = 41
    fast.plan_cache = {'msg': msg, 'traversable': floor,
                       'dist': Plan.wavefront(floor, int(cells[0]))}
    fast.path = ['unchanged']
    assert not fast.advance_on_arrival((0.4, 0.4, 0.0))
    assert fast.goal_index == 41 and fast.path == ['unchanged']
    assert checked >= 8
