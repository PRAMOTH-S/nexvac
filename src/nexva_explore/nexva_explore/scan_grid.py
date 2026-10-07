"""
Turning one lidar scan into grid cells, once, correctly.

`local_map` and `map_updater` both need the same thing: given a pose and a
scan, which cells did the beams pass through (free) and which cells did they
end on (occupied). They used to carry a copy each, and the copies shared three
faults. This module is the single implementation, so a fix lands in both.

What was wrong with the old marching, and what this does instead:

**Distant cells were cleared on evidence that cannot support it.** A beam was
followed all the way out to `max_range` - 8 m for the updater, 12 m for the
lidar - and every cell it crossed was voted free. But the beams fan out. With
450 beams over 360 degrees the gap between two of them is 0.8 deg, which is
1.1 cm at 1 m and 11 cm at 8 m. Once that gap is wider than a cell, the beams
stop sweeping a surface and start combing it: they clear stripes and leave the
cells between them untouched, and any small error in heading smears those
stripes sideways across whatever is really there. That is how an obstacle a
long way off gets erased by a robot that has moved away from it and can no
longer see it properly. Free space is now only written out to `clear_range`,
which by default is the range at which the beam gap reaches one cell -
resolution / angle_increment, 3.6 m for an RPLIDAR C1 on a 5 cm grid. Echoes
are still recorded out to `max_range`; it is *clearing* that is restricted,
because clearing is the destructive half.

**Evidence per cell depended on beam geometry rather than on the world.** The
beam was sampled every half cell, so a cell near the robot collected a free
vote from every sample of every beam that grazed it - a dozen or more - while
a cell further out collected one or none, and a diagonal beam voted at a
different rate from an axis-aligned one. Cells now vote at most once per scan:
one scan, one vote, wherever the cell is. That is what stops the near field
turning into a solid block of certainty while the mid field stays speckled.

**Cells outside the grid were folded onto its edge.** The old code used
`int()` / `astype(int32)`, which truncates towards zero, so a beam leaving the
grid at x = -0.4 cells landed on column 0 instead of being discarded, painting
a false stripe of free space up the left and bottom edges. Indices are floored
and bounds-checked instead.

A cell that is both crossed and hit in the same scan is occupied. The old code
tried to express that by adding `L_OCCUPIED - L_FREE` at the endpoint, which
cancels exactly one free vote - fine when the cell got exactly one, wrong when
it got eleven. Hit cells are removed from the free set outright.
"""

import math

import numpy as np


def auto_clear_range(scan, resolution):
    """
    How far out this scan can still be trusted to clear cells.

    The answer is the range at which two adjacent beams are one cell apart:
    beyond it they no longer cover the ground between them, so marking that
    ground free is a guess rather than an observation.

    RPLIDAR C1, 450 beams over 360 deg, 5 cm cells: 0.05 / 0.01396 = 3.6 m.
    """
    increment = abs(float(scan.angle_increment))
    if increment < 1e-9:
        return float(scan.range_max)

    return resolution / increment


def scan_cells(scan, x, y, yaw, resolution, origin_x, origin_y,
               width, height, clear_range, max_range, mark_range=None):
    """
    Cells this scan says are free, and cells it says are occupied.

    Returns `(free_rows, free_cols, hit_rows, hit_cols)`, each a flat int32
    array, each cell appearing at most once, and no cell in both.

    Three ranges, because the three things they bound fail at different
    distances:

    - `clear_range` bounds **free space**. Past it the beams comb the ground
      instead of sweeping it (see the module docstring).
    - `mark_range` bounds **occupied cells**. An echo is a real return at any
      distance, but *which cell it belongs in* is only known to within the
      beam's own width, `range * angle_increment`, plus whatever the pose is
      wrong by. Once that is wider than a cell, marking one particular cell
      occupied is a guess. Defaults to `max_range`, i.e. no extra limit.
    - `max_range` bounds **the sensor band** - readings beyond it are dropped
      entirely, neither clearing nor marking.

    A return further out than `mark_range` still clears the space in front of
    it, up to `clear_range`: the beam did travel through that space, and that
    is an observation regardless of where it eventually stopped.

    Everything is in the frame the grid is in; `origin_x/origin_y` are its
    bottom-left corner in metres.
    """
    if mark_range is None:
        mark_range = max_range

    ranges = np.asarray(scan.ranges, dtype=np.float32)
    if ranges.size == 0:
        empty = np.empty(0, dtype=np.int32)
        return empty, empty, empty, empty

    angles = (yaw + scan.angle_min
              + np.arange(ranges.size, dtype=np.float32) * scan.angle_increment)
    cos = np.cos(angles)
    sin = np.sin(angles)

    finite = np.isfinite(ranges)

    # An echo: a real return inside the sensor's band. Everything else is
    # "saw nothing out to here".
    echo = (finite
            & (ranges >= scan.range_min)
            & (ranges < scan.range_max)
            & (ranges <= max_range))

    # Near enough that the cell it landed in is known, not guessed.
    hit = echo & (ranges <= mark_range)

    # A reading below range_min is a sensor artefact, not an observation of
    # the floor in front of the wheels, so it clears nothing.
    too_close = finite & (ranges < scan.range_min)

    # How far along each beam free space extends: to the echo if there is
    # one, otherwise to the edge of the trust radius, and never past it.
    # Keyed on `echo`, not on `hit`: a return too far away to place is still
    # a return, and clearing straight through it would be wrong.
    clear_to = np.where(echo, ranges, clear_range)
    clear_to = np.minimum(clear_to, clear_range)
    clear_to[too_close] = 0.0

    free_rows, free_cols = _cells_along(
        clear_to, x, y, cos, sin, resolution, origin_x, origin_y, width, height)

    hit_rows, hit_cols = _cells_at(
        ranges[hit], x, y, cos[hit], sin[hit],
        resolution, origin_x, origin_y, width, height)

    # One vote per cell per scan, and occupied wins over free.
    free_flat = np.unique(free_rows.astype(np.int64) * width + free_cols)
    hit_flat = np.unique(hit_rows.astype(np.int64) * width + hit_cols)
    free_flat = np.setdiff1d(free_flat, hit_flat, assume_unique=True)

    return (
        (free_flat // width).astype(np.int32),
        (free_flat % width).astype(np.int32),
        (hit_flat // width).astype(np.int32),
        (hit_flat % width).astype(np.int32),
    )


def _cells_along(distances, x, y, cos, sin, resolution,
                 origin_x, origin_y, width, height):
    """Cells every beam passes through, up to its own distance."""
    longest = float(distances.max(initial=0.0))
    if longest <= 0.0:
        empty = np.empty(0, dtype=np.int32)
        return empty, empty

    # Half a cell keeps the march from stepping over a cell diagonally. The
    # duplicates this creates are removed by the unique() above, so sampling
    # finer costs work but never distorts the evidence.
    step = 0.5 * resolution
    samples = int(math.ceil(longest / step)) + 1

    reach = np.arange(samples, dtype=np.float32) * step
    inside = reach[None, :] < distances[:, None]
    if not inside.any():
        empty = np.empty(0, dtype=np.int32)
        return empty, empty

    xs = x + reach[None, :] * cos[:, None]
    ys = y + reach[None, :] * sin[:, None]

    return _to_cells(xs[inside], ys[inside], resolution,
                     origin_x, origin_y, width, height)


def _cells_at(distances, x, y, cos, sin, resolution,
              origin_x, origin_y, width, height):
    """Find the cell each echo landed in."""
    if distances.size == 0:
        empty = np.empty(0, dtype=np.int32)
        return empty, empty

    return _to_cells(x + distances * cos, y + distances * sin,
                     resolution, origin_x, origin_y, width, height)


def _to_cells(xs, ys, resolution, origin_x, origin_y, width, height):
    """
    Metres to grid indices, floored and bounds-checked.

    Floored, not truncated: truncation rounds towards zero, so a point just
    outside the bottom-left corner becomes cell 0 instead of being dropped.
    """
    cols = np.floor((xs - origin_x) / resolution).astype(np.int64)
    rows = np.floor((ys - origin_y) / resolution).astype(np.int64)

    keep = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)

    return rows[keep].astype(np.int32), cols[keep].astype(np.int32)


def clusters_at_least(mask, minimum):
    """
    Drop everything in `mask` that is not part of a blob of `minimum` cells.

    A change worth reporting is a thing - a chair, a door, a person. A change
    of one or two cells is a beam that clipped an edge, a scan-match that
    settled a centimetre differently, or noise. Without this the change grid
    carries both at once and nothing downstream can tell them apart.

    4-connected labelling by index propagation: seed each cell with its own
    flat index and repeatedly take the maximum over its neighbours until
    nothing moves, which takes about as many passes as the widest blob. On the
    windows this runs on - 80x80 - that is a handful of array operations.
    """
    if minimum <= 1 or not mask.any():
        return mask

    labels = np.where(mask, np.arange(mask.size).reshape(mask.shape), -1)

    for _ in range(mask.size):
        spread = labels.copy()
        spread[1:, :] = np.maximum(spread[1:, :], labels[:-1, :])
        spread[:-1, :] = np.maximum(spread[:-1, :], labels[1:, :])
        spread[:, 1:] = np.maximum(spread[:, 1:], labels[:, :-1])
        spread[:, :-1] = np.maximum(spread[:, :-1], labels[:, 1:])
        spread[~mask] = -1

        if np.array_equal(spread, labels):
            break

        labels = spread

    sizes = np.bincount(labels[mask].ravel())

    return mask & (sizes[np.where(mask, labels, 0)] >= minimum)
