"""
fov.py

Recursive shadowcasting field-of-view, decoupled from any particular grid.

Callers supply two predicates:
    is_blocked(x, y) -> bool   # True if sight is blocked here (walls AND out-of-bounds)
    in_bounds(x, y)  -> bool   # True if (x, y) is a real cell worth reporting

Used by:
- SensorModel, over the true map (what the agent actually sees)
- SLAMLocalization, over the agent's internal map (what it *would* see from a
  candidate pose) for scan matching.
"""

# Per-octant coordinate transforms (xx, xy, yx, yy).
_MULT = (
    (1, 0, 0, -1, -1, 0, 0, 1),
    (0, 1, -1, 0, 0, -1, 1, 0),
    (0, 1, 1, 0, 0, -1, -1, 0),
    (1, 0, 0, 1, -1, 0, 0, -1),
)


def visible_cells(is_blocked, in_bounds, cx, cy, radius) -> set:
    """Set of (x, y) cells visible from (cx, cy) within radius, with occlusion."""
    visible = set()

    if in_bounds(cx, cy):
        visible.add((cx, cy))

    for octant in range(8):
        _cast_light(
            is_blocked, in_bounds, visible, cx, cy,
            row=1, start_slope=1.0, end_slope=0.0, radius=radius,
            xx=_MULT[0][octant], xy=_MULT[1][octant],
            yx=_MULT[2][octant], yy=_MULT[3][octant],
        )

    return visible


def _cast_light(is_blocked, in_bounds, visible, cx, cy, row,
                start_slope, end_slope, radius, xx, xy, yx, yy):
    """Scan one octant outward, recursing into the sub-octant after each blocker."""
    if start_slope < end_slope:
        return

    radius_sq = radius * radius
    new_start = start_slope

    for j in range(row, radius + 1):
        dx, dy = -j - 1, -j
        blocked = False

        while dx <= 0:
            dx += 1

            X = cx + dx * xx + dy * xy
            Y = cy + dx * yx + dy * yy

            l_slope = (dx - 0.5) / (dy + 0.5)
            r_slope = (dx + 0.5) / (dy - 0.5)

            if start_slope < r_slope:
                continue
            elif end_slope > l_slope:
                break

            if dx * dx + dy * dy <= radius_sq and in_bounds(X, Y):
                visible.add((X, Y))

            wall = is_blocked(X, Y)

            if blocked:
                if wall:
                    new_start = r_slope
                    continue
                else:
                    blocked = False
                    start_slope = new_start
            else:
                if wall and j < radius:
                    blocked = True
                    _cast_light(
                        is_blocked, in_bounds, visible, cx, cy,
                        j + 1, start_slope, l_slope, radius, xx, xy, yx, yy,
                    )
                    new_start = r_slope

        if blocked:
            break
