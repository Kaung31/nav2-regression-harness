#!/usr/bin/env python3
"""Invariants of the canonical doorway. Each one, if broken, produces a
plausible-looking threshold that means nothing.

    python3 src/harness_runner/test/test_doorway.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import numpy as np                                          # noqa: E402
from harness_runner import scenario_gen as sg               # noqa: E402

WIDTHS = [round(0.5 + i * 0.05, 2) for i in range(21)]       # 0.50 .. 1.50


def test_path_length_is_constant():
    """The whole reason this generator exists.

    batch_r3 showed path length -- not gap width -- separates pass from fail
    in the random rooms. If it moves with width here too, the instrument has
    the same confound as the thing it replaces.
    """
    lengths = {sg.generate_doorway(w).optimal_path for w in WIDTHS}
    assert len(lengths) == 1, f"path length varies with width: {sorted(lengths)}"
    assert abs(lengths.pop() - 4.0) < 0.06, "expected the straight 4 m route"


def test_geometry_is_pinned():
    for w in WIDTHS:
        L = sg.generate_doorway(w)
        assert L.start == (0.0, 0.0)        # harness.launch.py spawns here
        assert L.goal == (0.0, 4.0)
        assert L.n_obstacles == 0
        assert L.n_walls == 1
        # 4 perimeter boxes + 2 wall segments, always.
        assert len(L.boxes) == 6, (w, len(L.boxes))
        left, right = L.boxes[4], L.boxes[5]
        assert abs(left.cx + right.cx) < 1e-9, "aperture is not centred on x=0"
        assert abs(left.sx - right.sx) < 1e-9, "wall segments differ in length"
        assert left.cy == right.cy == sg.DOORWAY_WALL_Y
        # The aperture is the requested width, to the millimetre.
        opening = (right.cx - right.sx / 2) - (left.cx + left.sx / 2)
        assert abs(opening - w) < 1e-9, (w, opening)


def test_no_alternative_route():
    """Sealing the gap must make the goal unreachable.

    If any way round survives, a 'failure' at a narrow width might just be the
    planner preferring a detour, and the measurement means nothing.
    """
    for w in (0.5, 1.0, 1.5):
        L = sg.generate_doorway(w)
        sealed = list(L.boxes) + [sg.Box(0.0, sg.DOORWAY_WALL_Y, w, sg.WALL_T)]
        blocked = sg.to_grid(sealed, sg.ROBOT_RADIUS) | sg._outside_mask()
        assert sg.bfs_path_length(blocked, (0.0, 0.0), L.goal) is None, \
            f"a route exists around the wall at w={w}"


def test_ground_truth_refuses_below_the_footprint():
    """Below 2 x ROBOT_RADIUS the robot physically cannot fit, and BFS must
    say so rather than the harness discovering it at runtime."""
    assert sg.generate_doorway(0.40) is None
    assert sg.generate_doorway(0.44) is None      # exactly 2 x 0.22
    assert sg.generate_doorway(0.50) is not None


def test_map_and_world_agree():
    """Design rule 7: both derive from one box list, so the occupancy grid
    must show the aperture where the SDF puts it."""
    L = sg.generate_doorway(1.0)
    occ = sg.to_grid(L.boxes)
    row, _ = sg.w2g(0.0, sg.DOORWAY_WALL_Y)
    _, c_mid = sg.w2g(0.0, sg.DOORWAY_WALL_Y)
    assert not occ[row, c_mid], "aperture centre is occupied in the map"
    _, c_off = sg.w2g(1.5, sg.DOORWAY_WALL_Y)
    assert occ[row, c_off], "wall is missing 1.5 m off centre"
    # Free span across the wall row should be the gap, within one cell.
    # _outside_mask matters here: to_grid only marks boxes, so cells beyond
    # the perimeter read as free and would make the aperture look 10 m wide.
    navigable = ~(occ | sg._outside_mask())
    free = np.where(navigable[row])[0]
    span = (free.max() - free.min() + 1) * sg.RES if len(free) else 0
    assert abs(span - 1.0) <= 2 * sg.RES, f"aperture reads {span:.2f} m, want 1.0"
    # ...and it must be one contiguous opening, not two gaps at the edges.
    assert len(free) == free.max() - free.min() + 1, "aperture is not contiguous"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
