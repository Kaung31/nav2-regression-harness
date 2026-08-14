#!/usr/bin/env python3
"""Generate matched Gazebo worlds and occupancy maps from a seed."""
import math
import os
import random
from collections import deque
from dataclasses import dataclass, field

import numpy as np

RES = 0.05
HALF = 5.0
WALL_T = 0.2
MARGIN = 0.2
EXTENT = HALF + WALL_T / 2 + MARGIN      # 5.3
GRID_N = int(round(2 * EXTENT / RES))    # 212
ROBOT_RADIUS = 0.22
NAV_INFLATION = 0.55


@dataclass
class Box:
    cx: float
    cy: float
    sx: float
    sy: float
    sz: float = 1.0


@dataclass
class Layout:
    seed: int
    boxes: list = field(default_factory=list)
    start: tuple = (0.0, 0.0)
    goal: tuple = (0.0, 0.0)
    gap_width: float = 1.5
    n_walls: int = 0
    n_obstacles: int = 0
    optimal_path: float = 0.0


def _perimeter():
    o = HALF + WALL_T / 2
    return [
        Box(0.0,  HALF, 2 * o, WALL_T),
        Box(0.0, -HALF, 2 * o, WALL_T),
        Box( HALF, 0.0, WALL_T, 2 * o),
        Box(-HALF, 0.0, WALL_T, 2 * o),
    ]


def _wall_with_gap(rng, vertical, pos, gap_width):
    """A wall across the room with one doorway."""
    inner = HALF - WALL_T / 2
    gap_c = rng.uniform(-inner + gap_width, inner - gap_width)
    lo_len = (gap_c - gap_width / 2) - (-inner)
    hi_len = inner - (gap_c + gap_width / 2)
    out = []
    if lo_len > 0.15:
        c = (-inner + (gap_c - gap_width / 2)) / 2
        out.append(Box(pos, c, WALL_T, lo_len) if vertical
                   else Box(c, pos, lo_len, WALL_T))
    if hi_len > 0.15:
        c = ((gap_c + gap_width / 2) + inner) / 2
        out.append(Box(pos, c, WALL_T, hi_len) if vertical
                   else Box(c, pos, hi_len, WALL_T))
    return out


def to_grid(boxes, inflate=0.0):
    """Occupancy grid. inflate expands obstacles by this many metres."""
    g = np.zeros((GRID_N, GRID_N), dtype=bool)
    ys = EXTENT - np.arange(GRID_N) * RES
    xs = -EXTENT + np.arange(GRID_N) * RES
    X, Y = np.meshgrid(xs, ys)
    for b in boxes:
        hx = b.sx / 2 + inflate
        hy = b.sy / 2 + inflate
        g |= (np.abs(X - b.cx) <= hx) & (np.abs(Y - b.cy) <= hy)
    return g


def _outside_mask():
    """Cells beyond the perimeter walls are not navigable."""
    ys = EXTENT - np.arange(GRID_N) * RES
    xs = -EXTENT + np.arange(GRID_N) * RES
    X, Y = np.meshgrid(xs, ys)
    o = HALF + WALL_T / 2
    return (np.abs(X) > o) | (np.abs(Y) > o)


def w2g(x, y):
    return (int(round((EXTENT - y) / RES)), int(round((x + EXTENT) / RES)))


def bfs_path_length(blocked, start, goal):
    """8-connected BFS. Returns metres, or None if unreachable."""
    r0, c0 = w2g(*start)
    r1, c1 = w2g(*goal)
    if blocked[r0, c0] or blocked[r1, c1]:
        return None
    dist = np.full(blocked.shape, -1.0)
    dist[r0, c0] = 0.0
    q = deque([(r0, c0)])
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    while q:
        r, c = q.popleft()
        if (r, c) == (r1, c1):
            return dist[r, c] * RES
        for dr, dc, w in steps:
            nr, nc = r + dr, c + dc
            if 0 <= nr < GRID_N and 0 <= nc < GRID_N \
               and not blocked[nr, nc] and dist[nr, nc] < 0:
                dist[nr, nc] = dist[r, c] + w
                q.append((nr, nc))
    return None


def generate(seed, n_walls=None, gap_width=None, n_obstacles=None,
             min_goal_dist=4.0, max_tries=60):
    """Deterministic layout from a seed. Returns None if no valid goal found."""
    rng = random.Random(seed)

    # Draw every random parameter unconditionally, then apply overrides.
    # Skipping a draw would shift the whole stream and change the room.
    _nw = rng.randint(1, 3)
    _gw = round(rng.uniform(0.7, 2.0), 2)
    _no = rng.randint(0, 6)
    n_walls     = _nw if n_walls     is None else n_walls
    gap_width   = _gw if gap_width   is None else gap_width
    n_obstacles = _no if n_obstacles is None else n_obstacles

    boxes = _perimeter()
    used = []
    for _ in range(n_walls):
        for _ in range(20):
            vertical = rng.random() < 0.5
            pos = round(rng.uniform(-3.0, 3.0), 2)
            if abs(pos) < 0.8:                    # keep the spawn clear
                continue
            if all(abs(pos - p) > 1.5 or v != vertical for p, v in used):
                used.append((pos, vertical))
                boxes += _wall_with_gap(rng, vertical, pos, gap_width)
                break

    obstacles = []
    for _ in range(n_obstacles):
        for _ in range(20):
            ox = round(rng.uniform(-4.0, 4.0), 2)
            oy = round(rng.uniform(-4.0, 4.0), 2)
            if math.hypot(ox, oy) < 1.2:          # keep the spawn clear
                continue
            # never near a wall line -- an obstacle there could seal the doorway
            if any(abs((ox if v else oy) - p) < 1.0 for p, v in used):
                continue
            s = round(rng.uniform(0.3, 0.7), 2)
            obstacles.append(Box(ox, oy, s, s, 0.6))
            break
    boxes += obstacles

    blocked = to_grid(boxes, ROBOT_RADIUS) | _outside_mask()
    nav_blocked = to_grid(boxes, NAV_INFLATION) | _outside_mask()
    r0, c0 = w2g(0.0, 0.0)
    if nav_blocked[r0, c0]:
        return None

    goal, opt = None, None
    for _ in range(max_tries):
        gx = round(rng.uniform(-4.3, 4.3), 2)
        gy = round(rng.uniform(-4.3, 4.3), 2)
        if math.hypot(gx, gy) < min_goal_dist:
            continue
        gr, gc = w2g(gx, gy)
        if nav_blocked[gr, gc]:
            continue
        d = bfs_path_length(blocked, (0.0, 0.0), (gx, gy))
        if d is not None:
            goal, opt = (gx, gy), d
            break
    if goal is None:
        return None

    return Layout(seed=seed, boxes=boxes, start=(0.0, 0.0), goal=goal,
                  gap_width=gap_width, n_walls=n_walls,
                  n_obstacles=len(obstacles), optimal_path=round(opt, 3))


# --------------------------------------------------------------------------
# Phase 6.5: the canonical doorway -- the measurement instrument
# --------------------------------------------------------------------------
# The random rooms are the discovery mechanism and cannot produce a threshold:
# batch_r3 measured gap width as FLAT across always-pass / flips / always-fail
# (means 1.24 / 1.34 / 1.17), while path length separated them cleanly. So the
# controlled experiment pins path length above all else.
#
# Spawn is (0, 0) because harness.launch.py hardcodes it. Building the geometry
# around the fixed spawn -- rather than moving the robot -- is what lets the
# straight line from start, through a centred aperture, to the goal stay
# EXACTLY the same length at every width.
DOORWAY_WALL_Y = 2.0
DOORWAY_GOAL_Y = 4.0


def generate_doorway(gap_width, wall_y=DOORWAY_WALL_Y, goal_y=DOORWAY_GOAL_Y):
    """One wall, one centred aperture, gap width the only variable.

    Held constant by construction: room size, wall thickness, spawn, goal,
    approach angle (head-on -- the aperture sits on the straight line between
    start and goal), and path length. The wall spans the full room, so the gap
    is the only route: a failure means the stack could not use the aperture,
    not that it preferred a different way round.

    Returns None when the footprint physically cannot fit, i.e. below
    2 x ROBOT_RADIUS. That is ground truth refusing, not an error.
    """
    inner = HALF - WALL_T / 2
    half_gap = gap_width / 2.0
    seg = inner - half_gap
    if seg <= 0:
        raise ValueError(
            f"gap_width {gap_width} leaves no wall either side of the aperture")

    boxes = _perimeter()
    for sign in (-1, 1):
        boxes.append(Box(sign * (inner + half_gap) / 2.0, wall_y, seg, WALL_T))

    goal = (0.0, goal_y)
    blocked = to_grid(boxes, ROBOT_RADIUS) | _outside_mask()
    opt = bfs_path_length(blocked, (0.0, 0.0), goal)
    if opt is None:
        return None

    return Layout(seed=int(round(gap_width * 100)), boxes=boxes,
                  start=(0.0, 0.0), goal=goal, gap_width=gap_width,
                  n_walls=1, n_obstacles=0, optimal_path=round(opt, 3))


def write_world(layout, path):
    parts = ['<?xml version="1.0"?>', '<sdf version="1.10">',
             f'  <world name="scenario_{layout.seed}">',
             '''    <physics name="fast" type="ignored">
      <max_step_size>0.005</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>200</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.5 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>
    </model>''']
    for i, b in enumerate(layout.boxes):
        parts.append(f'''    <model name="box_{i}">
      <static>true</static>
      <pose>{b.cx} {b.cy} {b.sz / 2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{b.sx} {b.sy} {b.sz}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{b.sx} {b.sy} {b.sz}</size></box></geometry>
          <material><ambient>0.5 0.5 0.55 1</ambient><diffuse>0.5 0.5 0.55 1</diffuse></material></visual>
      </link>
    </model>''')
    parts += ['  </world>', '</sdf>']
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        f.write('\n'.join(parts))


def write_map(layout, path_yaml):
    occ = to_grid(layout.boxes)
    outside = _outside_mask()
    img = np.full((GRID_N, GRID_N), 254, dtype=np.uint8)
    img[occ] = 0
    img[outside & ~occ] = 205
    base = os.path.splitext(os.path.basename(path_yaml))[0]
    d = os.path.dirname(path_yaml) or '.'
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, base + '.pgm'), 'wb') as f:
        f.write(f"P5\n{GRID_N} {GRID_N}\n255\n".encode())
        f.write(img.tobytes())
    with open(path_yaml, 'w') as f:
        f.write(f"image: {base}.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{-EXTENT}, {-EXTENT}, 0.0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")