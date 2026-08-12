#!/usr/bin/env python3
"""Emit N scenarios: worlds, maps, and an index JSON."""
import argparse
import json
import os

from harness_runner import scenario_gen as sg


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--count", type=int, default=50)
    p.add_argument("--start-seed", type=int, default=0)
    p.add_argument("--out", default="/ws/scenarios")
    p.add_argument("--gap-width", type=float, default=None,
                   help="fix the gap width instead of randomising")
    a = p.parse_args()

    os.makedirs(a.out, exist_ok=True)
    index, seed, skipped = [], a.start_seed, 0

    while len(index) < a.count:
        L = sg.generate(seed, gap_width=a.gap_width)
        seed += 1
        if L is None:
            skipped += 1
            continue
        sid = f"s{L.seed:05d}"
        world = os.path.join(a.out, f"{sid}.sdf")
        yaml_p = os.path.join(a.out, f"{sid}.yaml")
        sg.write_world(L, world)
        sg.write_map(L, yaml_p)
        straight = round((L.goal[0] ** 2 + L.goal[1] ** 2) ** 0.5, 3)
        index.append({
            "scenario_id": sid,
            "seed": L.seed,
            "world": world,
            "map": yaml_p,
            "goal_x": L.goal[0],
            "goal_y": L.goal[1],
            "gap_width": L.gap_width,
            "n_walls": L.n_walls,
            "n_obstacles": L.n_obstacles,
            "optimal_path": L.optimal_path,
            "straight_line": straight,
            "detour_ratio": round(L.optimal_path / straight, 3),
        })

    idx_path = os.path.join(a.out, "index.json")
    with open(idx_path, "w") as f:
        json.dump(index, f, indent=2)
    print(f"wrote {len(index)} scenarios to {a.out} "
          f"({skipped} seeds skipped as unreachable)")
    print(f"index: {idx_path}")


if __name__ == "__main__":
    main()