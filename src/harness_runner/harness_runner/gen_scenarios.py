#!/usr/bin/env python3
"""Emit N scenarios: worlds, maps, and an index JSON."""
import argparse
import json
import os

from harness_runner import scenario_gen as sg


def emit(L, sid, out_dir):
    """Write world + map from one box list (design rule 7) and index it."""
    world = os.path.join(out_dir, f"{sid}.sdf")
    yaml_p = os.path.join(out_dir, f"{sid}.yaml")
    sg.write_world(L, world)
    sg.write_map(L, yaml_p)
    straight = round((L.goal[0] ** 2 + L.goal[1] ** 2) ** 0.5, 3)
    return {
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
            # Insurance, not a feature. A true collision check needs the
            # geometry against the ground-truth pose -- the lidar cannot give
            # it, because it saturates at range_min. Emitting the boxes now
            # costs three lines; discovering they are needed after a 300-run
            # batch means regenerating the worlds and re-running the batch.
            "boxes": [[b.cx, b.cy, b.sx, b.sy, b.sz] for b in L.boxes],
    }


def frange(lo, hi, step):
    """Inclusive of hi, rounded -- 0.05 steps must not drift into 1.4999999."""
    n = int(round((hi - lo) / step))
    return [round(lo + i * step, 4) for i in range(n + 1)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["random", "doorway"], default="random",
                   help="random = seeded rooms, the DISCOVERY mechanism. "
                        "doorway = canonical single aperture, the MEASUREMENT "
                        "instrument. They do different jobs; keep both.")
    p.add_argument("--count", type=int, default=50, help="random mode only")
    p.add_argument("--start-seed", type=int, default=0)
    p.add_argument("--out", default="/ws/scenarios")
    p.add_argument("--gap-width", type=float, default=None,
                   help="random mode: fix the gap width instead of randomising")
    p.add_argument("--width-min", type=float, default=0.5)
    p.add_argument("--width-max", type=float, default=1.5)
    p.add_argument("--width-step", type=float, default=0.05)
    p.add_argument("--index-name", default=None,
                   help="defaults to index.json (random) / doorway_index.json")
    a = p.parse_args()

    os.makedirs(a.out, exist_ok=True)
    index, skipped = [], 0

    if a.mode == "doorway":
        for w in frange(a.width_min, a.width_max, a.width_step):
            L = sg.generate_doorway(w)
            if L is None:
                # Below 2 x ROBOT_RADIUS the footprint does not fit. Ground
                # truth refusing is a result, not a generator failure -- but
                # there is nothing to run, so it is not indexed.
                skipped += 1
                print(f"  skip w={w:.2f}: footprint does not fit (ground truth)")
                continue
            index.append(emit(L, f"d{int(round(w * 100)):03d}", a.out))
        default_name = "doorway_index.json"
        # The whole point of the instrument: one number, unchanged throughout.
        lengths = {e["optimal_path"] for e in index}
        print(f"optimal_path across all widths: {sorted(lengths)}")
        if len(lengths) > 1:
            print("WARNING: path length is NOT constant across widths, so gap "
                  "width is confounded with it -- exactly the flaw the random "
                  "rooms have. Do not sweep until this is one value.")
    else:
        seed = a.start_seed
        while len(index) < a.count:
            L = sg.generate(seed, gap_width=a.gap_width)
            seed += 1
            if L is None:
                skipped += 1
                continue
            index.append(emit(L, f"s{L.seed:05d}", a.out))
        default_name = "index.json"

    idx_path = os.path.join(a.out, a.index_name or default_name)
    with open(idx_path, "w") as f:
        json.dump(index, f, indent=2)
    print(f"wrote {len(index)} scenarios to {a.out} ({skipped} skipped)")
    print(f"index: {idx_path}")


if __name__ == "__main__":
    main()