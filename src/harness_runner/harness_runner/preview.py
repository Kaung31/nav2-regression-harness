#!/usr/bin/env python3
"""ASCII preview of a generated scenario."""
import argparse
from harness_runner import scenario_gen as sg


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seed", type=int, required=True)
    p.add_argument("--gap-width", type=float, default=None)
    p.add_argument("--downsample", type=int, default=6)
    a = p.parse_args()

    L = sg.generate(a.seed, gap_width=a.gap_width)
    if L is None:
        print(f"seed {a.seed}: no valid goal found")
        return

    occ = sg.to_grid(L.boxes) | sg._outside_mask()
    ds = a.downsample
    small = occ[::ds, ::ds]
    r0, c0 = [v // ds for v in sg.w2g(*L.start)]
    r1, c1 = [v // ds for v in sg.w2g(*L.goal)]

    straight = (L.goal[0] ** 2 + L.goal[1] ** 2) ** 0.5
    print(f"seed {L.seed}  gap={L.gap_width}m  walls={L.n_walls}  "
          f"obstacles={L.n_obstacles}")
    print(f"goal {L.goal}  straight={straight:.2f}m  "
          f"optimal={L.optimal_path}m  detour={L.optimal_path / straight:.2f}x")
    for r in range(small.shape[0]):
        line = ""
        for c in range(small.shape[1]):
            if (r, c) == (r0, c0):
                line += "S"
            elif (r, c) == (r1, c1):
                line += "G"
            else:
                line += "#" if small[r, c] else "."
        print(line)


if __name__ == "__main__":
    main()