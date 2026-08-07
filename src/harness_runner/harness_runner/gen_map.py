#!/usr/bin/env python3
"""Generate an occupancy grid matching empty_room.sdf."""
import os
import argparse
import numpy as np


def make_map(out_dir, name="empty_room", res=0.05,
             half=5.0, wall_t=0.2, margin=0.2):
    extent = half + wall_t / 2 + margin
    n = int(round(2 * extent / res))
    img = np.full((n, n), 254, dtype=np.uint8)      # free

    inner = half - wall_t / 2                        # 4.9
    outer = half + wall_t / 2                        # 5.1

    for r in range(n):
        y = extent - r * res
        for c in range(n):
            x = -extent + c * res
            on_wall = (inner <= abs(x) <= outer and abs(y) <= outer) or \
                      (inner <= abs(y) <= outer and abs(x) <= outer)
            if on_wall:
                img[r, c] = 0                        # occupied
            elif abs(x) > outer or abs(y) > outer:
                img[r, c] = 205                      # unknown

    os.makedirs(out_dir, exist_ok=True)

    with open(os.path.join(out_dir, f"{name}.pgm"), "wb") as f:
        f.write(f"P5\n{n} {n}\n255\n".encode())
        f.write(img.tobytes())

    with open(os.path.join(out_dir, f"{name}.yaml"), "w") as f:
        f.write(
            f"image: {name}.pgm\n"
            f"mode: trinary\n"
            f"resolution: {res}\n"
            f"origin: [{-extent}, {-extent}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n"
        )
    return n, extent


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="maps")
    args = p.parse_args()
    n, extent = make_map(args.out)
    print(f"wrote {n}x{n} px map, extent +/-{extent} m -> {args.out}/")


if __name__ == "__main__":
    main()