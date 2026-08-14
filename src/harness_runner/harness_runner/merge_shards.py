#!/usr/bin/env python3
"""Join sharded batch CSVs into one, refusing to merge different experiments.

The concatenation is trivial; the refusal is the point. Design rule 9 exists
because two batches run either side of a tweak silently merge into one
meaningless dataset, and parallel workers make that easier, not harder -- a
shard that picked up a stale image or an un-rebuilt workspace produces rows
that look identical to every other row except for one column nobody reads.
This makes that column impossible to ignore.
"""
import argparse
import csv
import glob
import os
import sys


def merge(paths, out_path, allow_mixed=False):
    rows, header = [], None
    per_file = {}
    for p in sorted(paths):
        with open(p, newline="") as f:
            r = csv.DictReader(f)
            if r.fieldnames is None:
                continue
            if header is None:
                header = r.fieldnames
            elif r.fieldnames != header:
                raise SystemExit(
                    f"{p} has a different column set to the first shard.\n"
                    f"  extra   : {set(r.fieldnames) - set(header)}\n"
                    f"  missing : {set(header) - set(r.fieldnames)}\n"
                    f"Shards were produced by different code. Do not merge.")
            got = list(r)
        per_file[p] = len(got)
        rows += got

    if not rows:
        raise SystemExit("no rows found in " + ", ".join(paths))

    hashes = {r.get("config_hash", "") for r in rows}
    shas = {r.get("git_sha", "") for r in rows}
    if len(hashes) > 1 and not allow_mixed:
        raise SystemExit(
            f"shards carry {len(hashes)} different config_hash values: "
            f"{sorted(hashes)}\nThat means they did not run the same "
            f"experiment -- most likely one worker used a stale image or "
            f"skipped colcon build. Merging them would produce a dataset no "
            f"finding can be conditioned on. Re-run the odd shard, or pass "
            f"--allow-mixed if you genuinely intend a multi-config file.")

    # Duplicates mean two shards ran the same task -- a sharding bug, and one
    # that would silently double-weight those rows in every rate calculation.
    keys = [(r["scenario_id"], r["repeat"]) for r in rows]
    dupes = {k for k in keys if keys.count(k) > 1}
    if dupes:
        raise SystemExit(
            f"{len(dupes)} (scenario_id, repeat) keys appear in more than one "
            f"shard, e.g. {sorted(dupes)[:5]}. The shards overlap; rate "
            f"calculations would double-count them.")

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        w.writerows(rows)

    for p, n in per_file.items():
        print(f"  {n:4d}  {p}")
    print(f"{len(rows)} rows -> {out_path}")
    print(f"config_hash {sorted(hashes)}  git_sha {sorted(shas)}")
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("inputs", nargs="+", help="CSV paths or globs")
    p.add_argument("--out", required=True)
    p.add_argument("--allow-mixed", action="store_true",
                   help="permit differing config_hash values. Only for a "
                        "deliberate multi-config file, never to work around "
                        "a shard that came out wrong.")
    a = p.parse_args()

    paths = []
    for pattern in a.inputs:
        paths += glob.glob(pattern) if any(c in pattern for c in "*?[") else [pattern]
    missing = [q for q in paths if not os.path.exists(q)]
    if missing or not paths:
        print(f"no such file(s): {missing or a.inputs}", file=sys.stderr)
        sys.exit(2)

    merge(paths, a.out, a.allow_mixed)


if __name__ == "__main__":
    main()
