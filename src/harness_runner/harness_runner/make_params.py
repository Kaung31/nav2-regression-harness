#!/usr/bin/env python3
"""Write a nav2_params.yaml variant with one or more values overridden.

For Phase 7a/8.5 sweeps only. Design rule 10 forbids tuning a parameter to
make a failing scenario pass; varying one as a declared independent variable
across the whole scenario set is the exception, and it works only if each
value gets its own file, its own config_hash, and its own batch.

Overrides apply to EVERY occurrence of the key in the tree. That is the
correct semantic for the costmap parameters this exists to sweep --
`inflation_radius` and `robot_radius` appear once per costmap and must match,
and `probe_config` refuses to start a batch where they disagree.
"""
import argparse
import os
import sys


def set_everywhere(node, key, value):
    """Set key wherever it appears. Returns the number of sites changed."""
    n = 0
    if isinstance(node, dict):
        for k, v in node.items():
            if k == key and not isinstance(v, (dict, list)):
                node[k] = value
                n += 1
            else:
                n += set_everywhere(v, key, value)
    elif isinstance(node, list):
        for v in node:
            n += set_everywhere(v, key, value)
    return n


def coerce(s):
    for cast in (int, float):
        try:
            return cast(s)
        except ValueError:
            pass
    if s.lower() in ("true", "false"):
        return s.lower() == "true"
    return s


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--base", default=None,
                   help="defaults to the INSTALLED nav2_params.yaml")
    p.add_argument("--set", dest="sets", action="append", required=True,
                   metavar="KEY=VALUE", help="repeatable")
    p.add_argument("--out", required=True)
    a = p.parse_args()

    import yaml
    base = a.base
    if base is None:
        from ament_index_python.packages import get_package_share_directory
        base = os.path.join(get_package_share_directory("harness_description"),
                            "config", "nav2_params.yaml")
    with open(base) as f:
        cfg = yaml.safe_load(f)

    for pair in a.sets:
        if "=" not in pair:
            sys.exit(f"--set expects KEY=VALUE, got {pair!r}")
        key, raw = pair.split("=", 1)
        hits = set_everywhere(cfg, key, coerce(raw))
        if hits == 0:
            # Silently writing a file that does not contain the override is
            # the worst outcome: the sweep runs, every cell is identical, and
            # nothing says so until the results are meaningless.
            sys.exit(f"{key!r} does not appear anywhere in {base} -- refusing "
                     f"to write a variant that would not change anything.")
        print(f"  {key} = {coerce(raw)!r}  ({hits} site{'s' if hits > 1 else ''})")

    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    with open(a.out, "w") as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
