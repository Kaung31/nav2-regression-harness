#!/usr/bin/env python3
"""Self-checks for the two bits of Phase 5 logic that fail silently.

Both of these break in ways no run would surface: a clobbered scenario_id
just makes resume re-run work forever, and a misordered classifier still
writes a plausible-looking CSV. Runnable with no ROS on PATH:

    python3 src/harness_runner/test/test_phase5.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from harness_runner.batch_run import (            # noqa: E402
    BATCH_OWNED, FIELDNAMES, classify_failure)


def test_child_cannot_clobber_batch_owned_fields():
    """The repeat>0 resume bug: child echoes back the tag as its id."""
    row = {k: "" for k in FIELDNAMES}
    row.update({"scenario_id": "s00000", "repeat": 1, "inflation_radius": 0.55})
    res = {"scenario_id": "s00000_r1", "outcome": "SUCCESS", "path_length": 4.2,
           "inflation_radius": None}

    for k in FIELDNAMES:
        if k not in BATCH_OWNED and k in res and res[k] is not None:
            row[k] = res[k]

    assert row["scenario_id"] == "s00000", row["scenario_id"]
    assert row["inflation_radius"] == 0.55       # covariate survives the merge
    assert row["path_length"] == 4.2             # child data still lands
    assert row["outcome"] == "SUCCESS"


def _log(text):
    f = tempfile.NamedTemporaryFile("w", suffix=".log", delete=False)
    f.write(text)
    f.close()
    return f.name


PLANNED = "Passing new path to controller\n" * 3


def test_classification():
    planned = _log(PLANNED)
    silent = _log("[planner_server] no valid path\n")
    stalling = _log(PLANNED + "Failed to make progress\n" * 2)

    base = {"recoveries": 0, "true_path_length": 9.0, "optimal_path": 10.0,
            "path_length": 9.1}

    # SUCCESS and harness bugs are never given a robot failure mode.
    assert classify_failure("SUCCESS", base, planned)[0] == ""
    assert classify_failure("SETUP_FAILED:tf_timeout", base, planned)[0] == ""
    assert classify_failure("REJECTED", base, planned)[0] == "goal_rejected"

    # No path published outranks everything downstream of the planner.
    assert classify_failure("TIMEOUT", base, silent)[0] == "no_plan"

    # STUCK is ground-truth derived (wheels turn, robot does not move).
    stuck = dict(base, true_path_length=0.3, path_length=6.0)
    assert classify_failure("STUCK", stuck, planned)[0] == "collision"

    # Recoveries outrank distance: the BT actively noticed and fought back.
    fought = dict(base, recoveries=5, true_path_length=2.0)
    assert classify_failure("FAILED", fought, planned)[0] == "stuck_recovering"
    assert classify_failure("FAILED", base, stalling)[0] == "stuck_recovering"

    # s00010: 2 m travelled of an 11.5 m plan, no recoveries. This is the run
    # CLAUDE.md described as "went into the gap and wedged" -- it did not.
    s10 = {"recoveries": 0, "true_path_length": 1.993, "optimal_path": 11.549,
           "path_length": 1.993}
    assert classify_failure("TIMEOUT", s10, planned)[0] == "plan_not_executed"

    # Drove the full path and still failed: rule 11 has no name for this yet,
    # and inventing one by widening another class would be worse than saying so.
    assert classify_failure("FAILED", base, planned)[0] == "unclassified"

    for p in (planned, silent, stalling):
        os.unlink(p)


def test_every_batch_owned_field_is_a_real_column():
    assert BATCH_OWNED <= set(FIELDNAMES), BATCH_OWNED - set(FIELDNAMES)


if __name__ == "__main__":
    test_child_cannot_clobber_batch_owned_fields()
    test_classification()
    test_every_batch_owned_field_is_a_real_column()
    print("ok")
