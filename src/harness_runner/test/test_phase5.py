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
# Real s00000 shape: the planner published paths AND failed repeatedly.
PLANNER_FAILING = (PLANNED * 11 + "GridBased plugin failed to create plan\n" * 18
                   + "[compute_path_to_pose] [ActionServer] Aborting handle.\n" * 20)


def test_classification():
    planned = _log(PLANNED)
    silent = _log("[planner_server] nothing here\n")
    failing = _log(PLANNER_FAILING)

    base = {"recoveries": 0, "true_path_length": 9.0, "optimal_path": 10.0,
            "path_length": 9.1}

    # SUCCESS and harness bugs are never given a robot failure mode.
    assert classify_failure("SUCCESS", base, planned)[0] == ""
    assert classify_failure("SETUP_FAILED:tf_timeout", base, planned)[0] == ""
    assert classify_failure("REJECTED", base, planned)[0] == "goal_rejected"

    # A planner that published nothing at all.
    assert classify_failure("TIMEOUT", base, silent)[0] == "no_plan"

    # s00000, the regression this ordering exists for: 33 paths published but
    # 18 plan failures and 5 recoveries. Presence of paths must NOT hide the
    # planner failing, and the recoveries are downstream of it.
    s0 = dict(base, recoveries=5, true_path_length=4.852, optimal_path=10.044)
    cls, ev = classify_failure("FAILED", s0, failing)
    assert cls == "no_plan", cls
    assert "18 plan failures" in ev, ev

    # STUCK is ground-truth derived and outranks even planner failure: being
    # wedged is why the planner cannot plan, not the other way round.
    stuck = dict(base, true_path_length=0.3, path_length=6.0)
    assert classify_failure("STUCK", stuck, failing)[0] == "collision"

    # Recoveries only classify once the planner is exonerated.
    fought = dict(base, recoveries=5, true_path_length=2.0)
    assert classify_failure("FAILED", fought, planned)[0] == "stuck_recovering"

    # s00010: 2 m travelled of an 11.5 m plan, no recoveries, planner healthy.
    # This is the run CLAUDE.md described as "went into the gap and wedged".
    s10 = {"recoveries": 0, "true_path_length": 1.993, "optimal_path": 11.549,
           "path_length": 1.993}
    assert classify_failure("TIMEOUT", s10, planned)[0] == "plan_not_executed"

    # control_3: the controller aborted follow_path 16 times and the BT fired
    # 4 recoveries in response. Named by the cause, not the symptom -- and the
    # planner aborts in s00000 must NOT be counted as controller aborts.
    ctrl = _log(PLANNED + "[controller_server]: [follow_path] [ActionServer] "
                          "Aborting handle.\n" * 16)
    c3 = dict(base, recoveries=4, amcl_final_error=1.9,
              amcl_final_yaw_error=2.482, goal_tolerance_xy=0.25,
              goal_tolerance_yaw=0.25)
    cls, ev = classify_failure("FAILED", c3, ctrl)
    assert cls == "plan_not_executed", cls
    assert "16 controller aborts" in ev and "0 planner aborts" in ev, ev
    # s00000's planner aborts are counted on the other side of the pipeline.
    assert "20 planner aborts" in classify_failure("FAILED", s0, failing)[1]
    os.unlink(ctrl)

    # Drove the route, then failed the goal check on POSITION.
    tol = {"goal_tolerance_xy": 0.25, "goal_tolerance_yaw": 0.25}
    missed = dict(base, amcl_final_error=0.31, amcl_final_yaw_error=0.1, **tol)
    assert classify_failure("FAILED", missed, planned)[0] == "goal_tolerance_miss"

    # Ended 4 m from the goal: never arrived, so not a "tolerance miss".
    far = dict(base, amcl_final_error=4.23, amcl_final_yaw_error=2.834, **tol)
    assert classify_failure("FAILED", far, planned)[0] == "unclassified"

    # ...and on YAW alone, parked exactly on the goal facing the wrong way.
    # Without goal_tolerance_yaw recorded this was indistinguishable from
    # a position miss.
    spun = dict(base, amcl_final_error=0.02, amcl_final_yaw_error=1.4, **tol)
    assert classify_failure("FAILED", spun, planned)[0] == "goal_tolerance_miss"

    # Inside both tolerances but Nav2 still refused -> genuinely unnamed.
    inside = dict(base, amcl_final_error=0.11, amcl_final_yaw_error=0.05, **tol)
    assert classify_failure("FAILED", inside, planned)[0] == "unclassified"

    # A zero error must not read as "field missing" -- 0.0 is falsy.
    exact = dict(base, amcl_final_error=0.0, amcl_final_yaw_error=0.0, **tol)
    assert classify_failure("FAILED", exact, planned)[0] == "unclassified"

    # CSV round-trip: the same row arrives as strings, and must classify the same.
    as_str = {k: str(v) for k, v in missed.items()}
    assert classify_failure("FAILED", as_str, planned)[0] == "goal_tolerance_miss"

    # No pose data at all (e.g. tf lookup failed) -> never guess.
    assert classify_failure("FAILED", base, planned)[0] == "unclassified"

    for p in (planned, silent, failing):
        os.unlink(p)


def test_every_batch_owned_field_is_a_real_column():
    assert BATCH_OWNED <= set(FIELDNAMES), BATCH_OWNED - set(FIELDNAMES)


if __name__ == "__main__":
    test_child_cannot_clobber_batch_owned_fields()
    test_classification()
    test_every_batch_owned_field_is_a_real_column()
    print("ok")
