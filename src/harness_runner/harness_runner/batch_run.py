#!/usr/bin/env python3
"""Phase 5: loop over a scenario index, run each one, write a single CSV.

Design notes -- each of these is a bug that bit, not a preference.

SUBPROCESS, NOT FUNCTION CALL
    Every scenario runs as its own `ros2 run harness_runner run_scenario`
    child. run_scenario's try/except only catches Python exceptions inside
    its own process; it cannot catch a segfault in rclpy/DDS/Gazebo's C++,
    and its internal timeouts rely on its own event loop still being
    responsive. A separate process bounds both: a crash kills one child,
    and a wedge is caught by an outer clock that does not care what is
    broken inside.

SIGINT BEFORE SIGKILL
    subprocess.run(timeout=) sends SIGKILL. On this harness that is
    actively harmful: run_scenario tears down its launch tree from
    StackProcess.__exit__, and SIGKILL means __exit__ never runs, so the
    whole `ros2 launch` tree -- which lives in its own session -- is
    orphaned by definition. SIGINT instead becomes a KeyboardInterrupt,
    unwinds the `with`, and lets run_scenario killpg its own tree. Only
    escalate if that fails. (Measured: SIGKILL -> teardown never runs,
    tree survives. SIGINT -> teardown runs, tree dies.)

LEAK DETECTION IS SET-DIFFERENCE, NOT A COUNT
    Counting all matching processes on the machine is wrong twice over: it
    counts the user's own manually-launched Gazebo/Foxglove session as a
    leak, and it counts unreaped ZOMBIES as leaks. The container has no
    init/reaper (docker-compose.yml has no `init: true`), so every SIGKILL
    escalation leaves permanent zombies -- a count-based check would see
    them forever and abort the batch after three runs, blaming teardown.
    Instead: snapshot live PIDs before each run, and a leak is a PID that
    appeared during the run and is still alive and not a zombie.

KILL ONLY WHAT THIS RUN CREATED
    Phase 3 forbids a machine-wide `pkill -f "gz sim"` and it is right to.
    Cleanup here targets the process groups of PIDs that were absent from
    the pre-run snapshot -- so a stack the user launched by hand in another
    terminal is never touched, and this stays correct when Phase 6 runs
    scenarios in parallel.

ROTATING ROS_DOMAIN_ID
    Consecutive runs on one domain can discover each other. A dying
    participant from run N is still announced for longer than any sane
    inter-run pause, so run N+1's wait_for_stack can latch onto the
    previous run's action server, declare itself ready, and fire a goal
    into a stack that is shutting down. Rotating the domain per run makes
    that structurally impossible. run_scenario already derives GZ_PARTITION
    from domain_id, so this isolates Gazebo transport too.

RESUME KEY IS (scenario_id, repeat)
    Repeats are not optional for this project's headline claim -- a
    threshold measured from one run per scenario cannot be distinguished
    from luck. The `repeat` column exists from row one so that adding
    --repeats later does not invalidate every CSV written before it.
"""
import argparse
import csv
import datetime
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import time

FIELDNAMES = [
    "scenario_id", "repeat", "seed",
    "gap_width", "gap_width_continuous", "n_walls", "n_obstacles",
    "detour_ratio", "goal_x", "goal_y", "goal_yaw",
    "outcome", "failure_class", "failure_evidence",
    "sim_duration", "wall_duration", "wall_time", "realtime_factor",
    "path_length", "true_path_length", "slip_ratio",
    "straight_line", "path_efficiency",
    "optimal_path", "planning_efficiency",
    "min_lidar_range", "min_clearance", "clearance_saturated",
    "max_speed", "recoveries",
    "final_error", "true_final_error",
    "odom_samples", "scan_samples",
    # Covariates. Constant within a batch by construction, but stored per row
    # so a merged CSV is still analysable and a merge of two configs is still
    # detectable. "A finding you cannot condition on is a finding you cannot
    # defend" -- and retrofitting these after a 300-run batch means re-running it.
    "robot_version", "lidar_hz", "beam_count", "inflation_radius",
    "robot_radius_param", "planner_plugin", "controller_plugin",
    "map_resolution", "config_hash", "git_sha",
    "started_at", "domain_id", "batch_attempt",
    "leaked_procs", "leaked_detail", "error_detail",
]

# Fields the BATCH is authoritative for. The child is told a per-repeat tag
# (`s00000_r1`) as its --scenario-id and echoes that back in its result JSON;
# copying it over the row would make the resume key unmatchable, so every
# repeat>0 would re-run forever and duplicate on each resume. Guarding the
# whole set rather than just scenario_id means the next field added to both
# sides cannot reintroduce the same class of bug.
BATCH_OWNED = frozenset({
    "scenario_id", "repeat", "seed", "gap_width", "gap_width_continuous",
    "n_walls", "n_obstacles", "detour_ratio",
    "failure_class", "failure_evidence", "realtime_factor",
    "robot_version", "lidar_hz", "beam_count", "inflation_radius",
    "robot_radius_param", "planner_plugin", "controller_plugin",
    "map_resolution", "config_hash", "git_sha",
    "started_at", "domain_id", "batch_attempt",
    "leaked_procs", "leaked_detail",
})

# Every node harness.launch.py starts. The Phase 2 one-liner only watched
# `gz sim` and `ros2 launch`; a run that leaks controller_server but not
# bt_navigator would have read as clean.
NAV_NODE_NAMES = ["controller_server", "smoother_server", "planner_server",
                  "behavior_server", "velocity_smoother", "bt_navigator",
                  "waypoint_follower", "map_server", "amcl",
                  "lifecycle_manager", "robot_state_publisher",
                  "parameter_bridge", "foxglove_bridge"]
STRAGGLER_PATTERN = re.compile(
    r"\bgz sim\b|\bros2 launch\b|" + "|".join(NAV_NODE_NAMES), re.I)

# Outcomes meaning "the harness or environment broke", not "the robot failed
# to navigate". Only these are ever retried. Retrying a genuine navigation
# failure would quietly delete the finding this project exists to produce.
HARNESS_SIDE = ("SETUP_FAILED", "HARNESS_ERROR", "BATCH_TIMEOUT",
                "BATCH_LAUNCH_FAILED")

EXIT_OK = 0
EXIT_HARNESS = 1        # batch aborted / environment broken
EXIT_CONFIG = 2         # bad arguments, unusable CSV, ros2 not on PATH


class ConfigError(Exception):
    pass


# --------------------------------------------------------------------------
# covariates and config fingerprint  (design rule 9)
# --------------------------------------------------------------------------

def _dig(d, *keys):
    for k in keys:
        if not isinstance(d, dict) or k not in d:
            raise ConfigError(f"nav2_params.yaml has no {'.'.join(keys)}")
        d = d[k]
    return d


def _exactly_one(pattern, text, what, path):
    """Covariates must be unambiguous or not recorded at all."""
    m = re.findall(pattern, text)
    if len(m) != 1:
        raise ConfigError(
            f"expected exactly one <{what}> in {path}, found {len(m)}. "
            f"The URDF has grown a second sensor, so this can no longer "
            f"guess which one is the lidar -- name it explicitly here.")
    return m[0]


def _git_sha():
    """Short SHA, suffixed -dirty when the tree has uncommitted changes.

    Provenance only. `config_hash` is what actually guards design rule 10 --
    git is too easy to lose (separate mounts, safe.directory, a CI checkout
    with no history, no git binary at all) to be the thing a dataset's
    integrity rests on.

    GIT_DISCOVERY_ACROSS_FILESYSTEM is required here: colcon installs into a
    separate mount from the bind-mounted source, and git refuses to walk up
    past a filesystem boundary by default, so discovery from the installed
    package fails without it. build/ install/ log/ are gitignored, so the
    dirty check does not trip over colcon's own output.

    The dirty flag is the point: a SHA that does not describe the code that
    actually ran is worse than no SHA, because it looks trustworthy.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    env = dict(os.environ, GIT_DISCOVERY_ACROSS_FILESYSTEM="1")
    try:
        rev = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=here,
                             env=env, capture_output=True, text=True, timeout=10)
        if rev.returncode != 0:
            return "no-git"
        st = subprocess.run(["git", "status", "--porcelain"], cwd=here,
                            env=env, capture_output=True, text=True, timeout=30)
        dirty = "-dirty" if st.stdout.strip() else ""
        return rev.stdout.strip() + dirty
    except Exception:
        return "no-git"


def probe_config(params_file, urdf_file):
    """Read every covariate ONCE per batch. Raises rather than degrading.

    Silently writing blank covariates would be the expensive failure: the
    batch completes, looks fine, and is unanalysable months later. Refusing
    to start costs seconds.

    These read the INSTALLED share/ copies -- the same files
    harness.launch.py hands to Nav2 -- not src/. If someone edits src without
    `colcon build`, this fingerprints what actually ran.
    """
    for f in (params_file, urdf_file):
        if not os.path.exists(f):
            raise ConfigError(
                f"cannot read {f} for covariates. Did `colcon build` install "
                f"it? CMakeLists must install every data dir, and a missing "
                f"one fails at runtime, not at build.")
    try:
        import yaml
    except ImportError:
        raise ConfigError("pyyaml missing -- source /ws/install/setup.bash")

    with open(params_file) as f:
        P = yaml.safe_load(f)
    with open(urdf_file) as f:
        urdf = f.read()

    gc = _dig(P, "global_costmap", "global_costmap", "ros__parameters")
    lc = _dig(P, "local_costmap", "local_costmap", "ros__parameters")
    infl_g = _dig(gc, "inflation_layer", "inflation_radius")
    infl_l = _dig(lc, "inflation_layer", "inflation_radius")
    if infl_g != infl_l:
        raise ConfigError(
            f"global inflation_radius ({infl_g}) != local ({infl_l}). One "
            f"column cannot describe both, and during the Phase 7a sweep this "
            f"is a config slip, not a design. Set them equal or split the "
            f"column deliberately.")

    # Resolve plugins through the *_plugins list so renaming an instance in
    # the YAML cannot silently change what gets recorded.
    cs = _dig(P, "controller_server", "ros__parameters")
    ps = _dig(P, "planner_server", "ros__parameters")

    return {
        "robot_version": "v1",
        "lidar_hz": float(_exactly_one(
            r"<update_rate>([\d.]+)</update_rate>", urdf,
            "update_rate", urdf_file)),
        "beam_count": int(_exactly_one(
            r"<samples>(\d+)</samples>", urdf, "samples", urdf_file)),
        "inflation_radius": infl_g,
        "robot_radius_param": gc.get("robot_radius"),
        "map_resolution": gc.get("resolution"),
        "planner_plugin": _dig(ps, ps["planner_plugins"][0], "plugin"),
        "controller_plugin": _dig(cs, cs["controller_plugins"][0], "plugin"),
        "config_hash": _config_hash(params_file, urdf_file),
        "git_sha": _git_sha(),
    }


def _config_hash(params_file, urdf_file):
    """Fingerprint everything that determines what a recorded number MEANS.

    Not just the tuned config -- the harness source too. Changing
    run_scenario.py's clearance computation or scenario_gen.py's BFS alters
    the meaning of a column without touching one YAML value, and that has
    already happened once in this project: the min_clearance frame fix makes
    every earlier value incomparable while nav2_params.yaml is untouched.
    A fingerprint that misses that is not doing its job.

    git_sha cannot be relied on to cover it -- colcon installs into a separate
    mount, so git cannot even discover the repo from the installed package --
    so the hash carries it instead. Hashing the INSTALLED .py files means this
    fingerprints what actually ran, matching the share/ choice above: edit
    src without `colcon build` and the hash correctly does not move, because
    the edit did not run.

    Deliberately over-sensitive: a comment-only edit moves the hash and blocks
    a merge that would have been harmless. That is the cheap direction to be
    wrong in -- the expensive failure is merging two batches that ran
    different code.
    """
    h = hashlib.sha256()
    pkg = os.path.dirname(os.path.abspath(__file__))
    sources = sorted(f for f in os.listdir(pkg) if f.endswith(".py"))
    for p in [params_file, urdf_file] + [os.path.join(pkg, f) for f in sources]:
        with open(p, "rb") as f:
            h.update(f.read())
    return h.hexdigest()[:12]


# --------------------------------------------------------------------------
# failure classification  (design rule 11)
# --------------------------------------------------------------------------

# Below this fraction of the BFS optimal path, the robot did not meaningfully
# execute the plan it was given. Fixed here rather than chosen after seeing
# the results -- picking it later is how a classifier ends up fitted to the
# hypothesis it is supposed to test.
EXECUTED_FRACTION = 0.5
STUCK_RECOVERY_COUNT = 3


def classify_failure(outcome, row, log_path):
    """-> (failure_class, evidence). TIMEOUT is a symptom, not a mode.

    Deliberately does NOT emit `collision` from min_clearance: that metric is
    derived from the robot's own lidar, which saturates at range_min, so a
    negative value is a lower bound and not proof of contact. The one
    collision signal here is ground-truth-derived (rule 1): wheels turning
    while the true pose barely moves means the robot is pressed against
    geometry on a flat, single-robot world.
    """
    if outcome == "SUCCESS":
        return "", ""
    if outcome.startswith(HARNESS_SIDE):
        return "", ""                    # harness bug, not a robot failure
    if outcome == "REJECTED":
        return "goal_rejected", "action server rejected the goal"

    try:
        with open(log_path, errors="replace") as f:
            log = f.read()
    except OSError:
        return "unclassified", f"launch log unreadable: {log_path}"

    plans = log.count("Passing new path")
    no_progress = log.count("Failed to make progress")
    recoveries = row.get("recoveries") or 0
    travelled = row.get("true_path_length")
    optimal = row.get("optimal_path")

    if plans == 0:
        return "no_plan", "no 'Passing new path' in launch log"
    if outcome == "STUCK":
        return "collision", (f"ground truth {travelled} m vs wheel odom "
                             f"{row.get('path_length')} m")
    if no_progress or recoveries >= STUCK_RECOVERY_COUNT:
        return "stuck_recovering", (f"{no_progress}x 'Failed to make "
                                    f"progress', {recoveries} recoveries")
    if travelled is not None and optimal:
        if travelled < EXECUTED_FRACTION * optimal:
            return "plan_not_executed", (f"{plans} paths published, moved "
                                         f"{travelled} m of {optimal} m")
    # Reaching here is a signal, not a bug: the run failed in a way rule 11
    # does not yet name. If a batch produces many, add a class -- do not widen
    # an existing one to absorb them.
    return "unclassified", (f"{plans} paths, {recoveries} recoveries, "
                            f"moved {travelled} m of {optimal} m")


# --------------------------------------------------------------------------
# process inspection
# --------------------------------------------------------------------------

def live_harness_pids():
    """{pid: cmdline} of LIVE harness processes. Zombies excluded.

    Returns None if ps itself failed -- 'unknown' must stay distinct from
    'none', or a broken ps silently reads as a clean machine.
    """
    try:
        r = subprocess.run(["ps", "-eo", "pid=,stat=,args="],
                           capture_output=True, text=True, timeout=15)
    except Exception:
        return None
    if r.returncode != 0:
        return None
    me = os.getpid()
    procs = {}
    for line in r.stdout.splitlines():
        parts = line.strip().split(None, 2)
        if len(parts) < 3:
            continue
        pid_s, stat, cmd = parts
        try:
            pid = int(pid_s)
        except ValueError:
            continue
        if pid == me or pid == 1:
            continue
        if stat.startswith("Z"):        # zombie: already dead, just unreaped
            continue
        if STRAGGLER_PATTERN.search(cmd):
            procs[pid] = cmd
    return procs


def zombie_count():
    try:
        r = subprocess.run(["ps", "-eo", "stat="], capture_output=True,
                           text=True, timeout=15)
        return sum(1 for s in r.stdout.split() if s.startswith("Z"))
    except Exception:
        return -1


def _pgid(pid):
    try:
        return os.getpgid(pid)
    except (ProcessLookupError, PermissionError):
        return None


def leaked_since(pre):
    """PIDs alive now that were not alive before the run. None if unknown."""
    cur = live_harness_pids()
    if cur is None or pre is None:
        return None
    return {pid: cmd for pid, cmd in cur.items() if pid not in pre}


def wait_for_clean(pre, max_wait, poll=1.0):
    """Poll until nothing new is left, or max_wait elapses."""
    deadline = time.time() + max_wait
    leaked = leaked_since(pre)
    while leaked and time.time() < deadline:
        time.sleep(poll)
        leaked = leaked_since(pre)
    return leaked


def kill_leaked(leaked):
    """Kill process groups of PIDs that appeared during this run only.

    Never a machine-wide pkill: anything in the pre-run snapshot -- the
    user's own terminal session, a sibling run under Phase 6 parallelism --
    is by construction not in `leaked` and is never signalled.
    """
    if not leaked:
        return
    mypg = os.getpgrp()
    groups, loose = set(), []
    for pid in leaked:
        pg = _pgid(pid)
        if pg is not None and pg != mypg:
            groups.add(pg)
        else:
            loose.append(pid)
    for sig in (signal.SIGINT, signal.SIGKILL):
        for pg in groups:
            try:
                os.killpg(pg, sig)
            except (ProcessLookupError, PermissionError):
                pass
        for pid in loose:
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(3)


def graceful_kill(proc, grace_int=45.0, grace_term=15.0):
    """SIGINT -> SIGTERM -> SIGKILL on the child's own process group.

    SIGINT first is the entire point: run_scenario turns it into a
    KeyboardInterrupt that unwinds `with StackProcess(...)`, whose __exit__
    killpg's the launch tree it owns. SIGKILL cannot do that.
    """
    pg = _pgid(proc.pid)
    for sig, wait in ((signal.SIGINT, grace_int),
                      (signal.SIGTERM, grace_term),
                      (signal.SIGKILL, 10.0)):
        if proc.poll() is not None:
            return
        try:
            if pg is not None and pg != os.getpgrp():
                os.killpg(pg, sig)
            else:
                proc.send_signal(sig)
        except (ProcessLookupError, PermissionError):
            return
        try:
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue


# --------------------------------------------------------------------------
# CSV / resume
# --------------------------------------------------------------------------

def load_completed(csv_path):
    """{(scenario_id, repeat)} already recorded. Validates the header."""
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        return set()
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            return set()
        if header != FIELDNAMES:
            missing = [c for c in FIELDNAMES if c not in header]
            extra = [c for c in header if c not in FIELDNAMES]
            raise ConfigError(
                f"{csv_path} was written with a different column set, so "
                f"appending to it would silently misalign every new row.\n"
                f"  missing from the file : {missing}\n"
                f"  no longer produced    : {extra}\n"
                f"Move it aside (mv {csv_path} {csv_path}.old) and start a "
                f"fresh CSV, or point --out somewhere new.")
        i_sid = header.index("scenario_id")
        i_rep = header.index("repeat")
        done = set()
        for row in reader:
            # A row with the wrong width is a torn final write from a hard
            # kill. Skipping it means that scenario is simply re-run.
            if len(row) != len(header):
                continue
            if row[i_sid]:
                done.add((row[i_sid], row[i_rep]))
    return done


def build_cmd(entry, sid, out_json, domain_id, args):
    cmd = ["ros2", "run", "harness_runner", "run_scenario",
           "--world", entry["world"], "--map", entry["map"],
           "--x", str(entry["goal_x"]), "--y", str(entry["goal_y"]),
           "--scenario-id", sid,
           "--domain-id", str(domain_id),
           "--goal-timeout", str(args.goal_timeout),
           "--startup-timeout", str(args.startup_timeout),
           "--out", out_json]
    if entry.get("optimal_path") is not None:
        cmd += ["--optimal-path", str(entry["optimal_path"])]
    return cmd


# --------------------------------------------------------------------------
# one run
# --------------------------------------------------------------------------

def run_one(entry, repeat, domain_id, args, cov, attempt=1):
    """Run one scenario as a subprocess. Always returns a row dict."""
    sid = entry["scenario_id"]
    tag = sid if repeat == 0 else f"{sid}_r{repeat}"
    out_json = os.path.join(args.results_dir, f"{tag}.json")

    # NOT <log-dir>/<sid>.log -- run_scenario hardcodes its LAUNCH log to
    # /tmp/<scenario-id>.log and opens it with mode 'w'. Two writers on one
    # path truncate each other and interleave at independent offsets, so the
    # launch log you need when a run fails is exactly the one that gets
    # corrupted. This file is only the child's own stdout.
    log_path = os.path.join(args.log_dir, f"{tag}.batch.log")

    if os.path.exists(out_json):
        try:
            os.remove(out_json)     # never read a stale result
        except OSError:
            pass

    row = {k: "" for k in FIELDNAMES}
    row.update(cov)
    row.update({
        "scenario_id": sid, "repeat": repeat, "seed": entry.get("seed"),
        "gap_width": entry.get("gap_width"), "n_walls": entry.get("n_walls"),
        # scenario_gen draws gap_width from rng.uniform and never rasterises
        # it, so the index value IS the continuous one. Named explicitly
        # because Phase 7 needs the un-quantised independent variable and a
        # column called `gap_width` alone does not say which it is.
        "gap_width_continuous": entry.get("gap_width"),
        "n_obstacles": entry.get("n_obstacles"),
        "detour_ratio": entry.get("detour_ratio"),
        "goal_x": entry.get("goal_x"), "goal_y": entry.get("goal_y"),
        "optimal_path": entry.get("optimal_path"),
        "domain_id": domain_id, "batch_attempt": attempt,
        "started_at": datetime.datetime.now().isoformat(timespec="seconds"),
    })

    cmd = build_cmd(entry, tag, out_json, domain_id, args)
    hard_timeout = args.startup_timeout + args.goal_timeout + args.teardown_grace

    pre = live_harness_pids()
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)

    interrupted = False
    try:
        with open(log_path, "w") as lf:
            try:
                proc = subprocess.Popen(cmd, stdout=lf,
                                        stderr=subprocess.STDOUT,
                                        env=env, start_new_session=True)
            except FileNotFoundError:
                # Overwhelmingly the most common cause: forgot to
                # `source /ws/install/setup.bash`. Fail with a sentence, not
                # a traceback that kills the batch on run 1.
                row["outcome"] = "BATCH_LAUNCH_FAILED"
                row["error_detail"] = ("`ros2` not found on PATH -- run "
                                       "`source /ws/install/setup.bash` first")
                row["leaked_procs"] = 0
                return row
            try:
                proc.wait(timeout=hard_timeout)
            except subprocess.TimeoutExpired:
                graceful_kill(proc)
                row["outcome"] = "BATCH_TIMEOUT"
                row["error_detail"] = (
                    f"child exceeded {hard_timeout:.0f}s hard timeout")
            except KeyboardInterrupt:
                # start_new_session means the child does NOT get the
                # terminal's Ctrl+C, so forward it explicitly -- otherwise
                # the launch tree survives the batch that started it.
                graceful_kill(proc)
                interrupted = True
                raise
    finally:
        if interrupted:
            leaked = wait_for_clean(pre, args.straggler_wait)
            kill_leaked(leaked)

    leaked = wait_for_clean(pre, args.straggler_wait)
    if leaked:
        kill_leaked(leaked)
        leaked = leaked_since(pre)

    timed_out = row["outcome"] == "BATCH_TIMEOUT"
    if os.path.exists(out_json):
        # Read it even after a BATCH_TIMEOUT. run_scenario writes --out only
        # after its own teardown has finished, so a result file existing
        # alongside a hang means navigation genuinely completed and something
        # wedged on the way out. Throwing that measurement away and recording
        # BATCH_TIMEOUT instead would be discarding real data.
        try:
            with open(out_json) as f:
                res = json.load(f)
            for k in FIELDNAMES:
                if k not in BATCH_OWNED and k in res and res[k] is not None:
                    row[k] = res[k]
            if not row["outcome"]:
                row["outcome"] = "HARNESS_ERROR:NoOutcomeField"
            if timed_out:
                row["error_detail"] = (
                    "result recorded, but the child had to be killed after "
                    "the hard timeout -- teardown hung")
        except (json.JSONDecodeError, OSError) as exc:
            row["outcome"] = "HARNESS_ERROR:BadResultFile"
            row["error_detail"] = str(exc)[:200]
    elif not timed_out:
        rc = proc.returncode
        sig_note = f"signal {-rc}" if rc is not None and rc < 0 else f"exit {rc}"
        row["outcome"] = f"HARNESS_ERROR:NoResultFile({sig_note})"
        row["error_detail"] = f"see {log_path}"

    if leaked is None:
        row["leaked_procs"] = -1            # ps failed: unknown, not zero
        row["leaked_detail"] = "ps unavailable"
    else:
        row["leaked_procs"] = len(leaked)
        row["leaked_detail"] = "; ".join(
            f"{p}:{c[:60]}" for p, c in list(leaked.items())[:3])

    # Recorded, never applied here. Phase 9 excludes low-RTF runs at a
    # threshold fixed in advance; dropping them now -- after seeing which ones
    # failed -- would be selection bias towards the hypothesis, since an
    # overloaded run is more likely to fail.
    sim, wall = row.get("sim_duration"), row.get("wall_duration")
    if isinstance(sim, (int, float)) and isinstance(wall, (int, float)) and wall > 0:
        row["realtime_factor"] = round(sim / wall, 3)

    # run_scenario writes its launch log to /tmp/<scenario-id>.log, and it was
    # given the per-repeat tag as its id.
    row["failure_class"], row["failure_evidence"] = classify_failure(
        row["outcome"], row, f"/tmp/{tag}.log")
    return row


# --------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description="Batch-run every scenario in an index and write one CSV.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--index", default="/ws/scenarios/index.json")
    p.add_argument("--out", default="/ws/results/batch_results.csv")
    p.add_argument("--results-dir", default="/ws/results")
    p.add_argument("--log-dir", default="/tmp")
    p.add_argument("--repeats", type=int, default=1,
                   help="runs per scenario. >1 lets you report a success "
                        "RATE instead of a single coin flip -- required "
                        "before any threshold claim from Phase 7.")
    p.add_argument("--domain-id", type=int, default=42,
                   help="base ROS_DOMAIN_ID")
    p.add_argument("--domain-span", type=int, default=8,
                   help="rotate over this many domain ids so consecutive "
                        "runs cannot discover each other's dying DDS "
                        "participants. 1 disables rotation.")
    p.add_argument("--goal-timeout", type=float, default=90.0)
    p.add_argument("--startup-timeout", type=float, default=180.0)
    p.add_argument("--teardown-grace", type=float, default=150.0,
                   help="added to startup+goal to form the child's hard "
                        "timeout; must exceed run_scenario's own worst-case "
                        "teardown (~30s) plus process startup")
    p.add_argument("--cooldown", type=float, default=3.0,
                   help="pause between runs to let transports settle")
    p.add_argument("--straggler-wait", type=float, default=20.0)
    p.add_argument("--max-consecutive-leaks", type=int, default=3)
    p.add_argument("--max-harness-failure-rate", type=float, default=0.2,
                   help="exit non-zero if more than this fraction of runs "
                        "failed harness-side after retries. Navigation "
                        "failures never count towards this.")
    p.add_argument("--retries", type=int, default=1,
                   help="total attempts for a harness-side failure. "
                        "SUCCESS/FAILED/TIMEOUT/STUCK/REJECTED are data and "
                        "are never retried at any setting.")
    p.add_argument("--limit", type=int, default=None)
    p.add_argument("--start-seed", type=int, default=None)
    p.add_argument("--dry-run", action="store_true")
    a = p.parse_args()

    if a.repeats < 1 or a.domain_span < 1:
        print("--repeats and --domain-span must be >= 1", file=sys.stderr)
        sys.exit(EXIT_CONFIG)

    # Before anything launches: if the covariates cannot be read, every row
    # this batch writes would be unanalysable. Fail in seconds, not in hours.
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("harness_description")
        cov = probe_config(os.path.join(share, "config", "nav2_params.yaml"),
                           os.path.join(share, "urdf", "robot.urdf.xacro"))
    except ConfigError as exc:
        print(f"\ncannot read run covariates: {exc}\n", file=sys.stderr)
        sys.exit(EXIT_CONFIG)
    except Exception as exc:
        print(f"\ncannot locate harness_description: {exc}\n"
              f"source /ws/install/setup.bash first.\n", file=sys.stderr)
        sys.exit(EXIT_CONFIG)

    try:
        with open(a.index) as f:
            scenarios = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"cannot read --index {a.index}: {exc}", file=sys.stderr)
        sys.exit(EXIT_CONFIG)

    if a.start_seed is not None:
        scenarios = [s for s in scenarios if s.get("seed", 0) >= a.start_seed]

    os.makedirs(a.results_dir, exist_ok=True)
    os.makedirs(a.log_dir, exist_ok=True)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)

    try:
        done = load_completed(a.out)
    except ConfigError as exc:
        print(f"\n{exc}\n", file=sys.stderr)
        sys.exit(EXIT_CONFIG)

    # Repeat-major: every scenario once, then every scenario again. A crash
    # partway through leaves one full pass rather than complete data for the
    # first few scenarios and nothing for the rest.
    tasks = [(s, r) for r in range(a.repeats) for s in scenarios
             if (s["scenario_id"], str(r)) not in done]
    if a.limit:
        tasks = tasks[:a.limit]

    total_slots = len(scenarios) * a.repeats
    print(f"{len(scenarios)} scenarios x {a.repeats} repeats = {total_slots} "
          f"runs; {len(done)} already in {a.out}; {len(tasks)} to run now")

    print(f"config {cov['config_hash']} @ {cov['git_sha']}: "
          f"inflation_radius={cov['inflation_radius']} "
          f"lidar={cov['beam_count']}@{cov['lidar_hz']}Hz "
          f"res={cov['map_resolution']} "
          f"{cov['controller_plugin'].split('::')[-1]}/"
          f"{cov['planner_plugin'].split('::')[-1]}")
    if cov["git_sha"] == "no-git":
        print("note: git provenance unavailable, so rows carry config_hash "
              "but no SHA. config_hash still fingerprints the params, the "
              "URDF and the harness source, so batches remain separable -- "
              "you just cannot map one back to a commit.")
    elif cov["git_sha"].endswith("-dirty"):
        print("note: working tree is dirty, so git_sha does not fully "
              "describe the code that ran. Commit before a batch you intend "
              "to publish numbers from.")

    if a.dry_run:
        if tasks:
            e, r = tasks[0]
            tag = e["scenario_id"] if r == 0 else f"{e['scenario_id']}_r{r}"
            print("first command:", " ".join(build_cmd(
                e, tag, os.path.join(a.results_dir, f"{tag}.json"),
                a.domain_id, a)))
            print(f"hard timeout per run: "
                  f"{a.startup_timeout + a.goal_timeout + a.teardown_grace:.0f}s")
        sys.exit(EXIT_OK)

    pre_existing = live_harness_pids()
    if pre_existing:
        print(f"WARNING: {len(pre_existing)} harness process(es) are already "
              f"running before the batch starts. They will be left alone, but "
              f"they will compete for CPU and skew every timing number:")
        for pid, cmd in list(pre_existing.items())[:5]:
            print(f"    {pid}  {cmd[:90]}")
    zc = zombie_count()
    if zc > 0:
        print(f"note: {zc} zombie process(es) present. Harmless here (they "
              f"are excluded from leak detection) but they mean the container "
              f"has no init reaper -- add `init: true` to docker-compose.yml.")

    write_header = not os.path.exists(a.out) or os.path.getsize(a.out) == 0
    f = open(a.out, "a", newline="")
    writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
    if write_header:
        writer.writeheader()
        f.flush()

    batch_start = time.time()
    consecutive_leaks = 0
    aborted = False
    outcomes = {}
    completed = 0

    try:
        for i, (entry, repeat) in enumerate(tasks, 1):
            sid = entry["scenario_id"]
            domain = a.domain_id + ((i - 1) % a.domain_span)
            if domain > 101:                 # ROS_DOMAIN_ID safe ceiling
                domain = a.domain_id + ((i - 1) % max(1, 101 - a.domain_id))

            attempt = 1
            row = run_one(entry, repeat, domain, a, cov, attempt=attempt)
            while row["outcome"].startswith(HARNESS_SIDE) and attempt < a.retries:
                attempt += 1
                print(f"  [{sid} r{repeat}] {row['outcome']} -> retry "
                      f"{attempt}/{a.retries}")
                time.sleep(a.cooldown)
                row = run_one(entry, repeat, domain, a, cov, attempt=attempt)

            writer.writerow(row)
            f.flush()
            os.fsync(f.fileno())        # flush() only reaches the OS buffer

            completed += 1
            outcomes[row["outcome"]] = outcomes.get(row["outcome"], 0) + 1
            elapsed = time.time() - batch_start
            eta = (elapsed / i) * (len(tasks) - i)
            print(f"[{i}/{len(tasks)}] {sid} r{repeat} d{domain}: "
                  f"{row['outcome']}  ({elapsed / 60:.1f}m elapsed, "
                  f"{eta / 60:.1f}m eta, leaked={row['leaked_procs']})")

            if isinstance(row["leaked_procs"], int) and row["leaked_procs"] > 0:
                consecutive_leaks += 1
                if consecutive_leaks >= a.max_consecutive_leaks:
                    aborted = True
                    print(f"\nABORTING: {consecutive_leaks} runs in a row left "
                          f"live processes behind that cleanup could not kill. "
                          f"That is a broken environment, not bad scenarios -- "
                          f"every later run would be competing for CPU and its "
                          f"timings would be worthless. Last log: "
                          f"{a.log_dir}/{sid}.batch.log and /tmp/{sid}.log. "
                          f"Everything completed so far is already in {a.out}.")
                    break
            else:
                consecutive_leaks = 0

            if i < len(tasks):
                time.sleep(a.cooldown)
    except KeyboardInterrupt:
        print(f"\nInterrupted. Completed runs are already saved to {a.out}; "
              f"re-run the same command to resume.")
    finally:
        f.close()

    harness_fail = sum(n for o, n in outcomes.items()
                       if o.startswith(HARNESS_SIDE))
    print("\nSummary:")
    for o, n in sorted(outcomes.items(), key=lambda kv: -kv[1]):
        print(f"  {n:4d}  {o}")
    print(f"Results: {a.out}")

    if aborted:
        sys.exit(EXIT_HARNESS)
    if completed and harness_fail / completed > a.max_harness_failure_rate:
        print(f"\n{harness_fail}/{completed} runs failed harness-side "
              f"(> {a.max_harness_failure_rate:.0%}). Exiting non-zero: that "
              f"is the harness breaking, not the robot failing.")
        sys.exit(EXIT_HARNESS)
    sys.exit(EXIT_OK)


if __name__ == "__main__":
    main()
