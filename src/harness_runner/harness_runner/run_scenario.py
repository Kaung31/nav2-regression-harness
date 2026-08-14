#!/usr/bin/env python3
"""Run one navigation scenario end to end and return a result dict."""
import os
os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

import argparse
import json
import math
import signal
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog
from lifecycle_msgs.srv import GetState

ROBOT_RADIUS = 0.22

# The lidar is NOT at the robot's centre. robot.urdf.xacro mounts lidar_link at
# xyz="0.10 0 0.075" on base_link, so a raw range is measured from a point 10 cm
# ahead of the footprint centre. `min_lidar_range - ROBOT_RADIUS` therefore
# carries a bearing-dependent error of up to LIDAR_X: forward beams read ~0.1 m
# too close, rear beams ~0.1 m too generous. Keep this in step with the URDF.
LIDAR_X = 0.10

# A hit closer than the sensor's range_min is not reported as "very close", it
# is dropped or clamped. Any clearance derived from such a scan is a LOWER
# BOUND, not a measurement, so the run records that it happened rather than
# quietly reporting the floor as if it were data. 0.01 is <resolution> in the
# URDF's <range> block.
RANGE_SATURATION_EPS = 0.01


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_error(a, b):
    """Smallest absolute angle between two headings, wrap-safe."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class ScenarioRun(Node):
    RECOVERY_NODES = {"Spin", "BackUp", "Wait", "DriveOnHeading",
                      "ClearGlobalCostmap-Context", "ClearLocalCostmap-Context"}

    def __init__(self):
        super().__init__(
            "scenario_run",
            parameter_overrides=[rclpy.parameter.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        self.path_length = 0.0
        self.prev_xy = None
        self.min_lidar_range = float("inf")
        self.min_centre_range = float("inf")
        self.clearance_saturated = False
        self.true_path_length = 0.0
        self.true_prev_xy = None
        self.true_final_xy = None
        self.true_final_yaw = None
        self.max_speed = 0.0
        self.recoveries = 0
        self.odom_samples = 0
        self.scan_samples = 0
        self.final_xy = None
        self.sim_time = None
        self.state_client = self.create_client(
            GetState, "/bt_navigator/get_state")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Clock, "/clock", self._clock_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, sensor_qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(BehaviorTreeLog, "/behavior_tree_log",
                                 self._bt_cb, 10)
        self.create_subscription(Odometry, "/ground_truth", self._truth_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def _clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _odom_cb(self, msg):
        self.odom_samples += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.final_xy = (x, y)
        if self.prev_xy is not None:
            step = math.hypot(x - self.prev_xy[0], y - self.prev_xy[1])
            if step < 1.0:                      # reject teleport artefacts
                self.path_length += step
        self.prev_xy = (x, y)

    def _scan_cb(self, msg):
        self.scan_samples += 1
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            # Check saturation BEFORE the validity filter: a reading clamped to
            # exactly range_min is the case that matters most and the filter
            # below would silently discard it.
            if r <= msg.range_min + RANGE_SATURATION_EPS:
                self.clearance_saturated = True
            if not (msg.range_min < r < msg.range_max):
                continue
            if r < self.min_lidar_range:
                self.min_lidar_range = r
            # Re-reference the hit to base_link before comparing it against a
            # base_link-centred radius. Mixing the two frames is what made the
            # old min_clearance wrong by up to LIDAR_X.
            a = msg.angle_min + i * msg.angle_increment
            d = math.hypot(LIDAR_X + r * math.cos(a), r * math.sin(a))
            if d < self.min_centre_range:
                self.min_centre_range = d

    def _cmd_cb(self, msg):
        self.max_speed = max(self.max_speed, abs(msg.linear.x))

    def _bt_cb(self, msg):
        for ev in msg.event_log:
            if ev.node_name in self.RECOVERY_NODES and \
                    ev.current_status == "RUNNING" and \
                    ev.previous_status != "RUNNING":
                self.recoveries += 1

    def _truth_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.true_final_xy = (x, y)
        self.true_final_yaw = quat_to_yaw(msg.pose.pose.orientation)
        if self.true_prev_xy is not None:
            step = math.hypot(x - self.true_prev_xy[0], y - self.true_prev_xy[1])
            if step < 1.0:
                self.true_path_length += step
        self.true_prev_xy = (x, y)

    def map_pose(self):
        """(x, y, yaw) in the map frame, or None.

        This is the AMCL-corrected estimate that `SimpleGoalChecker` actually
        evaluates. `/odom` drifts and `/ground_truth` is the simulator's
        secret, so neither explains a goal-checker verdict: recording those
        two alone left every near-miss failure `unclassified`, because the
        number Nav2 judged on was not being measured.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
        except Exception:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, quat_to_yaw(t.transform.rotation)

    def wait_for_active(self, timeout):
        """bt_navigator must be ACTIVE, not merely present."""
        deadline = time.time() + timeout
        if not self.state_client.wait_for_service(timeout_sec=timeout):
            return False, "state_service_timeout"
        while time.time() < deadline:
            fut = self.state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            res = fut.result()
            if res is not None and res.current_state.id == 3:   # ACTIVE
                return True, "active"
            time.sleep(1.0)
        return False, "not_active"
    
    def wait_for_stack(self, timeout):
        """Action server available AND map->odom transform present."""
        half = timeout / 2.0
        t0 = time.time()
        if not self.nav_client.wait_for_server(timeout_sec=half):
            return False, "action_server_timeout"
        deadline = t0 + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time()):
                return self.wait_for_active(max(10.0, deadline - time.time()))
        return False, "tf_timeout"

    def navigate(self, x, y, yaw, timeout):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        sim_start = self.sim_time
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return "REJECTED", 0.0, None

        start = time.time()
        result_future = handle.get_result_async()
        outcome = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if result_future.done():
                break
            if time.time() - start > timeout:
                handle.cancel_goal_async()
                for _ in range(25):
                    rclpy.spin_once(self, timeout_sec=0.2)
                outcome = "TIMEOUT"
                break
            if (self.path_length > 1.0 and
                    self.true_path_length < 0.2 * self.path_length):
                handle.cancel_goal_async()
                for _ in range(25):
                    rclpy.spin_once(self, timeout_sec=0.2)
                outcome = "STUCK"
                break

        wall = time.time() - start
        sim_dur = None
        if sim_start is not None and self.sim_time is not None:
            sim_dur = self.sim_time - sim_start

        if outcome == "TIMEOUT":
            return "TIMEOUT", wall, sim_dur
        res = result_future.result()
        status = res.status if res is not None else 0
        ok = status == GoalStatus.STATUS_SUCCEEDED
        return ("SUCCESS" if ok else "FAILED"), wall, sim_dur


class StackProcess:
    """Owns the launch subprocess and guarantees scoped teardown."""

    def __init__(self, world, map_yaml, domain_id, log_path):
        self.world = world
        self.map_yaml = map_yaml
        self.domain_id = domain_id
        self.log_path = log_path
        self.proc = None
        self.log_fh = None

    def __enter__(self):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.domain_id)
        env["GZ_PARTITION"] = f"harness_{self.domain_id}"
        env["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
        self.log_fh = open(self.log_path, "w")
        cmd = ["ros2", "launch", "harness_description", "harness.launch.py",
               f"world:={self.world}", f"map:={self.map_yaml}",
               "use_foxglove:=false"]
        self.proc = subprocess.Popen(
            cmd, stdout=self.log_fh, stderr=subprocess.STDOUT,
            env=env, start_new_session=True)
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False

    def stop(self):
        if self.proc is None:
            return
        try:
            pgid = os.getpgid(self.proc.pid)
        except ProcessLookupError:
            pgid = None
        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGINT)
                self.proc.wait(timeout=20)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(pgid, signal.SIGKILL)
                    self.proc.wait(timeout=10)
                except Exception:
                    pass
        if self.log_fh:
            self.log_fh.close()


def exit_code_for(outcome):
    """Navigation failure is data; harness failure is an error."""
    if outcome.startswith("HARNESS_ERROR") or outcome.startswith("SETUP_FAILED"):
        return 1
    return 0


def run_scenario(cfg):
    """Launch, navigate, measure, tear down. Always returns a dict."""
    result = {
        "scenario_id": cfg["scenario_id"],
        "goal_x": cfg["goal_x"], "goal_y": cfg["goal_y"],
        "goal_yaw": cfg.get("goal_yaw", 0.0),
        "outcome": "SETUP_FAILED",
        "sim_duration": None, "wall_duration": None,
        "path_length": None, "straight_line": None, "path_efficiency": None,
        "min_lidar_range": None, "min_clearance": None,
        "clearance_saturated": False,
        "max_speed": None, "recoveries": None, "final_error": None,
        "odom_samples": 0, "scan_samples": 0, "wall_time": None,
        "true_path_length": None, "true_final_error": None, "slip_ratio": None,
        "true_final_yaw_error": None, "amcl_final_error": None,
        "amcl_final_yaw_error": None, "localisation_error": None,
    }
    wall_start = time.time()
    domain = cfg.get("domain_id", 42)
    log_path = cfg.get("log_path", f"/tmp/{cfg['scenario_id']}.log")

    node = None
    try:
        with StackProcess(cfg["world"], cfg["map"], domain, log_path):
            os.environ["ROS_DOMAIN_ID"] = str(domain)
            rclpy.init()
            node = ScenarioRun()

            ok, why = node.wait_for_stack(cfg.get("startup_timeout", 180.0))
            if not ok:
                result["outcome"] = f"SETUP_FAILED:{why}"
                return result

            for _ in range(15):
                rclpy.spin_once(node, timeout_sec=0.2)

            start_xy = node.final_xy or (0.0, 0.0)
            outcome, wall, sim_dur = node.navigate(
                cfg["goal_x"], cfg["goal_y"], cfg.get("goal_yaw", 0.0),
                cfg.get("goal_timeout", 180.0))

            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)

            straight = math.hypot(cfg["goal_x"] - start_xy[0],
                                  cfg["goal_y"] - start_xy[1])
            end_xy = node.final_xy or start_xy
            actual_straight = math.hypot(end_xy[0] - start_xy[0],
                                         end_xy[1] - start_xy[1])
            mlr = (node.min_lidar_range
                   if math.isfinite(node.min_lidar_range) else None)
            mcr = (node.min_centre_range
                   if math.isfinite(node.min_centre_range) else None)
            result.update({
                "outcome": outcome,
                "sim_duration": round(sim_dur, 2) if sim_dur is not None else None,
                "wall_duration": round(wall, 2),
                "path_length": round(node.path_length, 3),
                "straight_line": round(straight, 3),
                "path_efficiency": (round(actual_straight / node.path_length, 3)
                                    if node.path_length > 0.01 else None),
                "optimal_path": cfg.get("optimal_path"),
                "planning_efficiency": (
                    round(cfg["optimal_path"] / node.path_length, 3)
                    if (outcome == "SUCCESS" and cfg.get("optimal_path")
                        and node.path_length > 0.01)
                    else None),
                "min_lidar_range": round(mlr, 3) if mlr is not None else None,
                "min_clearance": (round(mcr - ROBOT_RADIUS, 3)
                                  if mcr is not None else None),
                # True = the lidar hit its floor, so min_clearance is an upper
                # bound on how bad it got, not a measurement. A negative
                # min_clearance with this set is NOT evidence of a specific
                # overlap depth.
                "clearance_saturated": node.clearance_saturated,
                "max_speed": round(node.max_speed, 3),
                "true_path_length": round(node.true_path_length, 3),
                "slip_ratio": (round(node.true_path_length / node.path_length, 3)
                               if node.path_length > 0.01 else None),
                "recoveries": node.recoveries,
                "odom_samples": node.odom_samples,
                "scan_samples": node.scan_samples,
            })
            if node.final_xy:
                result["final_error"] = round(
                    math.hypot(cfg["goal_x"] - node.final_xy[0],
                               cfg["goal_y"] - node.final_xy[1]), 3)
            if node.true_final_xy:
                result["true_final_error"] = round(
                    math.hypot(cfg["goal_x"] - node.true_final_xy[0],
                               cfg["goal_y"] - node.true_final_xy[1]), 3)
            if node.true_final_yaw is not None:
                result["true_final_yaw_error"] = round(
                    angle_error(node.true_final_yaw,
                                cfg.get("goal_yaw", 0.0)), 3)

            mp = node.map_pose()
            if mp is not None:
                mx, my, myaw = mp
                result["amcl_final_error"] = round(
                    math.hypot(cfg["goal_x"] - mx, cfg["goal_y"] - my), 3)
                result["amcl_final_yaw_error"] = round(
                    angle_error(myaw, cfg.get("goal_yaw", 0.0)), 3)
                if node.true_final_xy:
                    # How far the robot's belief is from where it actually is.
                    # Rule 1 applied to localisation: a large value here means
                    # any goal-checker verdict is about AMCL, not about
                    # navigation.
                    result["localisation_error"] = round(
                        math.hypot(mx - node.true_final_xy[0],
                                   my - node.true_final_xy[1]), 3)
    except Exception as exc:
        result["outcome"] = f"HARNESS_ERROR:{type(exc).__name__}"
        result["error_detail"] = str(exc)[:200]
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        result["wall_time"] = round(time.time() - wall_start, 1)
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--world", required=True)
    p.add_argument("--map", required=True)
    p.add_argument("--x", type=float, required=True)
    p.add_argument("--y", type=float, required=True)
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument("--scenario-id", default=None)
    p.add_argument("--domain-id", type=int, default=42)
    p.add_argument("--goal-timeout", type=float, default=180.0)
    p.add_argument("--startup-timeout", type=float, default=180.0)
    p.add_argument("--optimal-path", type=float, default=None)
    p.add_argument("--out", default=None)
    a = p.parse_args()

    sid = a.scenario_id or f"run_{uuid.uuid4().hex[:8]}"
    cfg = {"scenario_id": sid, "world": a.world, "map": a.map,
           "goal_x": a.x, "goal_y": a.y, "goal_yaw": a.yaw,
           "domain_id": a.domain_id, "goal_timeout": a.goal_timeout,
           "startup_timeout": a.startup_timeout,
           "optimal_path": a.optimal_path,
           "log_path": f"/tmp/{sid}.log"}
    res = run_scenario(cfg)
    print(json.dumps(res, indent=2))
    if a.out:
        os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
        with open(a.out, "w") as f:
            json.dump(res, f, indent=2)
    sys.exit(exit_code_for(res["outcome"]))


if __name__ == "__main__":
    main()