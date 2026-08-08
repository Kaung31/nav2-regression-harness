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


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


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
        valid = [r for r in msg.ranges
                 if msg.range_min < r < msg.range_max and math.isfinite(r)]
        if valid:
            self.min_lidar_range = min(self.min_lidar_range, min(valid))

    def _cmd_cb(self, msg):
        self.max_speed = max(self.max_speed, abs(msg.linear.x))

    def _bt_cb(self, msg):
        for ev in msg.event_log:
            if ev.node_name in self.RECOVERY_NODES and \
                    ev.current_status == "RUNNING" and \
                    ev.previous_status != "RUNNING":
                self.recoveries += 1

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
        "max_speed": None, "recoveries": None, "final_error": None,
        "odom_samples": 0, "scan_samples": 0, "wall_time": None,
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
            result.update({
                "outcome": outcome,
                "sim_duration": round(sim_dur, 2) if sim_dur is not None else None,
                "wall_duration": round(wall, 2),
                "path_length": round(node.path_length, 3),
                "straight_line": round(straight, 3),
                "path_efficiency": (round(actual_straight / node.path_length, 3)
                                    if node.path_length > 0.01 else None),
                "min_lidar_range": round(mlr, 3) if mlr is not None else None,
                "min_clearance": (round(mlr - ROBOT_RADIUS, 3)
                                  if mlr is not None else None),
                "max_speed": round(node.max_speed, 3),
                "recoveries": node.recoveries,
                "odom_samples": node.odom_samples,
                "scan_samples": node.scan_samples,
            })
            if node.final_xy:
                result["final_error"] = round(
                    math.hypot(cfg["goal_x"] - node.final_xy[0],
                               cfg["goal_y"] - node.final_xy[1]), 3)
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
    p.add_argument("--out", default=None)
    a = p.parse_args()

    sid = a.scenario_id or f"run_{uuid.uuid4().hex[:8]}"
    cfg = {"scenario_id": sid, "world": a.world, "map": a.map,
           "goal_x": a.x, "goal_y": a.y, "goal_yaw": a.yaw,
           "domain_id": a.domain_id, "goal_timeout": a.goal_timeout,
           "startup_timeout": a.startup_timeout,
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