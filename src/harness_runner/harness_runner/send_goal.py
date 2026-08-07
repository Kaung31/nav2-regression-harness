#!/usr/bin/env python3
"""Send a single navigation goal and report the outcome."""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--x", type=float, required=True)
    p.add_argument("--y", type=float, required=True)
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument("--timeout", type=float, default=120.0)
    p.add_argument("--startup-timeout", type=float, default=60.0)
    args = p.parse_args()

    rclpy.init()
    nav = BasicNavigator()

    if not nav.nav_to_pose_client.wait_for_server(timeout_sec=args.startup_timeout):
        print("RESULT: NAV2_NOT_READY")
        nav.destroy_node()
        rclpy.shutdown()
        return

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x = args.x
    goal.pose.position.y = args.y
    qx, qy, qz, qw = yaw_to_quat(args.yaw)
    goal.pose.orientation.x = qx
    goal.pose.orientation.y = qy
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw

    start = time.time()
    timed_out = False
    nav.goToPose(goal)

    while not nav.isTaskComplete():
        if time.time() - start > args.timeout:
            nav.cancelTask()
            timed_out = True
            break
        time.sleep(0.5)

    elapsed = time.time() - start
    result = nav.getResult()

    if timed_out:
        print(f"RESULT: TIMEOUT  time={elapsed:.1f}s")
    elif result == TaskResult.SUCCEEDED:
        print(f"RESULT: SUCCESS  time={elapsed:.1f}s")
    elif result == TaskResult.CANCELED:
        print(f"RESULT: CANCELED time={elapsed:.1f}s")
    else:
        print(f"RESULT: FAILED   time={elapsed:.1f}s")

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()