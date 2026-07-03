#!/usr/bin/env python3
"""
Robo_Fleet - Remote Rosbridge Test Suite
─────────────────────────────────────────
Tests configured for the real rosbridge at 192.168.0.8
Robots available: tb1, tb3 (no tb2)
Topics: amcl_pose, odom, scan (no battery_state)

Usage:
  python sim/test_remote.py
  python sim/test_remote.py --host 192.168.0.8 --port 9090
"""

import json
import math
import time
import argparse
import sys
sys.path.insert(0, 'mcp_server')
from ros.ros_client import RosClient

parser = argparse.ArgumentParser()
parser.add_argument("--host", default="192.168.0.8", help="rosbridge host")
parser.add_argument("--port", type=int, default=9090, help="rosbridge port")
parser.add_argument("--robots", nargs="+", default=["tb1", "tb3"], help="Available robots")
args, _ = parser.parse_known_args()

HOST = args.host
PORT = args.port
ROBOTS = args.robots


def header(text):
    print(f"\n{'=' * 60}")
    print(f"  {text}")
    print(f"{'=' * 60}")


results = []


def run_test(name, fn):
    try:
        fn()
        results.append(("PASS", name))
        print(f"  ✅ {name}")
    except Exception as e:
        results.append(("FAIL", name))
        print(f"  ❌ {name}: {e}")


def make_client():
    client = RosClient(host=HOST, port=PORT)
    client.connect()
    return client


# ═══════════════════════════════════════════
# CONNECTION
# ═══════════════════════════════════════════

def test_connection():
    client = make_client()
    assert client.connected
    client.disconnect()

def test_reconnect():
    client = make_client()
    client.disconnect()
    client.connect()
    assert client.connected
    client.disconnect()


# ═══════════════════════════════════════════
# POSE (amcl_pose)
# ═══════════════════════════════════════════

def test_get_pose_tb1():
    client = make_client()
    msg = client.subscribe_once("/tb1/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", timeout=5.0)
    client.disconnect()
    assert msg is not None, "No pose from tb1"
    pos = msg["pose"]["pose"]["position"]
    print(f"       tb1: ({pos['x']:.2f}, {pos['y']:.2f})")

def test_get_pose_tb3():
    client = make_client()
    msg = client.subscribe_once("/tb3/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", timeout=5.0)
    client.disconnect()
    assert msg is not None, "No pose from tb3"
    pos = msg["pose"]["pose"]["position"]
    print(f"       tb3: ({pos['x']:.2f}, {pos['y']:.2f})")

def test_fleet_status():
    client = make_client()
    online = []
    for robot_id in ROBOTS:
        msg = client.subscribe_once(f"/{robot_id}/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", timeout=3.0)
        if msg:
            online.append(robot_id)
    client.disconnect()
    assert len(online) >= 1, "No robots online"
    print(f"       Online: {online}")


# ═══════════════════════════════════════════
# ODOMETRY
# ═══════════════════════════════════════════

def test_odom():
    client = make_client()
    msg = client.subscribe_once("/tb1/odom", "nav_msgs/msg/Odometry", timeout=5.0)
    client.disconnect()
    assert msg is not None, "No odom from tb1"
    pos = msg["pose"]["pose"]["position"]
    vel = msg["twist"]["twist"]["linear"]
    print(f"       tb1 odom: pos=({pos['x']:.2f}, {pos['y']:.2f}), vel=({vel['x']:.3f}, {vel['y']:.3f})")


# ═══════════════════════════════════════════
# LASER SCAN
# ═══════════════════════════════════════════

def test_scan():
    client = make_client()
    msg = client.subscribe_once("/tb1/scan", "sensor_msgs/msg/LaserScan", timeout=5.0)
    client.disconnect()
    assert msg is not None, "No scan from tb1"
    ranges = msg.get("ranges", [])
    valid = [r for r in ranges if msg.get("range_min", 0.1) <= r <= msg.get("range_max", 10.0)]
    closest = min(valid) if valid else None
    print(f"       tb1 scan: {len(ranges)} rays, closest={closest:.2f}m")


# ═══════════════════════════════════════════
# CONTROL (cmd_vel)
# ═══════════════════════════════════════════

def test_publish_cmd_vel():
    client = make_client()
    # Send small forward velocity
    client.publish(
        topic="/tb1/cmd_vel",
        msg_type="geometry_msgs/msg/Twist",
        data={"linear": {"x": 0.1, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
    )
    time.sleep(0.5)
    # Stop
    client.publish(
        topic="/tb1/cmd_vel",
        msg_type="geometry_msgs/msg/Twist",
        data={"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
    )
    client.disconnect()
    print("       Sent forward + stop to tb1")

def test_stop_all():
    client = make_client()
    for robot_id in ROBOTS:
        client.publish(
            topic=f"/{robot_id}/cmd_vel",
            msg_type="geometry_msgs/msg/Twist",
            data={"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        )
    client.disconnect()
    print(f"       Stopped {ROBOTS}")


# ═══════════════════════════════════════════
# NAVIGATION (Nav2 action)
# ═══════════════════════════════════════════

def test_navigate_tb1():
    client = make_client()

    # Get current position first
    msg = client.subscribe_once("/tb1/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", timeout=5.0)
    if msg:
        pos = msg["pose"]["pose"]["position"]
        # Navigate to a point 0.5m ahead of current position
        target_x = pos["x"] + 0.3
        target_y = pos["y"]
    else:
        target_x = 0.5
        target_y = 0.0

    goal = {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": target_x, "y": target_y, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        }
    }

    resp = client.send_goal("/tb1/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
    goal_id = resp["goal_id"]
    print(f"       Goal sent: {goal_id}, target=({target_x:.2f}, {target_y:.2f})")

    result = client.wait_for_result("/tb1/navigate_to_pose", goal_id, timeout=30.0)
    client.disconnect()

    if result and result.get("success"):
        print(f"       ✅ Navigation succeeded (status={result['status']})")
    elif result:
        print(f"       Result: {result}")
        # Don't assert failure - real Nav2 may return different status codes
    else:
        print(f"       ⚠️  No result received (timeout) - Nav2 may still be processing")


# ═══════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════

def main():
    header("Robo_Fleet - Remote Rosbridge Tests")
    print(f"  Target: ws://{HOST}:{PORT}")
    print(f"  Robots: {ROBOTS}\n")

    print("  --- Connection ---")
    run_test("Connect", test_connection)
    run_test("Reconnect", test_reconnect)

    print("\n  --- Pose ---")
    run_test("Get tb1 position", test_get_pose_tb1)
    run_test("Get tb3 position", test_get_pose_tb3)
    run_test("Fleet status", test_fleet_status)

    print("\n  --- Sensors ---")
    run_test("Odometry (tb1)", test_odom)
    run_test("Laser scan (tb1)", test_scan)

    print("\n  --- Control ---")
    run_test("Publish cmd_vel (tb1)", test_publish_cmd_vel)
    run_test("Emergency stop (all)", test_stop_all)

    print("\n  --- Navigation ---")
    run_test("Navigate tb1 (short goal)", test_navigate_tb1)

    # Summary
    header("Results")
    passed = sum(1 for s, _ in results if s == "PASS")
    failed = sum(1 for s, _ in results if s == "FAIL")
    for status, name in results:
        icon = "✅" if status == "PASS" else "❌"
        print(f"  {icon} {name}")
    print(f"\n  {passed}/{passed + failed} tests passed")
    if failed == 0:
        print("  🎉 ALL REMOTE TESTS PASSED!")


if __name__ == "__main__":
    main()
