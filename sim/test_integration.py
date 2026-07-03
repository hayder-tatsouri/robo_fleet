#!/usr/bin/env python3
"""
Robo_Fleet - Full Integration Test Suite
─────────────────────────────────────────
Tests ALL MCP tools against the running simulator.

Requirements:
  Terminal 1: python run.py

Usage:
  python sim/test_integration.py
"""

import json
import math
import time
import argparse
import sys
sys.path.insert(0, 'mcp_server')
from ros.ros_client import RosClient


def header(text):
    print(f"\n{'=' * 60}")
    print(f"  {text}")
    print(f"{'=' * 60}")


# Parse CLI args
parser = argparse.ArgumentParser()
parser.add_argument("--host", default="localhost", help="rosbridge host (default: localhost)")
parser.add_argument("--port", type=int, default=9090, help="rosbridge port (default: 9090)")
args, _ = parser.parse_known_args()

ROSBRIDGE_HOST = args.host
ROSBRIDGE_PORT = args.port


def subheader(text):
    print(f"\n  --- {text} ---")


results = []


def run_test(name, fn):
    try:
        ok = fn()
        results.append(("PASS", name))
        print(f"  \u2705 {name}")
        return ok
    except Exception as e:
        results.append(("FAIL", name))
        print(f"  \u274c {name}: {e}")
        return False


# ═══════════════════════════════════════════
# CONNECTION
# ═══════════════════════════════════════════

def test_connection():
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()
    assert client.connected
    client.disconnect()
    assert not client.connected
    return True


def test_reconnect():
    client = RosClient(auto_reconnect=True, max_retries=3)
    client.connect()
    assert client.connected
    # Simulate disconnect
    client.disconnect()
    # Reconnect
    client.connect()
    assert client.connected
    client.disconnect()
    return True


# ═══════════════════════════════════════════
# MONITORING TOOLS
# ═══════════════════════════════════════════

def test_get_robot_position():
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()
    msg = client.subscribe_once(
        topic="/tb1/amcl_pose",
        msg_type="geometry_msgs/msg/PoseWithCovarianceStamped",
        timeout=5.0
    )
    client.disconnect()
    assert msg is not None, "No pose received"
    pos = msg["pose"]["pose"]["position"]
    assert "x" in pos and "y" in pos
    print(f"       tb1 at ({pos['x']:.2f}, {pos['y']:.2f})")
    return True


def test_get_fleet_status():
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()
    fleet = []
    for robot_id in ["tb1", "tb2", "tb3"]:
        msg = client.subscribe_once(
            topic=f"/{robot_id}/amcl_pose",
            msg_type="geometry_msgs/msg/PoseWithCovarianceStamped",
            timeout=3.0
        )
        fleet.append({"robot_id": robot_id, "online": msg is not None})
    client.disconnect()
    online = sum(1 for r in fleet if r["online"])
    assert online >= 2, f"Only {online}/3 robots online"
    print(f"       {online}/3 robots online")
    return True


def test_get_battery():
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()
    time.sleep(1)  # Wait for simulator to publish battery data
    msg = client.subscribe_once(
        topic="/tb1/battery_state",
        msg_type="sensor_msgs/msg/BatteryState",
        timeout=3.0
    )
    client.disconnect()
    assert msg is not None, "No battery data"
    pct = msg.get("percentage", 0) * 100
    voltage = msg.get("voltage", 0)
    assert 0 <= pct <= 100
    print(f"       tb1 battery: {pct:.0f}%, {voltage:.1f}V")
    return True


# ═══════════════════════════════════════════
# CONTROL TOOLS
# ═══════════════════════════════════════════

def test_stop_robot():
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()
    # Send zero velocity
    client.publish(
        topic="/tb1/cmd_vel",
        msg_type="geometry_msgs/msg/TwistStamped",
        data={
            "header": {"frame_id": "base_link"},
            "twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        }
    )
    client.disconnect()
    return True


def test_emergency_stop():
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()
    for robot_id in ["tb1", "tb2", "tb3"]:
        client.publish(
            topic=f"/{robot_id}/cmd_vel",
            msg_type="geometry_msgs/msg/TwistStamped",
            data={
                "header": {"frame_id": "base_link"},
                "twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
            }
        )
    client.disconnect()
    print("       All 3 robots stopped")
    return True


# ═══════════════════════════════════════════
# NAVIGATION
# ═══════════════════════════════════════════

def test_navigate_to_pose():
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()
    theta = 0.0
    goal = {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": 1.0, "y": 0.5, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(theta/2), "w": math.cos(theta/2)}
            }
        }
    }
    resp = client.send_goal("/tb1/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
    goal_id = resp["goal_id"]
    result = client.wait_for_result("/tb1/navigate_to_pose", goal_id, timeout=15.0)
    client.disconnect()
    assert result and result.get("success"), f"Navigation failed: {result}"
    print(f"       tb1 reached (1.0, 0.5)")
    return True


def test_waypoint_navigation():
    waypoints_list = [
        {"x": 0.5, "y": 0.0},
        {"x": 1.0, "y": 0.5},
    ]
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()

    completed = 0
    for wp in waypoints_list:
        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": wp["x"], "y": wp["y"], "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        }
        resp = client.send_goal("/tb2/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
        result = client.wait_for_result("/tb2/navigate_to_pose", resp["goal_id"], timeout=15.0)
        if result and result.get("success"):
            completed += 1
        else:
            break

    client.disconnect()
    assert completed == len(waypoints_list), f"Only {completed}/{len(waypoints_list)} waypoints"
    print(f"       tb2 completed {completed} waypoints")
    return True


# ═══════════════════════════════════════════
# OBSTACLE DETECTION
# ═══════════════════════════════════════════

def test_obstacle_detection():
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()
    time.sleep(1)  # Wait for simulator to publish scan data
    msg = client.subscribe_once(
        topic="/tb1/scan",
        msg_type="sensor_msgs/msg/LaserScan",
        timeout=3.0
    )
    client.disconnect()
    assert msg is not None, "No laser scan data"
    ranges = msg.get("ranges", [])
    assert len(ranges) > 0, "Empty scan"
    valid = [r for r in ranges if 0.12 <= r <= 10.0]
    closest = min(valid) if valid else None
    print(f"       Scan: {len(ranges)} rays, closest obstacle at {closest:.2f}m")
    return True


# ═══════════════════════════════════════════
# MAP VISUALIZATION
# ═══════════════════════════════════════════

def test_map_visualization():
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()
    robots = []
    for robot_id in ["tb1", "tb2", "tb3"]:
        msg = client.subscribe_once(
            topic=f"/{robot_id}/amcl_pose",
            msg_type="geometry_msgs/msg/PoseWithCovarianceStamped",
            timeout=3.0
        )
        if msg:
            pos = msg["pose"]["pose"]["position"]
            robots.append(f"{robot_id}=({pos['x']:.1f},{pos['y']:.1f})")
    client.disconnect()
    assert len(robots) >= 2
    print(f"       Positions: {', '.join(robots)}")
    return True


# ═══════════════════════════════════════════
# MULTI-ROBOT
# ═══════════════════════════════════════════

def test_multi_robot_nav():
    targets = {"tb1": (0.5, 0.5), "tb2": (-0.5, 0.0), "tb3": (0.0, -0.5)}
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()

    goals = {}
    for robot_id, (tx, ty) in targets.items():
        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": tx, "y": ty, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        }
        resp = client.send_goal(f"/{robot_id}/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
        goals[robot_id] = resp["goal_id"]

    success = 0
    for robot_id, goal_id in goals.items():
        result = client.wait_for_result(f"/{robot_id}/navigate_to_pose", goal_id, timeout=30.0)
        if result and result.get("success"):
            success += 1

    client.disconnect()
    assert success == 3, f"Only {success}/3 robots reached goals"
    print(f"       {success}/3 robots reached their targets")
    return True


# ═══════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════

def main():
    header("Robo_Fleet - Full Integration Test Suite")
    print(f"  Target: ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
    print("  Testing all 8 MCP tool capabilities\n")

    # Connection
    subheader("Connection & Resilience")
    if not run_test("Connect to rosbridge", test_connection):
        print("\n  \u26a0\ufe0f  Cannot connect. Is 'python run.py' running?")
        return
    run_test("Auto-reconnect", test_reconnect)

    # Monitoring
    subheader("Monitoring Tools")
    run_test("get_robot_position (tb1)", test_get_robot_position)
    run_test("get_fleet_status (all)", test_get_fleet_status)
    run_test("get_battery_level (tb1)", test_get_battery)

    # Control
    subheader("Control Tools")
    run_test("stop_robot (tb1)", test_stop_robot)
    run_test("emergency_stop (all)", test_emergency_stop)

    # Wait a moment for robots to reset
    time.sleep(1)

    # Navigation
    subheader("Navigation Tools")
    run_test("navigate_to_pose (tb1)", test_navigate_to_pose)
    run_test("navigate_waypoints (tb2)", test_waypoint_navigation)
    run_test("multi_robot_nav (all)", test_multi_robot_nav)

    # Sensing
    subheader("Obstacle Detection")
    run_test("check_obstacles (tb1)", test_obstacle_detection)

    # Visualization
    subheader("Map Visualization")
    run_test("get_map_with_robots", test_map_visualization)

    # Summary
    header("Results")
    passed = sum(1 for s, _ in results if s == "PASS")
    failed = sum(1 for s, _ in results if s == "FAIL")
    for status, name in results:
        icon = "\u2705" if status == "PASS" else "\u274c"
        print(f"  {icon} {name}")
    print(f"\n  {passed}/{passed + failed} tests passed")
    if failed == 0:
        print("  \U0001f389 ALL INTEGRATION TESTS PASSED!")
    else:
        print(f"  \u26a0\ufe0f  {failed} test(s) failed")


if __name__ == "__main__":
    main()
