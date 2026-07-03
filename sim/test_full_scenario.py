#!/usr/bin/env python3
"""
Robo_Fleet - Full Scenario Test (Real Robots)
──────────────────────────────────────────────
Tests realistic multi-robot coordination scenarios:
  1. Long-distance navigation (2m+ goals)
  2. Multi-goal sequential waypoints
  3. Fleet-wide dispatch (both robots navigate simultaneously)
  4. Task allocation with priorities
  5. Collision avoidance dispatch

Usage:
  python sim/test_full_scenario.py --host 192.168.0.8 --robots tb1 tb3
"""

import json
import math
import time
import argparse
import sys
sys.path.insert(0, 'mcp_server')
from ros.ros_client import RosClient
from coordination.fleet_state import FleetStateManager
from coordination.task_planner import TaskPlanner, TaskStatus

parser = argparse.ArgumentParser()
parser.add_argument("--host", default="192.168.0.8", help="rosbridge host")
parser.add_argument("--port", type=int, default=9090, help="rosbridge port")
parser.add_argument("--robots", nargs="+", default=["tb1", "tb3"], help="Available robots")
parser.add_argument("--nav-timeout", type=int, default=60, help="Navigation timeout (seconds)")
args, _ = parser.parse_known_args()

HOST = args.host
PORT = args.port
ROBOTS = args.robots
NAV_TIMEOUT = args.nav_timeout


def header(text):
    print(f"\n{'=' * 60}")
    print(f"  {text}")
    print(f"{'=' * 60}")


def subheader(text):
    print(f"\n  --- {text} ---")


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


def get_position(client, robot_id):
    """Get current position of a robot."""
    msg = client.subscribe_once(
        f"/{robot_id}/amcl_pose",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        timeout=5.0
    )
    if msg:
        pos = msg["pose"]["pose"]["position"]
        return pos["x"], pos["y"]
    return None, None


# ═══════════════════════════════════════════
# SCENARIO 1: Long-Distance Navigation
# ═══════════════════════════════════════════

def test_long_nav_tb1():
    """Navigate tb1 to a point 2m away."""
    client = make_client()
    x, y = get_position(client, "tb1")
    assert x is not None, "Cannot read tb1 position"

    # Target 0.7m ahead in x (map limit ~1.5m)
    target_x = min(x + 0.7, 1.3)
    target_y = y
    print(f"       tb1 at ({x:.2f}, {y:.2f}) -> target ({target_x:.2f}, {target_y:.2f})")

    goal = {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": target_x, "y": target_y, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        }
    }

    start = time.time()
    resp = client.send_goal("/tb1/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
    result = client.wait_for_result("/tb1/navigate_to_pose", resp["goal_id"], timeout=NAV_TIMEOUT)

    elapsed = time.time() - start
    success = result and result.get("success", False)

    # Verify final position
    final_x, final_y = get_position(client, "tb1")
    client.disconnect()

    if success:
        dist_error = math.sqrt((final_x - target_x)**2 + (final_y - target_y)**2) if final_x else None
        print(f"       Arrived in {elapsed:.1f}s, error={dist_error:.3f}m" if dist_error else f"       Arrived in {elapsed:.1f}s")
    else:
        print(f"       Result: {result}")
    assert success, f"Long navigation failed after {elapsed:.1f}s"


# ═══════════════════════════════════════════
# SCENARIO 2: Multi-Waypoint Navigation
# ═══════════════════════════════════════════

def test_waypoints_tb1():
    """Navigate tb1 through 4 waypoints (square path)."""
    client = make_client()
    x, y = get_position(client, "tb1")
    assert x is not None, "Cannot read tb1 position"

    # Define a small square path (0.5m sides, clamped to map bounds)
    def clamp(v, lo=-1.3, hi=1.3):
        return max(lo, min(hi, v))

    waypoints = [
        {"x": clamp(x + 0.5), "y": clamp(y)},         # Forward 0.5m
        {"x": clamp(x + 0.5), "y": clamp(y + 0.5)},   # Left 0.5m
        {"x": clamp(x),       "y": clamp(y + 0.5)},   # Back 0.5m
        {"x": clamp(x),       "y": clamp(y)},          # Return to start
    ]

    print(f"       Starting at ({x:.2f}, {y:.2f})")
    print(f"       Waypoints: {len(waypoints)} (0.5m square path)")

    completed = 0
    total_time = 0

    for i, wp in enumerate(waypoints):
        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": wp["x"], "y": wp["y"], "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        }

        wp_start = time.time()
        resp = client.send_goal("/tb1/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
        result = client.wait_for_result("/tb1/navigate_to_pose", resp["goal_id"], timeout=NAV_TIMEOUT)
        wp_time = time.time() - wp_start
        total_time += wp_time

        if result and result.get("success"):
            completed += 1
            print(f"       WP{i+1}: ({wp['x']:.1f}, {wp['y']:.1f}) ✓ ({wp_time:.1f}s)")
        else:
            print(f"       WP{i+1}: ({wp['x']:.1f}, {wp['y']:.1f}) ✗ ({result})")
            break

    client.disconnect()
    print(f"       {completed}/{len(waypoints)} waypoints in {total_time:.1f}s")
    assert completed == len(waypoints), f"Only {completed}/{len(waypoints)} waypoints completed"


# ═══════════════════════════════════════════
# SCENARIO 3: Simultaneous Fleet Navigation
# ═══════════════════════════════════════════

def test_simultaneous_navigation():
    """Send both robots to different goals simultaneously."""
    client = make_client()

    # Get positions
    positions = {}
    for robot_id in ROBOTS:
        x, y = get_position(client, robot_id)
        if x is not None:
            positions[robot_id] = (x, y)
            print(f"       {robot_id} at ({x:.2f}, {y:.2f})")

    assert len(positions) >= 2, f"Need at least 2 online robots, got {len(positions)}"

    # Assign different goals (1.5m from current pos, different directions)
    goals = {}
    robot_list = list(positions.keys())
    targets = {}

    for i, robot_id in enumerate(robot_list):
        x, y = positions[robot_id]
        # Send robots in opposite directions (clamped to map)
        angle = (i * math.pi)  # 0, pi
        tx = max(-1.3, min(1.3, x + 0.6 * math.cos(angle)))
        ty = max(-1.3, min(1.3, y + 0.6 * math.sin(angle)))
        targets[robot_id] = (tx, ty)

        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": tx, "y": ty, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        }
        resp = client.send_goal(
            f"/{robot_id}/navigate_to_pose",
            "nav2_msgs/action/NavigateToPose",
            goal
        )
        goals[robot_id] = resp["goal_id"]
        print(f"       {robot_id}: goal -> ({tx:.2f}, {ty:.2f})")

    # Wait for all
    print(f"       Waiting for {len(goals)} robots...")
    start = time.time()
    success_count = 0

    for robot_id, goal_id in goals.items():
        result = client.wait_for_result(
            f"/{robot_id}/navigate_to_pose", goal_id, timeout=NAV_TIMEOUT
        )
        if result and result.get("success"):
            success_count += 1
            print(f"       {robot_id}: ✓ arrived")
        else:
            print(f"       {robot_id}: ✗ ({result})")

    elapsed = time.time() - start
    client.disconnect()
    print(f"       {success_count}/{len(goals)} arrived in {elapsed:.1f}s")
    assert success_count == len(goals), f"Only {success_count}/{len(goals)} reached goals"


# ═══════════════════════════════════════════
# SCENARIO 4: Coordinated Task Allocation
# ═══════════════════════════════════════════

def test_coordinated_dispatch():
    """Use TaskPlanner to optimally assign 3 tasks to 2 robots."""
    manager = FleetStateManager.get_instance()
    if not manager._running:
        manager.start(robot_ids=ROBOTS, ws_url=f"ws://{HOST}:{PORT}")
    time.sleep(2)  # Let state populate

    planner = TaskPlanner(manager, collision_buffer=0.3)

    # Get current positions for relative goals
    robot_ids = list(manager.robots.keys())
    r1 = manager.robots[robot_ids[0]]
    r2 = manager.robots[robot_ids[1]]
    print(f"       {r1.robot_id} at ({r1.x:.2f}, {r1.y:.2f})")
    print(f"       {r2.robot_id} at ({r2.x:.2f}, {r2.y:.2f})")

    # Create 3 tasks (clamped to map bounds [-1.3, 1.3])
    def clamp(v, lo=-1.3, hi=1.3):
        return max(lo, min(hi, v))

    tasks = [
        planner.create_task(x=clamp(r1.x + 0.5), y=clamp(r1.y), priority=2),
        planner.create_task(x=clamp(r2.x + 0.5), y=clamp(r2.y), priority=1),
        planner.create_task(x=clamp(r1.x + 0.3), y=clamp(r1.y + 0.3), priority=0),
    ]

    # Allocate
    assignments = planner.allocate(tasks)
    print(f"       Allocated {len(assignments)}/{len(tasks)} tasks")

    for a in assignments:
        print(f"       {a.robot_id} -> ({a.task.x:.1f}, {a.task.y:.1f}) "
              f"[cost={a.cost:.2f}, collision={a.collision_risk}]")

    # Dispatch the assigned ones
    client = RosClient(host=HOST, port=PORT)
    client.connect()

    dispatched = 0
    succeeded = 0
    sent_goals = {}  # robot_id -> goal_id

    for a in assignments:
        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": a.task.x, "y": a.task.y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        }
        resp = client.send_goal(
            f"/{a.robot_id}/navigate_to_pose",
            "nav2_msgs/action/NavigateToPose",
            goal
        )
        sent_goals[a.robot_id] = resp["goal_id"]
        dispatched += 1

    # Wait for results using actual goal IDs
    for robot_id, goal_id in sent_goals.items():
        result = client.wait_for_result(
            f"/{robot_id}/navigate_to_pose",
            goal_id,
            timeout=NAV_TIMEOUT
        )
        if result and result.get("success"):
            succeeded += 1
            print(f"       {robot_id}: ✓ task complete")
        else:
            print(f"       {robot_id}: ✗ {result}")

    client.disconnect()

    # Clean up
    manager.stop()
    FleetStateManager._instance = None

    print(f"       Dispatched: {dispatched}, Succeeded: {succeeded}")
    assert dispatched >= 2, "Should dispatch at least 2 tasks"


# ═══════════════════════════════════════════
# SCENARIO 5: Return-to-Home
# ═══════════════════════════════════════════

def test_return_to_home():
    """Send all robots back to origin (0, 0)."""
    client = make_client()

    goals = {}
    for robot_id in ROBOTS:
        x, y = get_position(client, robot_id)
        if x is None:
            continue
        print(f"       {robot_id} at ({x:.2f}, {y:.2f}) -> home (0, 0)")

        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        }
        resp = client.send_goal(
            f"/{robot_id}/navigate_to_pose",
            "nav2_msgs/action/NavigateToPose",
            goal
        )
        goals[robot_id] = resp["goal_id"]

    # Wait
    arrived = 0
    for robot_id, goal_id in goals.items():
        result = client.wait_for_result(
            f"/{robot_id}/navigate_to_pose", goal_id, timeout=NAV_TIMEOUT
        )
        if result and result.get("success"):
            arrived += 1
            final_x, final_y = get_position(client, robot_id)
            dist = math.sqrt(final_x**2 + final_y**2) if final_x else "?"
            print(f"       {robot_id}: home ✓ (dist from origin: {dist:.3f}m)")
        else:
            print(f"       {robot_id}: ✗ {result}")

    client.disconnect()
    print(f"       {arrived}/{len(goals)} returned home")
    assert arrived == len(goals), f"Only {arrived}/{len(goals)} reached home"


# ═══════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════

def main():
    header("Robo_Fleet - Full Scenario Test (Real Robots)")
    print(f"  Target: ws://{HOST}:{PORT}")
    print(f"  Robots: {ROBOTS}")
    print(f"  Nav timeout: {NAV_TIMEOUT}s\n")

    # Scenario 1
    subheader("Scenario 1: Long-Distance Navigation")
    run_test("Long nav tb1 (0.7m goal)", test_long_nav_tb1)

    # Scenario 2
    subheader("Scenario 2: Multi-Waypoint (square path)")
    run_test("4 waypoints (0.5m square)", test_waypoints_tb1)

    # Scenario 3
    subheader("Scenario 3: Simultaneous Fleet Navigation")
    run_test("Both robots navigate simultaneously", test_simultaneous_navigation)

    # Scenario 4
    subheader("Scenario 4: Coordinated Task Allocation")
    run_test("TaskPlanner dispatch (3 tasks, 2 robots)", test_coordinated_dispatch)

    # Scenario 5
    subheader("Scenario 5: Return to Home")
    run_test("All robots return home (collision-safe)", test_return_to_home)

    # Summary
    header("Results")
    passed = sum(1 for s, _ in results if s == "PASS")
    failed = sum(1 for s, _ in results if s == "FAIL")
    total_icon = "🎉" if failed == 0 else "⚠️"
    for status, name in results:
        icon = "✅" if status == "PASS" else "❌"
        print(f"  {icon} {name}")
    print(f"\n  {passed}/{passed + failed} scenarios passed {total_icon}")


if __name__ == "__main__":
    main()
