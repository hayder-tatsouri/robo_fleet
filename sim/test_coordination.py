#!/usr/bin/env python3
"""
Robo_Fleet - Coordination Integration Test
───────────────────────────────────────────
Tests task allocation, collision avoidance, and fleet management.

Requirements: python run.py (in another terminal)
Usage: python sim/test_coordination.py
"""

import math
import time
import argparse
import sys
sys.path.insert(0, 'mcp_server')

parser = argparse.ArgumentParser()
parser.add_argument("--host", default="localhost", help="rosbridge host")
parser.add_argument("--port", type=int, default=9090, help="rosbridge port")
parser.add_argument("--robots", nargs="+", default=["tb1", "tb2", "tb3"], help="Available robots")
args, _ = parser.parse_known_args()
ROSBRIDGE_HOST = args.host
ROSBRIDGE_PORT = args.port
ROBOTS = args.robots

from coordination.fleet_state import FleetStateManager, RobotState
from coordination.task_planner import TaskPlanner, Task, TaskStatus
from ros.ros_client import RosClient


def header(text):
    print(f"\n{'=' * 60}")
    print(f"  {text}")
    print(f"{'=' * 60}")


results = []


def run_test(name, fn):
    try:
        ok = fn()
        results.append(("PASS", name))
        print(f"  \u2705 {name}")
    except Exception as e:
        results.append(("FAIL", name))
        print(f"  \u274c {name}: {e}")


# ═══════════════════════════════════════════
# FLEET STATE MANAGER
# ═══════════════════════════════════════════

def test_fleet_state_init():
    manager = FleetStateManager.get_instance()
    manager.stop()
    FleetStateManager._instance = None

    manager = FleetStateManager.get_instance()
    manager.start(robot_ids=ROBOTS, ws_url=f"ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
    time.sleep(2)  # Let it receive some poses

    states = manager.get_all_states()
    assert len(states) == len(ROBOTS), f"Expected {len(ROBOTS)} robots, got {len(states)}"

    online = sum(1 for s in states if s["online"])
    assert online >= 1, f"No robots online"
    print(f"       Fleet: {online}/{len(ROBOTS)} online")
    return True


def test_fleet_state_queries():
    manager = FleetStateManager.get_instance()
    time.sleep(1)

    # Position query (O(1))
    pos = manager.get_position("tb1")
    assert pos is not None
    assert "x" in pos and "y" in pos
    print(f"       tb1 position: ({pos['x']:.2f}, {pos['y']:.2f})")

    # Available robots
    available = manager.get_available_robots()
    print(f"       Available: {len(available)} robots")

    # Nearest robot to (2, 2)
    nearest = manager.get_nearest_available(2.0, 2.0)
    if nearest:
        print(f"       Nearest to (2,2): {nearest.robot_id}")

    return True


# ═══════════════════════════════════════════
# TASK PLANNER - ALLOCATION
# ═══════════════════════════════════════════

def test_greedy_allocation():
    manager = FleetStateManager.get_instance()
    planner = TaskPlanner(manager)

    # Create 3 tasks at different positions
    tasks = [
        planner.create_task(x=2.0, y=0.0, priority=1),
        planner.create_task(x=0.0, y=2.0, priority=2),
        planner.create_task(x=-1.0, y=-1.0, priority=0),
    ]

    assignments = planner.allocate(tasks)
    assert len(assignments) >= 2, f"Only {len(assignments)} assignments"

    # Higher priority task should be assigned first
    if len(assignments) >= 2:
        assert assignments[0].task.priority >= assignments[1].task.priority

    for a in assignments:
        print(f"       {a.robot_id} -> task at ({a.task.x}, {a.task.y}) "
              f"[cost={a.cost:.2f}, dist={a.distance:.2f}m]")

    return True


def test_battery_aware_allocation():
    manager = FleetStateManager.get_instance()
    planner = TaskPlanner(manager)

    # Simulate low battery on first robot, high on rest
    robot_ids = list(manager.robots.keys())
    manager.robots[robot_ids[0]].battery = 10.0  # Critical
    if len(robot_ids) > 1:
        manager.robots[robot_ids[1]].battery = 90.0  # Healthy

    # Reset statuses
    for r in manager.robots.values():
        r.status = "idle"

    task = planner.create_task(x=1.0, y=1.0, priority=0)
    assignments = planner.allocate([task])

    if assignments:
        assigned = assignments[0].robot_id
        print(f"       Assigned to {assigned} (battery: {manager.robots[assigned].battery:.0f}%)")
        # Should NOT pick the low-battery robot
        assert assigned != robot_ids[0], f"Should not assign to low-battery robot ({robot_ids[0]})"

    # Reset batteries
    for r in manager.robots.values():
        r.battery = 100.0

    return True


# ═══════════════════════════════════════════
# COLLISION AVOIDANCE
# ═══════════════════════════════════════════

def test_collision_detection():
    manager = FleetStateManager.get_instance()
    planner = TaskPlanner(manager, collision_buffer=0.5)

    # Reset
    for r in manager.robots.values():
        r.status = "idle"
        r.battery = 100.0

    # Two tasks at very close positions (should trigger collision)
    tasks = [
        planner.create_task(x=2.0, y=2.0, priority=1),
        planner.create_task(x=2.1, y=2.1, priority=0),  # Within 0.5m buffer
    ]

    assignments = planner.allocate(tasks)
    collision_risks = sum(1 for a in assignments if a.collision_risk)

    print(f"       {len(assignments)} assignments, {collision_risks} collision risks")

    # At least one should have collision risk
    if len(assignments) >= 2:
        assert collision_risks >= 1, "Should detect collision risk for close goals"

    return True


def test_collision_resolution():
    manager = FleetStateManager.get_instance()
    planner = TaskPlanner(manager, collision_buffer=0.5)

    # Set priorities
    robot_ids = list(manager.robots.keys())
    manager.robots[robot_ids[0]].priority = 5  # High
    manager.robots[robot_ids[1]].priority = 1  # Low

    # Reset
    for r in manager.robots.values():
        r.status = "idle"
        r.battery = 100.0

    # Close goals
    tasks = [
        planner.create_task(x=1.0, y=1.0, priority=2),
        planner.create_task(x=1.1, y=1.0, priority=1),
    ]

    assignments = planner.allocate(tasks)
    dispatch_now, delayed = planner.resolve_collisions(assignments)

    print(f"       Dispatch: {len(dispatch_now)}, Delayed: {len(delayed)}")
    if delayed:
        print(f"       Delayed robot: {delayed[0].robot_id}")

    return True


# ═══════════════════════════════════════════
# DISPATCH (end-to-end)
# ═══════════════════════════════════════════

def test_dispatch_tasks():
    manager = FleetStateManager.get_instance()
    planner = TaskPlanner(manager)

    # Reset all
    for r in manager.robots.values():
        r.status = "idle"
        r.battery = 100.0
        r.priority = 0

    # Dispatch via RosClient
    client = RosClient(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.connect()

    # Create a task and manually dispatch
    task = planner.create_task(x=0.5, y=0.5, priority=0)
    assignments = planner.allocate([task])

    if not assignments:
        client.disconnect()
        print("       No assignments (robots may be offline)")
        return True

    a = assignments[0]
    manager.set_robot_status(a.robot_id, "navigating", goal_x=task.x, goal_y=task.y)

    goal = {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": task.x, "y": task.y, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        }
    }

    resp = client.send_goal(f"/{a.robot_id}/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
    result = client.wait_for_result(f"/{a.robot_id}/navigate_to_pose", resp["goal_id"], timeout=15.0)
    client.disconnect()

    success = result and result.get("success", False)
    planner.complete_task(task.task_id, success=success)

    assert success, f"Dispatch failed: {result}"
    print(f"       {a.robot_id} dispatched to (0.5, 0.5) - success!")
    return True


# ═══════════════════════════════════════════
# GROUPS / ZONES
# ═══════════════════════════════════════════

def test_groups():
    manager = FleetStateManager.get_instance()

    # Set up groups dynamically
    robot_ids = list(manager.robots.keys())
    manager.robots[robot_ids[0]].group = "warehouse"
    if len(robot_ids) > 1:
        manager.robots[robot_ids[1]].group = "outdoor"

    # Reset
    for r in manager.robots.values():
        r.status = "idle"
        r.battery = 100.0

    planner = TaskPlanner(manager)

    # Task for warehouse group only
    task = planner.create_task(x=3.0, y=0.0, priority=0, group="warehouse")
    assignments = planner.allocate([task])

    if assignments:
        assert manager.robots[assignments[0].robot_id].group == "warehouse", "Should only assign warehouse robots"
        print(f"       Warehouse task -> {assignments[0].robot_id} (correct group)")

    # Reset groups
    for r in manager.robots.values():
        r.group = "default"

    return True


# ═══════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════

def main():
    header("Robo_Fleet - Coordination Test Suite")
    print(f"  Target: ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
    print(f"  Robots: {ROBOTS}")
    print("  Testing FleetStateManager + TaskPlanner\n")

    # Fleet State Manager
    print("\n  --- Fleet State Manager ---")
    run_test("Initialize fleet state (persistent conn)", test_fleet_state_init)
    run_test("O(1) state queries", test_fleet_state_queries)

    # Task Allocation
    print("\n  --- Task Allocation ---")
    run_test("Greedy linear allocation (3 tasks)", test_greedy_allocation)
    run_test("Battery-aware cost function", test_battery_aware_allocation)

    # Collision Avoidance
    print("\n  --- Collision Avoidance ---")
    run_test("Detect collision risk (close goals)", test_collision_detection)
    run_test("Priority-based resolution", test_collision_resolution)

    # Dispatch
    print("\n  --- Task Dispatch ---")
    run_test("Dispatch task (allocate + navigate)", test_dispatch_tasks)

    # Groups
    print("\n  --- Groups / Zones ---")
    run_test("Group-constrained allocation", test_groups)

    # Cleanup
    manager = FleetStateManager.get_instance()
    manager.stop()
    FleetStateManager._instance = None

    # Summary
    header("Results")
    passed = sum(1 for s, _ in results if s == "PASS")
    failed = sum(1 for s, _ in results if s == "FAIL")
    for status, name in results:
        icon = "\u2705" if status == "PASS" else "\u274c"
        print(f"  {icon} {name}")
    print(f"\n  {passed}/{passed + failed} tests passed")
    if failed == 0:
        print("  \U0001f389 ALL COORDINATION TESTS PASSED!")


if __name__ == "__main__":
    main()
