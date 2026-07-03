#!/usr/bin/env python3
"""
Robo_Fleet - Test New Features on Remote Rosbridge
───────────────────────────────────────────────────
Tests collision prediction, task queue, natural language,
Hungarian allocation, and dashboard against real robots.

Usage:
  python sim/test_new_features_remote.py --host 192.168.0.8 --robots tb1 tb3
"""

import math
import time
import argparse
import sys
sys.path.insert(0, 'mcp_server')

from ros.ros_client import RosClient
from coordination.fleet_state import FleetStateManager, RobotState
from coordination.task_planner import TaskPlanner
from coordination.collision_predictor import CollisionPredictor
from coordination.task_queue import TaskQueue
from coordination.hungarian import assign_optimal
from tools import natural_language as nl_tools

parser = argparse.ArgumentParser()
parser.add_argument("--host", default="192.168.0.8", help="rosbridge host")
parser.add_argument("--port", type=int, default=9090, help="rosbridge port")
parser.add_argument("--robots", nargs="+", default=["tb1", "tb3"], help="Available robots")
parser.add_argument("--nav-timeout", type=int, default=60, help="Navigation timeout")
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
        print(f"  \u2705 {name}")
    except Exception as e:
        results.append(("FAIL", name))
        print(f"  \u274c {name}: {e}")


def make_client():
    client = RosClient(host=HOST, port=PORT)
    client.connect()
    return client


def get_fleet_manager():
    """Get or create FleetStateManager connected to remote."""
    manager = FleetStateManager.get_instance()
    if not manager._running:
        manager.start(robot_ids=ROBOTS, ws_url=f"ws://{HOST}:{PORT}")
        time.sleep(2)  # Wait for state to populate
    return manager


# ═══════════════════════════════════════════
# 1. COLLISION PREDICTOR (on real positions)
# ═══════════════════════════════════════════

def test_collision_check_real_positions():
    """Read real robot positions and check for collisions."""
    manager = get_fleet_manager()
    time.sleep(1)

    predictor = CollisionPredictor(manager, buffer_distance=0.4, time_horizon=5.0)
    collisions = predictor.predict_all()

    print(f"       Robots tracked: {len(manager.robots)}")
    for rid, r in manager.robots.items():
        print(f"       {rid}: ({r.x:.2f}, {r.y:.2f}) status={r.status}")

    print(f"       Collision risks: {len(collisions)}")
    for c in collisions:
        print(f"       - {c}")

    # With idle robots there should be no collision
    # (unless they're physically very close)


def test_collision_with_simulated_motion():
    """Simulate robots moving toward each other and predict collision."""
    manager = get_fleet_manager()

    # Get real positions
    robot_ids = list(manager.robots.keys())
    r1 = manager.robots[robot_ids[0]]
    r2 = manager.robots[robot_ids[1]]

    print(f"       {r1.robot_id}: ({r1.x:.2f}, {r1.y:.2f})")
    print(f"       {r2.robot_id}: ({r2.x:.2f}, {r2.y:.2f})")

    # Check current distance
    dist = math.sqrt((r1.x - r2.x)**2 + (r1.y - r2.y)**2)
    print(f"       Current distance: {dist:.2f}m")

    # Simulate: set goals toward each other (just in state, don't actually navigate)
    r1.goal_x = r2.x
    r1.goal_y = r2.y
    r1.status = "navigating"
    r2.goal_x = r1.x
    r2.goal_y = r1.y
    r2.status = "navigating"

    predictor = CollisionPredictor(manager, buffer_distance=0.4, time_horizon=10.0)
    collisions = predictor.predict_all()

    print(f"       Predicted collisions (head-on): {len(collisions)}")
    if dist > 0.5:
        assert len(collisions) >= 1, "Should predict collision when robots head toward each other"

    # Reset
    r1.status = "idle"
    r1.goal_x = None
    r2.status = "idle"
    r2.goal_x = None


# ═══════════════════════════════════════════
# 2. TASK QUEUE
# ═══════════════════════════════════════════

def test_task_queue_add_and_dispatch():
    """Add tasks to queue and dispatch one."""
    manager = get_fleet_manager()
    queue = TaskQueue(manager)
    queue.clear()
    time.sleep(0.2)

    # Verify empty
    before = len(queue.get_queue())

    # Add 3 tasks
    queue.add(x=0.3, y=0.0, priority=1)
    queue.add(x=-0.3, y=0.0, priority=2)
    queue.add(x=0.0, y=0.3, priority=0)

    status = queue.get_queue()
    added = len(status) - before
    print(f"       Queue size: {len(status)} (added {added})")
    assert added >= 3, f"Expected to add 3, only added {added}"

    # Peek (highest priority first)
    next_task = queue.peek()
    if next_task:
        print(f"       Peek: ({next_task.get('x', '?')}, {next_task.get('y', '?')}) priority={next_task.get('priority', '?')}")

    # Dispatch the highest priority task to nearest robot
    client = make_client()
    task = next_task
    robot = manager.get_nearest_available(task["x"], task["y"])

    if robot and task:
        tx, ty = task.get("x", 0), task.get("y", 0)
        print(f"       Dispatching {robot.robot_id} to ({tx}, {ty})")
        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": tx, "y": ty, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        }
        resp = client.send_goal(f"/{robot.robot_id}/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
        result = client.wait_for_result(f"/{robot.robot_id}/navigate_to_pose", resp["goal_id"], timeout=NAV_TIMEOUT)
        success = result and result.get("success", False)
        print(f"       Navigation: {'success' if success else 'failed'}")
        assert success, f"Queue dispatch failed: {result}"
    else:
        print(f"       No available robots or task")

    client.disconnect()
    queue.clear()


# ═══════════════════════════════════════════
# 3. NATURAL LANGUAGE GOALS
# ═══════════════════════════════════════════

def test_location_registry():
    """Test named locations with real navigation."""
    # List defaults
    locations = nl_tools._load_locations()
    print(f"       Default locations: {len(locations)}")
    for name, coords in list(locations.items())[:4]:
        print(f"       - {name}: ({coords['x']}, {coords['y']})")

    # Add a custom location
    nl_tools.add_location(name="test_spot", x=0.5, y=0.3)
    locations = nl_tools._load_locations()
    assert "test_spot" in locations
    print(f"       Added 'test_spot' at (0.5, 0.3)")

    # Remove it
    nl_tools.remove_location(name="test_spot")
    locations = nl_tools._load_locations()
    assert "test_spot" not in locations
    print(f"       Removed 'test_spot'")


def test_go_to_named_location():
    """Navigate a robot to a named location."""
    manager = get_fleet_manager()

    # Add a safe location within map bounds
    nl_tools.add_location(name="nav_test_point", x=0.5, y=0.3)
    locations = nl_tools._load_locations()
    loc = locations.get("nav_test_point")
    assert loc is not None

    # Explicitly use tb1 for this test (ensure both robots get used)
    robot = manager.robots.get("tb1")
    if not robot or not robot.is_available:
        robot = manager.get_nearest_available(loc["x"], loc["y"])
    if not robot:
        print(f"       No available robots - skipping nav")
        return

    print(f"       Sending {robot.robot_id} to 'safe_point' ({loc['x']}, {loc['y']})")

    client = make_client()
    goal = {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": loc["x"], "y": loc["y"], "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        }
    }
    resp = client.send_goal(f"/{robot.robot_id}/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
    result = client.wait_for_result(f"/{robot.robot_id}/navigate_to_pose", resp["goal_id"], timeout=NAV_TIMEOUT)
    client.disconnect()

    success = result and result.get("success", False)
    print(f"       Navigation: {'success' if success else 'failed'}")
    assert success, f"Navigation to named location failed: {result}"

    # Cleanup
    nl_tools.remove_location(name="nav_test_point")


# ═══════════════════════════════════════════
# 4. HUNGARIAN OPTIMAL ALLOCATION
# ═══════════════════════════════════════════

def test_hungarian_vs_greedy():
    """Compare Hungarian and greedy allocation on real positions."""
    manager = get_fleet_manager()

    # Reset states
    for r in manager.robots.values():
        r.status = "idle"
        r.battery = 100.0

    # Create tasks at various positions
    tasks = [
        {"x": 0.5, "y": 0.5},
        {"x": -0.5, "y": -0.5},
        {"x": 0.0, "y": 0.5},
    ]

    # Greedy allocation
    planner = TaskPlanner(manager)
    greedy_tasks = [planner.create_task(x=t["x"], y=t["y"]) for t in tasks]
    greedy_assignments = planner.allocate(greedy_tasks)
    greedy_cost = sum(a.cost for a in greedy_assignments)

    print(f"       Greedy: {len(greedy_assignments)} assignments, total cost={greedy_cost:.3f}")
    for a in greedy_assignments:
        print(f"         {a.robot_id} -> ({a.task.x}, {a.task.y}) cost={a.cost:.3f}")

    # Hungarian allocation
    hungarian_result = assign_optimal(manager, tasks)

    h_cost = hungarian_result.get("total_cost", 0)
    h_assignments = hungarian_result.get("assignments", [])
    print(f"       Hungarian: {len(h_assignments)} assignments, total cost={h_cost:.3f}")
    for a in h_assignments:
        # Handle both flat and nested formats
        if "target" in a:
            x, y = a["target"]["x"], a["target"]["y"]
        else:
            x, y = a.get("x", 0), a.get("y", 0)
        print(f"         {a['robot_id']} -> ({x}, {y}) cost={a.get('cost', 0):.3f}")

    # Compare
    if greedy_cost > 0:
        improvement = (greedy_cost - h_cost) / greedy_cost * 100
        print(f"       Improvement: {improvement:.1f}% (Hungarian vs Greedy)")


def test_hungarian_dispatch():
    """Use Hungarian to assign and navigate - uses both robots with longer trips."""
    manager = get_fleet_manager()

    # Reset
    for r in manager.robots.values():
        r.status = "idle"
        r.battery = 100.0

    # Two tasks - one for each robot, meaningful distance
    tasks = [
        {"x": 0.8, "y": 0.3},   # Longer trip
        {"x": -0.8, "y": -0.3},  # Opposite side
    ]
    result = assign_optimal(manager, tasks)
    assignments = result.get("assignments", [])

    if not assignments:
        print("       No assignments (no available robots)")
        return

    client = make_client()
    goals_sent = {}

    for a in assignments:
        if "target" in a:
            tx, ty = a["target"]["x"], a["target"]["y"]
        else:
            tx, ty = a.get("x", 0), a.get("y", 0)
        print(f"       {a['robot_id']} -> ({tx}, {ty})")

        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": tx, "y": ty, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        }
        resp = client.send_goal(f"/{a['robot_id']}/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
        goals_sent[a['robot_id']] = resp["goal_id"]

    # Wait for both
    success_count = 0
    for robot_id, goal_id in goals_sent.items():
        nav_result = client.wait_for_result(f"/{robot_id}/navigate_to_pose", goal_id, timeout=NAV_TIMEOUT)
        ok = nav_result and nav_result.get("success", False)
        if ok:
            success_count += 1
        print(f"       {robot_id}: {'arrived' if ok else 'failed'}")

    client.disconnect()
    print(f"       {success_count}/{len(goals_sent)} robots navigated successfully")


# ═══════════════════════════════════════════
# 5. SCALE BENCHMARK (in-memory, no network)
# ═══════════════════════════════════════════

def test_scale_benchmark():
    """Benchmark allocation and collision detection with real robot data as seed."""
    manager = get_fleet_manager()
    import random

    # Create 10 fake robots seeded from real positions
    real_positions = [(r.x, r.y) for r in manager.robots.values()]

    fake_robots = {}
    for i in range(10):
        r = RobotState(f"bot_{i:02d}")
        # Scatter around real positions
        seed = real_positions[i % len(real_positions)]
        r.x = seed[0] + random.uniform(-1, 1)
        r.y = seed[1] + random.uniform(-1, 1)
        r.battery = random.uniform(50, 100)
        r.status = "idle"
        r.last_seen = time.time()
        fake_robots[r.robot_id] = r

    # Benchmark allocation
    class FakeFleet:
        def __init__(self, robots):
            self.robots = robots
        def get_available_robots(self):
            return [r for r in self.robots.values() if r.status == "idle"]

    planner = TaskPlanner.__new__(TaskPlanner)
    planner.fleet = FakeFleet(fake_robots)
    planner.collision_buffer = 0.4
    planner.robot_speed = 0.5
    planner.tasks = {}
    planner.assignments = {}
    planner._task_counter = 0
    planner._lock = __import__('threading').Lock()

    # 10 tasks
    tasks = []
    for i in range(10):
        t = planner.create_task(
            x=random.uniform(-1.3, 1.3),
            y=random.uniform(-1.3, 1.3),
            priority=random.randint(0, 5)
        )
        tasks.append(t)

    start = time.time()
    assignments = planner.allocate(tasks)
    alloc_time = (time.time() - start) * 1000

    print(f"       10 tasks x 10 robots:")
    print(f"       Allocation: {alloc_time:.2f}ms ({len(assignments)} assigned)")

    # Benchmark collision prediction
    predictor = CollisionPredictor.__new__(CollisionPredictor)
    predictor.fleet = FakeFleet(fake_robots)
    predictor.buffer_distance = 0.4
    predictor.time_horizon = 5.0
    predictor.robot_speed = 0.3

    # Set some robots as navigating
    for i, r in enumerate(fake_robots.values()):
        if i < 5:
            r.status = "navigating"
            r.goal_x = random.uniform(-1, 1)
            r.goal_y = random.uniform(-1, 1)

    start = time.time()
    collisions = predictor.predict_all()
    coll_time = (time.time() - start) * 1000

    print(f"       Collision check: {coll_time:.2f}ms ({len(collisions)} risks)")
    assert alloc_time < 100, f"Allocation too slow: {alloc_time}ms"
    assert coll_time < 50, f"Collision check too slow: {coll_time}ms"


# ═══════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════

def main():
    header("Robo_Fleet - New Features Remote Test")
    print(f"  Target: ws://{HOST}:{PORT}")
    print(f"  Robots: {ROBOTS}\n")

    # 1. Collision Prediction
    subheader("1. Collision Prediction")
    run_test("Check collisions (real positions)", test_collision_check_real_positions)
    run_test("Predict head-on collision", test_collision_with_simulated_motion)

    # 2. Task Queue
    subheader("2. Task Queue + Dispatch")
    run_test("Queue: add, peek, pop, dispatch", test_task_queue_add_and_dispatch)

    # 3. Natural Language
    subheader("3. Natural Language Goals")
    run_test("Location registry (add/list/remove)", test_location_registry)
    run_test("Navigate to named location", test_go_to_named_location)

    # 4. Hungarian Allocation
    subheader("4. Hungarian Optimal Allocation")
    run_test("Hungarian vs Greedy comparison", test_hungarian_vs_greedy)
    run_test("Hungarian dispatch (real nav)", test_hungarian_dispatch)

    # 5. Scale
    subheader("5. Scale Benchmark")
    run_test("10 robots x 10 tasks (< 100ms)", test_scale_benchmark)

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
        print("  \U0001f389 ALL NEW FEATURES PASSED ON REMOTE!")


if __name__ == "__main__":
    main()
