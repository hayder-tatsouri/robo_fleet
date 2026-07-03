#!/usr/bin/env python3
"""
Robo_Fleet - Scale Benchmark
─────────────────────────────
Benchmarks FleetStateManager and TaskPlanner at 10+ robots.
No rosbridge needed - uses in-memory state.

Usage:
  python sim/scale_test.py
  python sim/scale_test.py --robots 20
"""

import math
import time
import argparse
import sys
sys.path.insert(0, 'mcp_server')

from coordination.fleet_state import FleetStateManager, RobotState
from coordination.task_planner import TaskPlanner
from coordination.collision_predictor import CollisionPredictor


def benchmark(fn, iterations=100):
    """Run function N times and return avg/min/max time in ms."""
    times = []
    for _ in range(iterations):
        start = time.perf_counter()
        fn()
        elapsed = (time.perf_counter() - start) * 1000  # ms
        times.append(elapsed)
    return {
        "avg_ms": round(sum(times) / len(times), 3),
        "min_ms": round(min(times), 3),
        "max_ms": round(max(times), 3),
        "p99_ms": round(sorted(times)[int(len(times) * 0.99)], 3),
    }


def main():
    parser = argparse.ArgumentParser(description="Robo_Fleet Scale Benchmark")
    parser.add_argument("--robots", type=int, default=10, help="Number of robots")
    parser.add_argument("--tasks", type=int, default=0, help="Number of tasks (default: same as robots)")
    args = parser.parse_args()

    num_robots = args.robots
    num_tasks = args.tasks or num_robots

    print(f"""
{'=' * 60}
  Robo_Fleet - Scale Benchmark
{'=' * 60}
  Robots: {num_robots}
  Tasks:  {num_tasks}
{'=' * 60}
""")

    # ─── Setup: Create fleet with N robots ───
    robot_ids = [f"tb{i+1}" for i in range(num_robots)]

    # Create manager without network (direct state injection)
    manager = FleetStateManager.get_instance()
    manager.robots = {}
    for i, rid in enumerate(robot_ids):
        robot = RobotState(rid)
        # Spread robots in a grid
        robot.x = (i % 5) * 1.0 - 2.0
        robot.y = (i // 5) * 1.0 - 2.0
        robot.battery = 50 + (i * 5) % 50
        robot.status = "idle"
        robot.last_seen = time.time()
        manager.robots[rid] = robot

    manager._running = True
    planner = TaskPlanner(manager)
    predictor = CollisionPredictor(manager)

    # ─── Benchmark 1: FleetStateManager queries ───
    print("  --- FleetStateManager Queries ---")

    r1 = benchmark(lambda: manager.get_all_states())
    print(f"  get_all_states({num_robots} robots):  avg={r1['avg_ms']:.3f}ms  p99={r1['p99_ms']:.3f}ms")

    r2 = benchmark(lambda: manager.get_available_robots())
    print(f"  get_available_robots():        avg={r2['avg_ms']:.3f}ms  p99={r2['p99_ms']:.3f}ms")

    r3 = benchmark(lambda: manager.get_nearest_available(0.0, 0.0))
    print(f"  get_nearest_available():       avg={r3['avg_ms']:.3f}ms  p99={r3['p99_ms']:.3f}ms")

    r4 = benchmark(lambda: manager.get_position(robot_ids[0]))
    print(f"  get_position() (O(1)):         avg={r4['avg_ms']:.3f}ms  p99={r4['p99_ms']:.3f}ms")

    # ─── Benchmark 2: Task Allocation ───
    print(f"\n  --- Task Allocation ({num_tasks} tasks x {num_robots} robots) ---")

    def allocate_tasks():
        # Reset planner
        planner.tasks = {}
        planner._task_counter = 0
        for r in manager.robots.values():
            r.status = "idle"
        tasks = []
        for i in range(num_tasks):
            t = planner.create_task(
                x=(i % 5) * 0.8 - 1.5,
                y=(i // 5) * 0.8 - 1.5,
                priority=i % 3,
            )
            tasks.append(t)
        return planner.allocate(tasks)

    r5 = benchmark(allocate_tasks, iterations=50)
    assignments = allocate_tasks()
    print(f"  allocate({num_tasks} tasks):          avg={r5['avg_ms']:.3f}ms  p99={r5['p99_ms']:.3f}ms")
    print(f"  → Assigned: {len(assignments)}/{num_tasks}")

    # ─── Benchmark 3: Collision Detection ───
    print(f"\n  --- Collision Prediction (O(N^2) = {num_robots * (num_robots-1)//2} pairs) ---")

    # Set some robots to navigating
    for i, r in enumerate(manager.robots.values()):
        if i % 2 == 0:
            r.status = "navigating"
            r.goal_x = r.x + 0.5
            r.goal_y = r.y + 0.3

    r6 = benchmark(lambda: predictor.predict_all(), iterations=50)
    risks = predictor.predict_all()
    print(f"  predict_all() ({num_robots} robots): avg={r6['avg_ms']:.3f}ms  p99={r6['p99_ms']:.3f}ms")
    print(f"  → Collision risks found: {len(risks)}")

    # ─── Benchmark 4: Cost function ───
    print(f"\n  --- Cost Function ---")

    robot = list(manager.robots.values())[0]
    task = planner.create_task(x=1.0, y=1.0)
    r7 = benchmark(lambda: planner.compute_cost(robot, task), iterations=1000)
    print(f"  compute_cost():                avg={r7['avg_ms']:.4f}ms  (1000 iterations)")

    # ─── Summary Table ───
    print(f"""
{'=' * 60}
  RESULTS SUMMARY ({num_robots} robots, {num_tasks} tasks)
{'=' * 60}
  {'Operation':<35} {'Avg (ms)':<12} {'P99 (ms)':<12}
  {'─' * 59}
  {'get_all_states()':<35} {r1['avg_ms']:<12.3f} {r1['p99_ms']:<12.3f}
  {'get_available_robots()':<35} {r2['avg_ms']:<12.3f} {r2['p99_ms']:<12.3f}
  {'get_nearest_available()':<35} {r3['avg_ms']:<12.3f} {r3['p99_ms']:<12.3f}
  {'get_position() O(1)':<35} {r4['avg_ms']:<12.3f} {r4['p99_ms']:<12.3f}
  {'allocate() greedy':<35} {r5['avg_ms']:<12.3f} {r5['p99_ms']:<12.3f}
  {'predict_all() O(N^2)':<35} {r6['avg_ms']:<12.3f} {r6['p99_ms']:<12.3f}
  {'compute_cost()':<35} {r7['avg_ms']:<12.4f} {r7['p99_ms']:<12.4f}
{'=' * 60}
""")

    # Verdict
    if r5['p99_ms'] < 50 and r6['p99_ms'] < 50:
        print(f"  ✅ All operations under 50ms at {num_robots} robots - SCALES WELL")
    elif r5['p99_ms'] < 100:
        print(f"  ⚠️  Approaching limits - consider optimization above {num_robots} robots")
    else:
        print(f"  ❌ Too slow at {num_robots} robots - need optimization")

    # Cleanup
    FleetStateManager._instance = None


if __name__ == "__main__":
    main()
