"""
Hungarian Algorithm - Optimal task-robot assignment.
Uses scipy.optimize.linear_sum_assignment when available.
Falls back to greedy with a warning if scipy not installed.
"""

import math
import time
from dataclasses import dataclass


@dataclass
class OptimalAssignment:
    """Result of optimal assignment."""
    robot_id: str
    task_id: str
    x: float
    y: float
    cost: float
    distance: float

    def to_dict(self):
        return {
            "robot_id": self.robot_id,
            "task_id": self.task_id,
            "target": {"x": self.x, "y": self.y},
            "cost": round(self.cost, 4),
            "distance": round(self.distance, 4),
        }


def _compute_cost(robot, x, y):
    """Compute cost for assigning a robot to a task location."""
    dx = robot.x - x
    dy = robot.y - y
    distance = math.sqrt(dx * dx + dy * dy)

    # Battery penalty
    if robot.battery > 30:
        battery_penalty = 0.0
    elif robot.battery > 15:
        battery_penalty = (30 - robot.battery) / 30
    else:
        battery_penalty = 5.0

    # Busy penalty
    if robot.status == "idle":
        busy_penalty = 0.0
    elif robot.status == "navigating":
        busy_penalty = 2.0
    else:
        busy_penalty = 10.0

    cost = distance * (1 + battery_penalty) * (1 + busy_penalty)
    return cost, distance


def assign_optimal(fleet_manager, tasks):
    """
    Find optimal robot-task assignment using Hungarian algorithm.
    
    Args:
        fleet_manager: FleetStateManager instance
        tasks: List of dicts with keys: x, y, and optional task_id, priority
    
    Returns:
        Dict with assignments, method used, total cost comparison.
    """
    available = fleet_manager.get_available_robots()
    if not available:
        return {
            "success": False,
            "error": "No available robots",
            "method": "none",
            "assignments": [],
        }

    if not tasks:
        return {
            "success": False,
            "error": "No tasks provided",
            "method": "none",
            "assignments": [],
        }

    n_robots = len(available)
    n_tasks = len(tasks)

    # Build cost matrix
    cost_matrix = []
    distance_matrix = []
    for robot in available:
        row_costs = []
        row_dists = []
        for task in tasks:
            cost, dist = _compute_cost(robot, task.get("x", 0), task.get("y", 0))
            row_costs.append(cost)
            row_dists.append(dist)
        cost_matrix.append(row_costs)
        distance_matrix.append(row_dists)

    # Try Hungarian (scipy)
    method = "hungarian"
    try:
        from scipy.optimize import linear_sum_assignment
        import numpy as np

        # Pad matrix if not square
        max_dim = max(n_robots, n_tasks)
        padded = np.full((max_dim, max_dim), 1e6)
        for i in range(n_robots):
            for j in range(n_tasks):
                padded[i][j] = cost_matrix[i][j]

        row_ind, col_ind = linear_sum_assignment(padded)

        # Extract valid assignments
        assignments = []
        total_cost_optimal = 0
        for i, j in zip(row_ind, col_ind):
            if i < n_robots and j < n_tasks and padded[i][j] < 1e5:
                task = tasks[j]
                assignments.append(OptimalAssignment(
                    robot_id=available[i].robot_id,
                    task_id=task.get("task_id", f"task_{j}"),
                    x=task.get("x", 0),
                    y=task.get("y", 0),
                    cost=cost_matrix[i][j],
                    distance=distance_matrix[i][j],
                ))
                total_cost_optimal += cost_matrix[i][j]

    except ImportError:
        # Fallback to greedy
        method = "greedy_fallback"
        assignments = []
        total_cost_optimal = 0
        used_robots = set()

        # Sort tasks by... just iterate and pick lowest cost
        for j, task in enumerate(tasks):
            best_cost = float('inf')
            best_i = -1
            for i, robot in enumerate(available):
                if i in used_robots:
                    continue
                if cost_matrix[i][j] < best_cost:
                    best_cost = cost_matrix[i][j]
                    best_i = i
            if best_i >= 0:
                used_robots.add(best_i)
                assignments.append(OptimalAssignment(
                    robot_id=available[best_i].robot_id,
                    task_id=task.get("task_id", f"task_{j}"),
                    x=task.get("x", 0),
                    y=task.get("y", 0),
                    cost=cost_matrix[best_i][j],
                    distance=distance_matrix[best_i][j],
                ))
                total_cost_optimal += cost_matrix[best_i][j]

    # Also compute greedy cost for comparison
    greedy_cost = 0
    used = set()
    for j, task in enumerate(tasks):
        best_cost = float('inf')
        best_i = -1
        for i, robot in enumerate(available):
            if i in used:
                continue
            if cost_matrix[i][j] < best_cost:
                best_cost = cost_matrix[i][j]
                best_i = i
        if best_i >= 0:
            used.add(best_i)
            greedy_cost += best_cost

    improvement = ((greedy_cost - total_cost_optimal) / greedy_cost * 100) if greedy_cost > 0 else 0

    return {
        "success": True,
        "method": method,
        "assignments": [a.to_dict() for a in assignments],
        "total_cost_optimal": round(total_cost_optimal, 4),
        "total_cost_greedy": round(greedy_cost, 4),
        "improvement_percent": round(improvement, 2),
        "scipy_available": method == "hungarian",
        "note": "Using scipy Hungarian algorithm" if method == "hungarian" else "scipy not installed - using greedy fallback",
    }
