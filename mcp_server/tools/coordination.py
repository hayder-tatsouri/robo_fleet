"""
MCP Tools for multi-robot coordination.
Tools: assign_tasks, dispatch_tasks, get_plan, replan, set_priority
"""

import math
import time
from server import mcp
from ros.ros_client import RosClient
from coordination.fleet_state import FleetStateManager
from coordination.task_planner import TaskPlanner, Task, TaskStatus


# Global planner instance (lazy init)
_planner = None
_manager = None


def _get_planner():
    """Get or create the global TaskPlanner instance."""
    global _planner, _manager
    if _planner is None:
        _manager = FleetStateManager.get_instance()
        if not _manager._running:
            _manager.start()
        _planner = TaskPlanner(_manager)
    return _planner


def _get_manager():
    """Get or create the global FleetStateManager instance."""
    global _manager
    if _manager is None:
        _manager = FleetStateManager.get_instance()
        if not _manager._running:
            _manager.start()
    return _manager


@mcp.tool()
def assign_tasks(
    tasks: list[dict],
    collision_buffer: float = 0.5,
) -> dict:
    """
    Assign a list of navigation tasks to the optimal robots using greedy allocation.
    Uses battery-aware cost function and collision detection.
    
    Args:
        tasks: List of task dicts with keys: x, y, and optional theta, priority, group.
               Example: [{"x": 2.0, "y": 3.0, "priority": 1}, {"x": -1.0, "y": 0.5}]
        collision_buffer: Minimum distance (meters) between robot goals (default: 0.5).
    
    Returns:
        Dict with assignments (robot-task pairs with cost breakdown) and any collision risks.
    """
    planner = _get_planner()
    planner.collision_buffer = collision_buffer

    # Create task objects
    created_tasks = []
    for t in tasks:
        task = planner.create_task(
            x=t.get("x", 0.0),
            y=t.get("y", 0.0),
            theta=t.get("theta", 0.0),
            priority=t.get("priority", 0),
            group=t.get("group", None),
        )
        created_tasks.append(task)

    # Run allocation
    assignments = planner.allocate(created_tasks)

    # Resolve collisions
    dispatch_now, delayed = planner.resolve_collisions(assignments)

    return {
        "success": True,
        "total_tasks": len(tasks),
        "assigned": len(assignments),
        "collision_risks": sum(1 for a in assignments if a.collision_risk),
        "dispatch_now": [a.to_dict() for a in dispatch_now],
        "delayed": [a.to_dict() for a in delayed],
        "unassigned": len(tasks) - len(assignments),
    }


@mcp.tool()
def dispatch_tasks(
    tasks: list[dict],
    collision_buffer: float = 0.5,
    timeout_per_task: float = 30.0,
) -> dict:
    """
    Assign AND dispatch navigation tasks - allocates optimally then navigates robots.
    Combines assign_tasks + navigate_to_pose in one call.
    
    Args:
        tasks: List of task dicts with keys: x, y, and optional theta, priority, group.
        collision_buffer: Minimum distance between robot goals (default: 0.5).
        timeout_per_task: Max seconds to wait per navigation goal (default: 30).
    
    Returns:
        Dict with per-robot dispatch results.
    """
    planner = _get_planner()
    manager = _get_manager()
    planner.collision_buffer = collision_buffer

    # Create and allocate tasks
    created_tasks = []
    for t in tasks:
        task = planner.create_task(
            x=t.get("x", 0.0),
            y=t.get("y", 0.0),
            theta=t.get("theta", 0.0),
            priority=t.get("priority", 0),
            group=t.get("group", None),
        )
        created_tasks.append(task)

    assignments = planner.allocate(created_tasks)
    dispatch_now, delayed = planner.resolve_collisions(assignments)

    # Navigate assigned robots
    client = RosClient()
    client.connect()

    results = []
    try:
        for assignment in dispatch_now:
            task = assignment.task
            robot_id = assignment.robot_id

            # Update fleet state
            manager.set_robot_status(robot_id, "navigating",
                                     goal_x=task.x, goal_y=task.y)
            task.status = TaskStatus.IN_PROGRESS

            # Send navigation goal
            goal = {
                "pose": {
                    "header": {"frame_id": "map"},
                    "pose": {
                        "position": {"x": task.x, "y": task.y, "z": 0.0},
                        "orientation": {
                            "x": 0.0, "y": 0.0,
                            "z": math.sin(task.theta / 2.0),
                            "w": math.cos(task.theta / 2.0),
                        },
                    },
                }
            }

            resp = client.send_goal(
                action=f"/{robot_id}/navigate_to_pose",
                action_type="nav2_msgs/action/NavigateToPose",
                goal=goal,
            )
            goal_id = resp["goal_id"]
            task.status = TaskStatus.IN_PROGRESS

            # Wait for result
            result = client.wait_for_result(
                f"/{robot_id}/navigate_to_pose", goal_id, timeout=timeout_per_task
            )

            success = result.get("success", False) if result else False
            planner.complete_task(task.task_id, success=success)

            results.append({
                "robot_id": robot_id,
                "task_id": task.task_id,
                "target": {"x": task.x, "y": task.y},
                "success": success,
                "cost": round(assignment.cost, 3),
                "distance": round(assignment.distance, 3),
            })

    finally:
        client.disconnect()

    succeeded = sum(1 for r in results if r["success"])
    return {
        "success": succeeded == len(dispatch_now),
        "dispatched": len(dispatch_now),
        "succeeded": succeeded,
        "failed": len(dispatch_now) - succeeded,
        "delayed_due_to_collision": len(delayed),
        "results": results,
        "delayed": [a.to_dict() for a in delayed],
    }


@mcp.tool()
def get_plan() -> dict:
    """
    Get the current coordination plan - all tasks, assignments, and fleet state.
    
    Returns:
        Dict with task queue, active assignments, fleet positions, and statistics.
    """
    planner = _get_planner()
    manager = _get_manager()

    plan = planner.get_plan()
    plan["fleet"] = manager.get_all_states()
    plan["available_robots"] = len(manager.get_available_robots())
    plan["total_robots"] = len(manager.robots)

    return plan


@mcp.tool()
def replan() -> dict:
    """
    Force re-allocation of all pending and failed tasks.
    Call when: a robot fails, priorities change, new robots come online.
    
    Returns:
        New allocation plan with updated assignments.
    """
    planner = _get_planner()

    assignments = planner.replan()
    dispatch_now, delayed = planner.resolve_collisions(assignments)

    return {
        "success": True,
        "action": "replanned",
        "new_assignments": len(assignments),
        "dispatch_now": [a.to_dict() for a in dispatch_now],
        "delayed": [a.to_dict() for a in delayed],
        "plan": planner.get_plan(),
    }


@mcp.tool()
def set_robot_priority(
    robot_id: str,
    priority: int,
) -> dict:
    """
    Set a robot's priority for task allocation and collision resolution.
    Higher priority robots get assigned urgent tasks first and win collision conflicts.
    
    Args:
        robot_id: Robot namespace (e.g. 'tb1').
        priority: Integer priority (higher = more important). Default is 0.
    
    Returns:
        Confirmation with updated priority.
    """
    manager = _get_manager()
    robot = manager.robots.get(robot_id)

    if robot is None:
        return {
            "success": False,
            "error": f"Unknown robot: {robot_id}",
        }

    old_priority = robot.priority
    manager.set_priority(robot_id, priority)

    return {
        "success": True,
        "robot_id": robot_id,
        "old_priority": old_priority,
        "new_priority": priority,
    }


@mcp.tool()
def configure_fleet(
    robot_ids: list[str] = None,
    groups: dict = None,
    collision_buffer: float = 0.5,
) -> dict:
    """
    Configure fleet composition and groups. Call once at startup or when fleet changes.
    
    Args:
        robot_ids: List of all robot namespaces. Default: ['tb1', 'tb2', 'tb3'].
        groups: Dict mapping group names to robot ID lists.
                Example: {"warehouse": ["tb1", "tb2"], "outdoor": ["tb3"]}
        collision_buffer: Minimum distance between robot paths (meters).
    
    Returns:
        Fleet configuration summary.
    """
    manager = _get_manager()

    if robot_ids is None:
        robot_ids = ["tb1", "tb2", "tb3"]

    # Restart manager with new config
    manager.stop()
    manager._instance = None
    new_manager = FleetStateManager.get_instance()
    new_manager.start(robot_ids=robot_ids, groups=groups)

    # Update planner
    global _manager, _planner
    _manager = new_manager
    _planner = TaskPlanner(_manager, collision_buffer=collision_buffer)

    return {
        "success": True,
        "fleet_size": len(robot_ids),
        "robots": robot_ids,
        "groups": groups or {"default": robot_ids},
        "collision_buffer": collision_buffer,
    }
