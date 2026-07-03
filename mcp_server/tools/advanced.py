"""
Advanced MCP Tools - Collision prediction, task queue, dashboard, optimal allocation.
"""

from server import mcp
from coordination.fleet_state import FleetStateManager
from coordination.collision_predictor import CollisionPredictor
from coordination.task_queue import TaskQueue
from coordination.dashboard_server import get_dashboard
from coordination.hungarian import assign_optimal


# ─── Global instances (lazy init) ───
_predictor = None
_task_queue = None


def _get_manager():
    manager = FleetStateManager.get_instance()
    if not manager._running:
        manager.start()
    return manager


def _get_predictor():
    global _predictor
    if _predictor is None:
        _predictor = CollisionPredictor(_get_manager())
    return _predictor


def _get_task_queue():
    global _task_queue
    if _task_queue is None:
        _task_queue = TaskQueue(_get_manager())
    return _task_queue


# ═══════════════════════════════════════════
# 1. COLLISION PREDICTION
# ═══════════════════════════════════════════

@mcp.tool()
def predict_collisions(
    buffer_distance: float = 0.4,
    time_horizon: float = 5.0,
) -> dict:
    """
    Predict potential collisions between all moving robots.
    Uses linear trajectory extrapolation to find time of closest approach.
    
    Args:
        buffer_distance: Minimum safe distance in meters (default: 0.4).
        time_horizon: How far ahead to predict in seconds (default: 5.0).
    
    Returns:
        Dict with collision risks sorted by severity, including resolution suggestions.
    """
    predictor = _get_predictor()
    predictor.buffer_distance = buffer_distance
    predictor.time_horizon = time_horizon

    risks = predictor.predict_all()

    return {
        "success": True,
        "collision_risks": len(risks),
        "critical": sum(1 for r in risks if r.severity == "critical"),
        "warnings": sum(1 for r in risks if r.severity == "warning"),
        "risks": [r.to_dict() for r in risks],
    }


# ═══════════════════════════════════════════
# 2. TASK QUEUE
# ═══════════════════════════════════════════

@mcp.tool()
def add_task_to_queue(
    x: float,
    y: float,
    theta: float = 0.0,
    priority: int = 0,
    group: str = None,
) -> dict:
    """
    Add a navigation task to the dispatch queue.
    Tasks are auto-dispatched to idle robots (if auto-dispatch is running).
    
    Args:
        x: Target X position.
        y: Target Y position.
        theta: Target orientation in radians.
        priority: Higher number = dispatched first (default: 0).
        group: Only dispatch to robots in this group (optional).
    
    Returns:
        Task details including queue position.
    """
    queue = _get_task_queue()
    result = queue.add(x=x, y=y, theta=theta, priority=priority, group=group)
    queue_state = queue.get_queue()
    result["queue_position"] = queue_state["pending_count"]
    result["auto_dispatch"] = queue_state["auto_dispatch"]
    return result


@mcp.tool()
def get_queue() -> dict:
    """
    Get the current task queue state.
    Returns:
        All tasks organized by status: pending, dispatched, completed, failed.
    """
    queue = _get_task_queue()
    return queue.get_queue()


@mcp.tool()
def clear_queue() -> dict:
    """
    Clear all pending tasks from the queue.
    Does not affect already-dispatched or completed tasks.
    Returns:
        Number of tasks removed.
    """
    queue = _get_task_queue()
    removed = queue.clear()
    return {"success": True, "removed": removed}


@mcp.tool()
def start_auto_dispatch() -> dict:
    """
    Start auto-dispatch: tasks are automatically sent to idle robots.
    Background thread checks every second for available robots and pending tasks.
    Returns:
        Status confirmation.
    """
    queue = _get_task_queue()
    return queue.start_auto_dispatch()


@mcp.tool()
def stop_auto_dispatch() -> dict:
    """
    Stop auto-dispatch. Tasks remain in queue but won't be sent automatically.
    Returns:
        Status confirmation.
    """
    queue = _get_task_queue()
    return queue.stop_auto_dispatch()


# ═══════════════════════════════════════════
# 3. DASHBOARD
# ═══════════════════════════════════════════

@mcp.tool()
def start_dashboard(port: int = 8080) -> dict:
    """
    Start the live fleet dashboard WebSocket server.
    Opens a WebSocket on the specified port that streams fleet state at 5Hz.
    Open dashboard/live_dashboard.html in a browser to view.
    
    Args:
        port: WebSocket port (default: 8080).
    
    Returns:
        Dashboard URL and connection info.
    """
    manager = _get_manager()
    dashboard = get_dashboard(manager, port=port)
    return dashboard.start()


@mcp.tool()
def stop_dashboard() -> dict:
    """Stop the live fleet dashboard server."""
    dashboard = get_dashboard()
    if dashboard:
        return dashboard.stop()
    return {"success": False, "error": "Dashboard not running"}


# ═══════════════════════════════════════════
# 6. HUNGARIAN OPTIMAL ALLOCATION
# ═══════════════════════════════════════════

@mcp.tool()
def assign_tasks_optimal(
    tasks: list[dict],
) -> dict:
    """
    Assign tasks to robots using the Hungarian algorithm (globally optimal).
    Falls back to greedy if scipy is not installed.
    
    Args:
        tasks: List of task dicts with keys: x, y, and optional task_id.
               Example: [{"x": 1.0, "y": 2.0}, {"x": -1.0, "y": 0.5}]
    
    Returns:
        Optimal assignments with cost comparison vs greedy approach.
    """
    manager = _get_manager()
    return assign_optimal(manager, tasks)
