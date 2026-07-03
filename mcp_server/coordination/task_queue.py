"""
TaskQueue - FIFO queue with priority override and auto-dispatch.
Background thread monitors fleet state and dispatches tasks
when robots become idle.
"""

import math
import time
import json
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Callable


@dataclass(order=True)
class QueuedTask:
    """A task in the dispatch queue."""
    priority: int  # Higher = more urgent (negated for min-heap behavior)
    created_at: float = field(compare=False)
    task_id: str = field(compare=False)
    x: float = field(compare=False)
    y: float = field(compare=False)
    theta: float = field(compare=False, default=0.0)
    group: Optional[str] = field(compare=False, default=None)
    status: str = field(compare=False, default="pending")  # pending, dispatched, completed, failed
    assigned_robot: Optional[str] = field(compare=False, default=None)

    def to_dict(self):
        return {
            "task_id": self.task_id,
            "priority": -self.priority,  # Show positive priority to user
            "target": {"x": self.x, "y": self.y, "theta": self.theta},
            "group": self.group,
            "status": self.status,
            "assigned_robot": self.assigned_robot,
            "age_seconds": round(time.time() - self.created_at, 1),
        }


class TaskQueue:
    """
    Priority queue with auto-dispatch capability.
    
    Usage:
        queue = TaskQueue(fleet_manager, dispatch_callback)
        queue.add(x=2.0, y=3.0, priority=1)
        queue.start_auto_dispatch()  # Background thread dispatches when robots free
    """

    def __init__(self, fleet_manager, dispatch_fn=None):
        """
        Args:
            fleet_manager: FleetStateManager instance
            dispatch_fn: Callback(robot_id, task) -> bool. Called to dispatch.
                         If None, uses default RosClient navigation.
        """
        self.fleet = fleet_manager
        self.dispatch_fn = dispatch_fn or self._default_dispatch

        self._queue = []  # Sorted list (priority queue via heapq)
        self._all_tasks = {}  # task_id -> QueuedTask
        self._lock = threading.Lock()
        self._counter = 0

        self._auto_dispatch = False
        self._dispatch_thread = None
        self._dispatch_interval = 1.0  # Check every second

    def add(self, x, y, theta=0.0, priority=0, group=None, task_id=None):
        """Add a task to the queue. Higher priority = dispatched first."""
        with self._lock:
            if task_id is None:
                self._counter += 1
                task_id = f"q_task_{self._counter:04d}"

            task = QueuedTask(
                priority=-priority,  # Negate for min-heap (highest priority first)
                created_at=time.time(),
                task_id=task_id,
                x=x, y=y, theta=theta,
                group=group,
            )

            import heapq
            heapq.heappush(self._queue, task)
            self._all_tasks[task_id] = task

        return task.to_dict()

    def peek(self):
        """Look at next task without removing."""
        with self._lock:
            pending = [t for t in self._queue if t.status == "pending"]
            return pending[0].to_dict() if pending else None

    def get_queue(self):
        """Get all tasks in queue order."""
        with self._lock:
            pending = sorted(
                [t for t in self._all_tasks.values() if t.status == "pending"],
                key=lambda t: t.priority
            )
            dispatched = [t for t in self._all_tasks.values() if t.status == "dispatched"]
            completed = [t for t in self._all_tasks.values() if t.status == "completed"]
            failed = [t for t in self._all_tasks.values() if t.status == "failed"]

            return {
                "pending": [t.to_dict() for t in pending],
                "dispatched": [t.to_dict() for t in dispatched],
                "completed": [t.to_dict() for t in completed],
                "failed": [t.to_dict() for t in failed],
                "total": len(self._all_tasks),
                "pending_count": len(pending),
                "auto_dispatch": self._auto_dispatch,
            }

    def clear(self):
        """Clear all pending tasks."""
        with self._lock:
            removed = 0
            for task in list(self._all_tasks.values()):
                if task.status == "pending":
                    task.status = "cancelled"
                    removed += 1
            self._queue = [t for t in self._queue if t.status == "pending"]
            return removed

    def start_auto_dispatch(self):
        """Start background auto-dispatch thread."""
        if self._auto_dispatch:
            return {"status": "already_running"}

        self._auto_dispatch = True
        self._dispatch_thread = threading.Thread(target=self._auto_dispatch_loop, daemon=True)
        self._dispatch_thread.start()
        return {"status": "started"}

    def stop_auto_dispatch(self):
        """Stop background auto-dispatch."""
        self._auto_dispatch = False
        return {"status": "stopped"}

    def _auto_dispatch_loop(self):
        """Background thread: dispatch pending tasks to idle robots."""
        while self._auto_dispatch:
            try:
                self._try_dispatch()
            except Exception:
                pass
            time.sleep(self._dispatch_interval)

    def _try_dispatch(self):
        """Try to dispatch the next pending task to an available robot."""
        with self._lock:
            # Find next pending task
            pending = sorted(
                [t for t in self._all_tasks.values() if t.status == "pending"],
                key=lambda t: t.priority
            )
            if not pending:
                return

            # Find available robots
            available = self.fleet.get_available_robots()
            if not available:
                return

            task = pending[0]

            # Filter by group if specified
            candidates = available
            if task.group:
                candidates = [r for r in available if r.group == task.group]
                if not candidates:
                    return

            # Find nearest robot
            best_robot = min(candidates, key=lambda r: r.distance_to(task.x, task.y))

            # Mark as dispatched
            task.status = "dispatched"
            task.assigned_robot = best_robot.robot_id

        # Dispatch (outside lock)
        success = self.dispatch_fn(best_robot.robot_id, task)

        with self._lock:
            if success:
                task.status = "completed"
            else:
                task.status = "failed"

        # Update fleet state
        if success:
            self.fleet.set_robot_status(best_robot.robot_id, "idle")
        else:
            self.fleet.set_robot_status(best_robot.robot_id, "idle")

    def _default_dispatch(self, robot_id, task):
        """Default dispatch: send Nav2 goal via RosClient."""
        try:
            from ros.ros_client import RosClient
            client = RosClient()
            client.connect()

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
                f"/{robot_id}/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose",
                goal
            )
            result = client.wait_for_result(
                f"/{robot_id}/navigate_to_pose",
                resp["goal_id"],
                timeout=30.0
            )
            client.disconnect()
            return result and result.get("success", False)
        except Exception:
            return False

    def mark_completed(self, task_id, success=True):
        """Externally mark a task as completed/failed."""
        with self._lock:
            task = self._all_tasks.get(task_id)
            if task:
                task.status = "completed" if success else "failed"
