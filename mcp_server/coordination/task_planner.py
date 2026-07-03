"""
TaskPlanner - Greedy linear task allocation with battery-aware cost function
and zone-based collision avoidance.

Design:
  - Cost = distance * battery_penalty * busy_penalty
  - Collision = pairwise distance check with priority-based resolution
  - Re-planning on: task completion, failure, new task arrival
"""

import math
import time
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class TaskStatus(Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class Task:
    """A navigation task to be assigned to a robot."""
    task_id: str
    x: float
    y: float
    theta: float = 0.0
    priority: int = 0  # higher = more urgent
    status: TaskStatus = TaskStatus.PENDING
    assigned_robot: Optional[str] = None
    created_at: float = field(default_factory=time.time)
    completed_at: Optional[float] = None
    group: Optional[str] = None  # only assign to robots in this group

    def to_dict(self):
        return {
            "task_id": self.task_id,
            "target": {"x": self.x, "y": self.y, "theta": self.theta},
            "priority": self.priority,
            "status": self.status.value,
            "assigned_robot": self.assigned_robot,
            "age_seconds": round(time.time() - self.created_at, 1),
            "group": self.group,
        }


@dataclass
class Assignment:
    """A robot-task assignment with cost breakdown."""
    robot_id: str
    task: Task
    cost: float
    distance: float
    battery_penalty: float
    collision_risk: bool = False
    eta_seconds: float = 0.0

    def to_dict(self):
        return {
            "robot_id": self.robot_id,
            "task_id": self.task.task_id,
            "target": {"x": self.task.x, "y": self.task.y},
            "cost": round(self.cost, 3),
            "distance": round(self.distance, 3),
            "battery_penalty": round(self.battery_penalty, 3),
            "collision_risk": self.collision_risk,
            "eta_seconds": round(self.eta_seconds, 1),
        }


class TaskPlanner:
    """
    Greedy linear task allocation with collision avoidance.
    
    Algorithm:
      1. Sort tasks by priority (highest first)
      2. For each task, compute cost for all available robots
      3. Assign lowest-cost robot
      4. Check collision buffer before dispatching
      5. If collision risk: delay lower-priority robot
    """

    def __init__(self, fleet_manager, collision_buffer=0.5, robot_speed=0.5):
        """
        Args:
            fleet_manager: FleetStateManager instance
            collision_buffer: Minimum distance (m) between robot paths
            robot_speed: Average robot speed (m/s) for ETA estimation
        """
        self.fleet = fleet_manager
        self.collision_buffer = collision_buffer
        self.robot_speed = robot_speed
        self.tasks = {}  # task_id -> Task
        self.assignments = {}  # robot_id -> Assignment
        self._task_counter = 0
        self._lock = threading.Lock()

    def create_task(self, x, y, theta=0.0, priority=0, group=None, task_id=None):
        """Create a new task and add it to the queue."""
        with self._lock:
            if task_id is None:
                self._task_counter += 1
                task_id = f"task_{self._task_counter:04d}"

            task = Task(
                task_id=task_id,
                x=x, y=y, theta=theta,
                priority=priority,
                group=group,
            )
            self.tasks[task_id] = task
            return task

    def compute_cost(self, robot, task):
        """
        Compute assignment cost for a robot-task pair.
        
        cost = distance * (1 + battery_penalty) * (1 + busy_penalty)
        
        Battery penalty: exponential increase below 30%
        Busy penalty: 0 if idle, high if navigating
        """
        distance = robot.distance_to(task.x, task.y)

        # Battery penalty: exponential below 30%
        if robot.battery > 30:
            battery_penalty = 0.0
        elif robot.battery > 15:
            battery_penalty = (30 - robot.battery) / 30  # 0 to 0.5
        else:
            battery_penalty = 5.0  # Very high - avoid this robot

        # Busy penalty
        if robot.status == "idle":
            busy_penalty = 0.0
        elif robot.status == "navigating":
            busy_penalty = 2.0  # Can be assigned but costly
        else:
            busy_penalty = 10.0  # Error/offline - avoid

        cost = distance * (1 + battery_penalty) * (1 + busy_penalty)

        return cost, distance, battery_penalty

    def allocate(self, tasks=None):
        """
        Run greedy linear allocation on pending tasks.
        
        Returns list of Assignments sorted by priority.
        """
        with self._lock:
            # Get tasks to allocate
            if tasks is None:
                pending = [t for t in self.tasks.values() if t.status == TaskStatus.PENDING]
            else:
                pending = tasks

            # Sort by priority (highest first)
            pending.sort(key=lambda t: t.priority, reverse=True)

            # Get available robots
            available = list(self.fleet.get_available_robots())
            assignments = []

            for task in pending:
                if not available:
                    break

                # Filter by group if specified
                candidates = available
                if task.group:
                    candidates = [r for r in available if r.group == task.group]
                    if not candidates:
                        continue  # No robots in requested group

                # Find lowest cost robot
                best_robot = None
                best_cost = float('inf')
                best_distance = 0
                best_penalty = 0

                for robot in candidates:
                    cost, distance, penalty = self.compute_cost(robot, task)
                    if cost < best_cost:
                        best_cost = cost
                        best_robot = robot
                        best_distance = distance
                        best_penalty = penalty

                if best_robot is None:
                    continue

                # Create assignment
                eta = best_distance / self.robot_speed
                assignment = Assignment(
                    robot_id=best_robot.robot_id,
                    task=task,
                    cost=best_cost,
                    distance=best_distance,
                    battery_penalty=best_penalty,
                    eta_seconds=eta,
                )

                # Check collision with existing assignments
                assignment.collision_risk = self._check_collision(assignment, assignments)

                assignments.append(assignment)
                task.status = TaskStatus.ASSIGNED
                task.assigned_robot = best_robot.robot_id
                available.remove(best_robot)

            return assignments

    def _check_collision(self, new_assignment, existing_assignments):
        """
        Check if a new assignment's path conflicts with existing ones.
        Uses simple endpoint proximity check (zone-based).
        
        Returns True if collision risk detected.
        """
        for existing in existing_assignments:
            # Check if goals are too close
            dx = new_assignment.task.x - existing.task.x
            dy = new_assignment.task.y - existing.task.y
            goal_distance = math.sqrt(dx * dx + dy * dy)

            if goal_distance < self.collision_buffer:
                return True

            # Check if paths cross (simplified: check if robots pass near each other)
            # Robot A: current_pos -> goal_A
            # Robot B: current_pos -> goal_B
            robot_a = self.fleet.robots.get(new_assignment.robot_id)
            robot_b = self.fleet.robots.get(existing.robot_id)

            if robot_a and robot_b:
                # Check midpoint proximity (crude but effective for small fleets)
                mid_a_x = (robot_a.x + new_assignment.task.x) / 2
                mid_a_y = (robot_a.y + new_assignment.task.y) / 2
                mid_b_x = (robot_b.x + existing.task.x) / 2
                mid_b_y = (robot_b.y + existing.task.y) / 2

                mid_dist = math.sqrt((mid_a_x - mid_b_x)**2 + (mid_a_y - mid_b_y)**2)
                if mid_dist < self.collision_buffer:
                    return True

        return False

    def resolve_collisions(self, assignments):
        """
        Resolve collision risks by delaying lower-priority robots.
        Returns (dispatch_now, delayed) assignment lists.
        """
        dispatch_now = []
        delayed = []

        for assignment in assignments:
            if not assignment.collision_risk:
                dispatch_now.append(assignment)
            else:
                # Check priority against conflicting assignments
                conflicting = [
                    a for a in dispatch_now
                    if self._assignments_conflict(assignment, a)
                ]

                if not conflicting:
                    dispatch_now.append(assignment)
                else:
                    # Compare priorities
                    robot = self.fleet.robots.get(assignment.robot_id)
                    highest_conflict_priority = max(
                        self.fleet.robots.get(a.robot_id).priority
                        for a in conflicting
                        if self.fleet.robots.get(a.robot_id)
                    )

                    if robot and robot.priority > highest_conflict_priority:
                        # This robot has higher priority - dispatch it, delay others
                        dispatch_now.append(assignment)
                        for c in conflicting:
                            dispatch_now.remove(c)
                            delayed.append(c)
                    else:
                        delayed.append(assignment)

        return dispatch_now, delayed

    def _assignments_conflict(self, a1, a2):
        """Check if two assignments have collision risk."""
        dx = a1.task.x - a2.task.x
        dy = a1.task.y - a2.task.y
        return math.sqrt(dx*dx + dy*dy) < self.collision_buffer

    def complete_task(self, task_id, success=True):
        """Mark a task as completed or failed."""
        with self._lock:
            task = self.tasks.get(task_id)
            if task:
                task.status = TaskStatus.COMPLETED if success else TaskStatus.FAILED
                task.completed_at = time.time()

                # Free the robot
                if task.assigned_robot:
                    self.fleet.set_robot_status(task.assigned_robot, "idle")
                    if task.assigned_robot in self.assignments:
                        del self.assignments[task.assigned_robot]

    def replan(self):
        """
        Force re-allocation of all pending/failed tasks.
        Call when: robot failure, new robots added, priorities changed.
        """
        with self._lock:
            # Reset failed tasks to pending
            for task in self.tasks.values():
                if task.status in (TaskStatus.FAILED, TaskStatus.ASSIGNED):
                    task.status = TaskStatus.PENDING
                    task.assigned_robot = None

        return self.allocate()

    def get_plan(self):
        """Get current plan state."""
        return {
            "tasks": {tid: t.to_dict() for tid, t in self.tasks.items()},
            "assignments": {rid: a.to_dict() for rid, a in self.assignments.items()},
            "pending": sum(1 for t in self.tasks.values() if t.status == TaskStatus.PENDING),
            "in_progress": sum(1 for t in self.tasks.values() if t.status == TaskStatus.IN_PROGRESS),
            "completed": sum(1 for t in self.tasks.values() if t.status == TaskStatus.COMPLETED),
            "failed": sum(1 for t in self.tasks.values() if t.status == TaskStatus.FAILED),
        }

    def clear_completed(self):
        """Remove completed/cancelled tasks from memory."""
        with self._lock:
            to_remove = [
                tid for tid, t in self.tasks.items()
                if t.status in (TaskStatus.COMPLETED, TaskStatus.CANCELLED)
            ]
            for tid in to_remove:
                del self.tasks[tid]
            return len(to_remove)
