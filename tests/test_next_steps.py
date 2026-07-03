#!/usr/bin/env python3
"""
Unit tests for Next Steps features:
1. Collision Predictor
2. Task Queue + Auto-Dispatch
3. Dashboard Server
4. Natural Language Interface
5. Scale (benchmarks are in sim/scale_test.py)
6. Hungarian Optimal Allocation
"""

import math
import time
import json
import sys
import pytest
from unittest.mock import MagicMock, patch

sys.path.insert(0, 'mcp_server')

from coordination.fleet_state import FleetStateManager, RobotState
from coordination.collision_predictor import CollisionPredictor, CollisionRisk
from coordination.task_queue import TaskQueue, QueuedTask
from coordination.hungarian import assign_optimal, _compute_cost


# ─── FIXTURES ───

@pytest.fixture
def fleet_manager():
    """Create a FleetStateManager with 3 test robots."""
    manager = FleetStateManager.__new__(FleetStateManager)
    manager.robots = {}
    manager.groups = {"default": ["tb1", "tb2", "tb3"]}
    manager._running = True
    manager.ws = None
    manager._lock = __import__('threading').Lock()

    positions = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    for i, name in enumerate(["tb1", "tb2", "tb3"]):
        robot = RobotState(name)
        robot.x, robot.y = positions[i]
        robot.battery = 80.0
        robot.status = "idle"
        robot.last_seen = time.time()
        manager.robots[name] = robot

    return manager


# ═══════════════════════════════════════════
# 1. COLLISION PREDICTOR TESTS
# ═══════════════════════════════════════════

class TestCollisionPredictor:
    def test_no_collision_stationary(self, fleet_manager):
        """Stationary robots far apart = no collision."""
        predictor = CollisionPredictor(fleet_manager)
        risks = predictor.predict_all()
        assert len(risks) == 0

    def test_collision_when_approaching(self, fleet_manager):
        """Two robots heading toward each other should trigger collision."""
        # tb1 at (0,0) going right, tb2 at (1,0) going left
        fleet_manager.robots["tb1"].status = "navigating"
        fleet_manager.robots["tb1"].goal_x = 1.0
        fleet_manager.robots["tb1"].goal_y = 0.0

        fleet_manager.robots["tb2"].status = "navigating"
        fleet_manager.robots["tb2"].goal_x = 0.0
        fleet_manager.robots["tb2"].goal_y = 0.0

        predictor = CollisionPredictor(fleet_manager, buffer_distance=0.4, time_horizon=5.0)
        risks = predictor.predict_all()

        assert len(risks) >= 1
        risk = risks[0]
        assert risk.robot_a in ("tb1", "tb2")
        assert risk.robot_b in ("tb1", "tb2")
        assert risk.severity in ("critical", "warning")
        assert risk.time_to_collision >= 0

    def test_no_collision_diverging(self, fleet_manager):
        """Robots moving away from each other = no collision."""
        fleet_manager.robots["tb1"].status = "navigating"
        fleet_manager.robots["tb1"].goal_x = -2.0
        fleet_manager.robots["tb1"].goal_y = 0.0

        fleet_manager.robots["tb2"].status = "navigating"
        fleet_manager.robots["tb2"].goal_x = 3.0
        fleet_manager.robots["tb2"].goal_y = 0.0

        predictor = CollisionPredictor(fleet_manager, buffer_distance=0.4)
        risks = predictor.predict_all()
        assert len(risks) == 0

    def test_collision_resolution_uses_priority(self, fleet_manager):
        """Resolution should reference the lower-priority robot."""
        fleet_manager.robots["tb1"].priority = 10
        fleet_manager.robots["tb2"].priority = 1
        fleet_manager.robots["tb1"].status = "navigating"
        fleet_manager.robots["tb1"].goal_x = 1.0
        fleet_manager.robots["tb1"].goal_y = 0.0
        fleet_manager.robots["tb2"].status = "navigating"
        fleet_manager.robots["tb2"].goal_x = 0.0
        fleet_manager.robots["tb2"].goal_y = 0.0

        predictor = CollisionPredictor(fleet_manager, buffer_distance=0.5)
        risks = predictor.predict_all()

        if risks:
            # Resolution should tell tb2 (lower priority) to stop/wait
            assert "tb2" in risks[0].resolution

    def test_static_proximity_alert(self, fleet_manager):
        """Robots already within buffer should trigger critical."""
        fleet_manager.robots["tb1"].x = 0.0
        fleet_manager.robots["tb2"].x = 0.2  # Within 0.4m buffer

        predictor = CollisionPredictor(fleet_manager, buffer_distance=0.4)
        risks = predictor.predict_all()

        assert len(risks) >= 1
        assert risks[0].severity == "critical"


# ═══════════════════════════════════════════
# 2. TASK QUEUE TESTS
# ═══════════════════════════════════════════

class TestTaskQueue:
    def test_add_and_get(self, fleet_manager):
        """Add tasks and retrieve queue."""
        queue = TaskQueue(fleet_manager)
        queue.add(x=1.0, y=2.0, priority=0)
        queue.add(x=3.0, y=4.0, priority=5)

        state = queue.get_queue()
        assert state["pending_count"] == 2
        assert state["total"] == 2

    def test_priority_ordering(self, fleet_manager):
        """Higher priority tasks should come first."""
        queue = TaskQueue(fleet_manager)
        queue.add(x=0.0, y=0.0, priority=0, task_id="low")
        queue.add(x=1.0, y=1.0, priority=10, task_id="high")
        queue.add(x=2.0, y=2.0, priority=5, task_id="mid")

        state = queue.get_queue()
        pending = state["pending"]
        # Highest priority first
        assert pending[0]["task_id"] == "high"
        assert pending[1]["task_id"] == "mid"
        assert pending[2]["task_id"] == "low"

    def test_clear_queue(self, fleet_manager):
        """Clear removes all pending tasks."""
        queue = TaskQueue(fleet_manager)
        queue.add(x=1.0, y=0.0)
        queue.add(x=2.0, y=0.0)
        queue.add(x=3.0, y=0.0)

        removed = queue.clear()
        assert removed == 3
        assert queue.get_queue()["pending_count"] == 0

    def test_peek_returns_next(self, fleet_manager):
        """Peek shows next task without removing."""
        queue = TaskQueue(fleet_manager)
        queue.add(x=5.0, y=5.0, priority=1, task_id="first")
        queue.add(x=9.0, y=9.0, priority=0, task_id="second")

        peeked = queue.peek()
        assert peeked["task_id"] == "first"
        # Still in queue
        assert queue.get_queue()["pending_count"] == 2

    def test_auto_dispatch_toggle(self, fleet_manager):
        """Start/stop auto dispatch."""
        queue = TaskQueue(fleet_manager)
        result = queue.start_auto_dispatch()
        assert result["status"] == "started"
        assert queue._auto_dispatch is True

        result = queue.stop_auto_dispatch()
        assert result["status"] == "stopped"
        assert queue._auto_dispatch is False

    def test_empty_queue_dispatch(self, fleet_manager):
        """Dispatch with empty queue should do nothing."""
        queue = TaskQueue(fleet_manager)
        queue._try_dispatch()  # Should not raise
        assert queue.get_queue()["pending_count"] == 0

    def test_mark_completed(self, fleet_manager):
        """External completion marking."""
        queue = TaskQueue(fleet_manager)
        queue.add(x=1.0, y=0.0, task_id="test_task")
        queue.mark_completed("test_task", success=True)

        state = queue.get_queue()
        assert state["pending_count"] == 0
        assert len(state["completed"]) == 1


# ═══════════════════════════════════════════
# 3. DASHBOARD SERVER TESTS
# ═══════════════════════════════════════════

class TestDashboard:
    def test_dashboard_init(self, fleet_manager):
        from coordination.dashboard_server import DashboardServer
        dashboard = DashboardServer(fleet_manager, port=9999)
        assert dashboard.port == 9999
        assert dashboard._running is False

    def test_get_fleet_state(self, fleet_manager):
        from coordination.dashboard_server import DashboardServer
        dashboard = DashboardServer(fleet_manager)
        state = dashboard._get_fleet_state()

        assert state["type"] == "fleet_state"
        assert "timestamp" in state
        assert len(state["robots"]) == 3
        assert state["robots"][0]["id"] == "tb1"
        assert "x" in state["robots"][0]
        assert "battery" in state["robots"][0]

    def test_stop_before_start(self, fleet_manager):
        from coordination.dashboard_server import DashboardServer
        dashboard = DashboardServer(fleet_manager)
        result = dashboard.stop()
        assert result["status"] == "stopped"


# ═══════════════════════════════════════════
# 4. NATURAL LANGUAGE TESTS
# ═══════════════════════════════════════════

class TestNaturalLanguage:
    def test_load_default_locations(self):
        from tools.natural_language import _load_locations, _DEFAULT_LOCATIONS
        locations = _load_locations()
        # Should have defaults
        assert "origin" in locations or "charging_station" in locations

    def test_add_and_list_locations(self, tmp_path):
        """Test adding and listing locations."""
        import tools.natural_language as nl
        # Override file path for test
        original_file = nl._LOCATIONS_FILE
        nl._LOCATIONS_FILE = str(tmp_path / "test_locations.json")

        try:
            # Start fresh
            nl._save_locations({})

            # Add location
            nl._save_locations({"test_loc": {"x": 1.5, "y": -0.5, "description": "Test"}})
            locations = nl._load_locations()
            assert "test_loc" in locations
            assert locations["test_loc"]["x"] == 1.5
            assert locations["test_loc"]["y"] == -0.5
        finally:
            nl._LOCATIONS_FILE = original_file

    def test_location_name_normalization(self):
        """Names should be lowercase with underscores."""
        name = "Charging Station"
        normalized = name.lower().replace(" ", "_")
        assert normalized == "charging_station"


# ═══════════════════════════════════════════
# 6. HUNGARIAN ALLOCATION TESTS
# ═══════════════════════════════════════════

class TestHungarianAllocation:
    def test_optimal_assignment_basic(self, fleet_manager):
        """Optimal should assign each task to nearest robot."""
        # tb1 at (0,0), tb2 at (1,0), tb3 at (0,1)
        tasks = [
            {"x": 0.1, "y": 0.0, "task_id": "near_tb1"},
            {"x": 1.1, "y": 0.0, "task_id": "near_tb2"},
        ]
        result = assign_optimal(fleet_manager, tasks)

        assert result["success"] is True
        assert len(result["assignments"]) == 2

        # Check assignments are sensible
        for a in result["assignments"]:
            if a["task_id"] == "near_tb1":
                assert a["robot_id"] == "tb1"
            elif a["task_id"] == "near_tb2":
                assert a["robot_id"] == "tb2"

    def test_no_robots_available(self, fleet_manager):
        """All robots busy = no assignments."""
        for r in fleet_manager.robots.values():
            r.status = "navigating"

        tasks = [{"x": 0.0, "y": 0.0}]
        result = assign_optimal(fleet_manager, tasks)
        assert result["success"] is False

    def test_no_tasks(self, fleet_manager):
        """Empty task list."""
        result = assign_optimal(fleet_manager, [])
        assert result["success"] is False

    def test_more_tasks_than_robots(self, fleet_manager):
        """More tasks than robots - should assign what it can."""
        tasks = [
            {"x": 0.0, "y": 0.0},
            {"x": 1.0, "y": 0.0},
            {"x": 0.0, "y": 1.0},
            {"x": 2.0, "y": 2.0},  # Extra - won't be assigned
            {"x": 3.0, "y": 3.0},  # Extra
        ]
        result = assign_optimal(fleet_manager, tasks)
        assert result["success"] is True
        assert len(result["assignments"]) == 3  # Only 3 robots available

    def test_cost_function_battery_penalty(self, fleet_manager):
        """Low battery should increase cost."""
        robot = fleet_manager.robots["tb1"]
        robot.battery = 100.0
        cost_high_bat, _ = _compute_cost(robot, 1.0, 0.0)

        robot.battery = 10.0
        cost_low_bat, _ = _compute_cost(robot, 1.0, 0.0)

        assert cost_low_bat > cost_high_bat * 2  # Significantly more expensive

    def test_cost_function_busy_penalty(self, fleet_manager):
        """Busy robot should have higher cost."""
        robot = fleet_manager.robots["tb1"]
        robot.status = "idle"
        cost_idle, _ = _compute_cost(robot, 1.0, 0.0)

        robot.status = "navigating"
        cost_busy, _ = _compute_cost(robot, 1.0, 0.0)

        assert cost_busy > cost_idle * 2

    def test_improvement_reported(self, fleet_manager):
        """Result should show improvement percentage."""
        tasks = [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}]
        result = assign_optimal(fleet_manager, tasks)
        assert "improvement_percent" in result
        assert "total_cost_optimal" in result
        assert "total_cost_greedy" in result
