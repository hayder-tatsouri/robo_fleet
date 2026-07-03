#!/usr/bin/env python3
"""
Unit tests for new MCP tools - uses mocked WebSocket.
Run: pytest tests/ -v
"""

import json
import math
import sys
import pytest
from unittest.mock import patch, MagicMock

sys.path.insert(0, 'mcp_server')
from ros.ros_client import RosClient


@pytest.fixture
def mock_ws():
    with patch('ros.ros_client.websocket.create_connection') as mock_create:
        ws_mock = MagicMock()
        mock_create.return_value = ws_mock
        yield ws_mock


@pytest.fixture
def client(mock_ws):
    c = RosClient()
    c.connect()
    return c


# ─── MONITORING ───

class TestMonitoring:
    def test_get_position_returns_pose(self, client, mock_ws):
        pose_msg = {
            "op": "publish",
            "topic": "/tb1/amcl_pose",
            "msg": {
                "header": {"frame_id": "map"},
                "pose": {"pose": {
                    "position": {"x": 2.5, "y": -1.3, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
                }}
            }
        }
        mock_ws.recv.return_value = json.dumps(pose_msg)

        result = client.subscribe_once("/tb1/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped")
        pos = result["pose"]["pose"]["position"]
        assert pos["x"] == 2.5
        assert pos["y"] == -1.3

    def test_get_battery_returns_state(self, client, mock_ws):
        battery_msg = {
            "op": "publish",
            "topic": "/tb1/battery_state",
            "msg": {
                "percentage": 0.75,
                "voltage": 11.8,
                "current": -0.3,
            }
        }
        mock_ws.recv.return_value = json.dumps(battery_msg)

        result = client.subscribe_once("/tb1/battery_state", "sensor_msgs/msg/BatteryState")
        assert result["percentage"] == 0.75
        assert result["voltage"] == 11.8

    def test_fleet_status_multiple_robots(self, client, mock_ws):
        """Test subscribing to multiple robot topics."""
        responses = []
        for i, name in enumerate(["tb1", "tb2", "tb3"]):
            responses.append(json.dumps({
                "op": "publish",
                "topic": f"/{name}/amcl_pose",
                "msg": {"pose": {"pose": {
                    "position": {"x": float(i), "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }}}
            }))
        mock_ws.recv.side_effect = responses

        for name in ["tb1", "tb2", "tb3"]:
            result = client.subscribe_once(f"/{name}/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped")
            assert result is not None


# ─── CONTROL ───

class TestControl:
    def test_stop_sends_zero_velocity(self, client, mock_ws):
        client.publish(
            "/tb1/cmd_vel",
            "geometry_msgs/msg/TwistStamped",
            {"header": {"frame_id": "base_link"},
             "twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                       "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}
        )
        sent = json.loads(mock_ws.send.call_args[0][0])
        assert sent["msg"]["twist"]["linear"]["x"] == 0.0
        assert sent["msg"]["twist"]["angular"]["z"] == 0.0

    def test_emergency_stop_all_robots(self, client, mock_ws):
        for robot_id in ["tb1", "tb2", "tb3"]:
            client.publish(
                f"/{robot_id}/cmd_vel",
                "geometry_msgs/msg/TwistStamped",
                {"header": {"frame_id": "base_link"},
                 "twist": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}}
            )
        assert mock_ws.send.call_count >= 3


# ─── WAYPOINTS ───

class TestWaypoints:
    def test_waypoint_sends_sequential_goals(self, client, mock_ws):
        waypoints = [{"x": 1.0, "y": 0.0}, {"x": 2.0, "y": 1.0}]

        # Each goal gets a success result
        mock_ws.recv.return_value = json.dumps({
            "op": "action_result", "id": "goal_test1234", "values": {}, "status": 4
        })

        for wp in waypoints:
            theta = wp.get("theta", 0.0)
            goal = {
                "pose": {
                    "header": {"frame_id": "map"},
                    "pose": {
                        "position": {"x": wp["x"], "y": wp["y"], "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(theta/2), "w": math.cos(theta/2)}
                    }
                }
            }
            resp = client.send_goal("/tb1/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
            assert "goal_id" in resp


# ─── OBSTACLES ───

class TestObstacles:
    def test_laser_scan_detection(self, client, mock_ws):
        scan_msg = {
            "op": "publish",
            "topic": "/tb1/scan",
            "msg": {
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0175,
                "range_min": 0.12,
                "range_max": 10.0,
                "ranges": [3.0] * 180 + [0.3] * 10 + [3.0] * 170,
            }
        }
        mock_ws.recv.return_value = json.dumps(scan_msg)

        result = client.subscribe_once("/tb1/scan", "sensor_msgs/msg/LaserScan")
        ranges = result["ranges"]
        valid = [r for r in ranges if 0.12 <= r <= 10.0]
        closest = min(valid)
        assert closest == 0.3
        assert len(ranges) == 360

    def test_no_obstacles_detected(self, client, mock_ws):
        scan_msg = {
            "op": "publish",
            "topic": "/tb1/scan",
            "msg": {
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0175,
                "range_min": 0.12,
                "range_max": 10.0,
                "ranges": [5.0] * 360,  # All far away
            }
        }
        mock_ws.recv.return_value = json.dumps(scan_msg)

        result = client.subscribe_once("/tb1/scan", "sensor_msgs/msg/LaserScan")
        closest = min(result["ranges"])
        assert closest > 0.5  # No close obstacles


# ─── MAP VISUALIZATION ───

class TestMapVisualization:
    def test_multiple_robot_positions(self, client, mock_ws):
        positions = {"tb1": (1.0, 2.0), "tb2": (-1.0, 0.5), "tb3": (0.0, -1.0)}
        responses = []
        for name, (x, y) in positions.items():
            responses.append(json.dumps({
                "op": "publish",
                "topic": f"/{name}/amcl_pose",
                "msg": {"pose": {"pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }}}
            }))
        mock_ws.recv.side_effect = responses

        for name in positions:
            result = client.subscribe_once(f"/{name}/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped")
            assert result is not None


# ─── CONNECTION RESILIENCE ───

class TestResilience:
    def test_auto_reconnect_on_send_failure(self, mock_ws):
        with patch('ros.ros_client.websocket.create_connection') as mock_create:
            ws1 = MagicMock()
            ws2 = MagicMock()
            mock_create.side_effect = [ws1, ws2]

            client = RosClient(auto_reconnect=True)
            client.connect()

            # First send fails
            ws1.send.side_effect = Exception("connection lost")
            # Should reconnect and retry
            client.publish("/tb1/cmd_vel", "geometry_msgs/msg/Twist", {"linear": {"x": 0.5}})

            # Verify reconnection happened
            assert mock_create.call_count == 2

    def test_max_retries_exceeded(self):
        with patch('ros.ros_client.websocket.create_connection') as mock_create:
            mock_create.side_effect = Exception("connection refused")

            client = RosClient(auto_reconnect=True, max_retries=2)
            with pytest.raises(ConnectionError):
                client.connect()
