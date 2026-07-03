#!/usr/bin/env python3
"""
Unit tests for RosClient - uses mocked WebSocket connections.
Run: pytest tests/ -v
"""

import json
import sys
import pytest
from unittest.mock import patch, MagicMock

sys.path.insert(0, 'mcp_server')
from ros.ros_client import RosClient


# ─────────────────────────────────────────
# FIXTURES
# ─────────────────────────────────────────

@pytest.fixture
def mock_ws():
    """Mock WebSocket connection."""
    with patch('ros.ros_client.websocket.create_connection') as mock_create:
        ws_mock = MagicMock()
        mock_create.return_value = ws_mock
        yield ws_mock


@pytest.fixture
def client(mock_ws):
    """Connected RosClient with mocked WebSocket."""
    c = RosClient(host="localhost", port=9090)
    c.connect()
    return c


# ─────────────────────────────────────────
# TEST: Connection
# ─────────────────────────────────────────

class TestConnection:
    def test_connect_creates_websocket(self, mock_ws):
        client = RosClient(host="192.168.1.10", port=9090)
        client.connect()
        assert client.ws is not None

    def test_disconnect_closes_websocket(self, client, mock_ws):
        client.disconnect()
        mock_ws.close.assert_called_once()
        assert client.ws is None

    def test_disconnect_when_not_connected(self):
        client = RosClient()
        client.disconnect()  # Should not raise


# ─────────────────────────────────────────
# TEST: Publish
# ─────────────────────────────────────────

class TestPublish:
    def test_publish_sends_correct_message(self, client, mock_ws):
        client.publish(
            topic="/tb1/cmd_vel",
            msg_type="geometry_msgs/msg/Twist",
            data={"linear": {"x": 0.5}, "angular": {"z": 0.1}}
        )

        sent = json.loads(mock_ws.send.call_args[0][0])
        assert sent["op"] == "publish"
        assert sent["topic"] == "/tb1/cmd_vel"
        assert sent["type"] == "geometry_msgs/msg/Twist"
        assert sent["msg"]["linear"]["x"] == 0.5

    def test_publish_multiple_robots(self, client, mock_ws):
        for robot in ["tb1", "tb2", "tb3"]:
            client.publish(
                topic=f"/{robot}/cmd_vel",
                msg_type="geometry_msgs/msg/TwistStamped",
                data={"twist": {"linear": {"x": 0.2}}}
            )
        assert mock_ws.send.call_count == 3


# ─────────────────────────────────────────
# TEST: Subscribe
# ─────────────────────────────────────────

class TestSubscribe:
    def test_subscribe_once_returns_message(self, client, mock_ws):
        # Simulate receiving a pose message
        pose_msg = {
            "op": "publish",
            "topic": "/tb1/amcl_pose",
            "msg": {
                "pose": {"pose": {"position": {"x": 1.5, "y": 2.3, "z": 0.0}}}
            }
        }
        mock_ws.recv.return_value = json.dumps(pose_msg)

        result = client.subscribe_once(
            topic="/tb1/amcl_pose",
            msg_type="geometry_msgs/msg/PoseWithCovarianceStamped",
            timeout=5.0
        )

        assert result is not None
        assert result["pose"]["pose"]["position"]["x"] == 1.5
        assert result["pose"]["pose"]["position"]["y"] == 2.3

    def test_subscribe_once_timeout_returns_none(self, client, mock_ws):
        import websocket as ws_module
        mock_ws.recv.side_effect = ws_module.WebSocketTimeoutException()

        result = client.subscribe_once(
            topic="/tb1/amcl_pose",
            msg_type="geometry_msgs/msg/PoseWithCovarianceStamped",
            timeout=1.0
        )

        assert result is None


# ─────────────────────────────────────────
# TEST: Action Goals
# ─────────────────────────────────────────

class TestActions:
    def test_send_goal_returns_goal_id(self, client, mock_ws):
        result = client.send_goal(
            action="/tb1/navigate_to_pose",
            action_type="nav2_msgs/action/NavigateToPose",
            goal={"pose": {"header": {"frame_id": "map"}}}
        )

        assert "goal_id" in result
        assert result["goal_id"].startswith("goal_")

        sent = json.loads(mock_ws.send.call_args[0][0])
        assert sent["op"] == "send_action_goal"
        assert sent["feedback"] is True

    def test_wait_for_result_success(self, client, mock_ws):
        goal_id = "goal_abc12345"
        success_msg = json.dumps({
            "op": "action_result",
            "id": goal_id,
            "action": "/tb1/navigate_to_pose",
            "values": {},
            "status": 4  # SUCCEEDED
        })
        mock_ws.recv.return_value = success_msg

        result = client.wait_for_result("/tb1/navigate_to_pose", goal_id, timeout=10.0)

        assert result["success"] is True
        assert result["status"] == 4
        assert result["goal_id"] == goal_id

    def test_wait_for_result_failure(self, client, mock_ws):
        goal_id = "goal_xyz99999"
        fail_msg = json.dumps({
            "op": "action_result",
            "id": goal_id,
            "action": "/tb1/navigate_to_pose",
            "values": {"error": "path blocked"},
            "status": 5  # ABORTED
        })
        mock_ws.recv.return_value = fail_msg

        result = client.wait_for_result("/tb1/navigate_to_pose", goal_id, timeout=10.0)

        assert result["success"] is False
        assert result["status"] == 5
        assert "path blocked" in result["error"]

    def test_cancel_action(self, client, mock_ws):
        result = client.cancel_action("/tb1/navigate_to_pose", "goal_abc123")

        assert result["success"] is True
        sent = json.loads(mock_ws.send.call_args[0][0])
        assert sent["op"] == "cancel_action_goal"


# ─────────────────────────────────────────
# TEST: Navigation Tool (integration)
# ─────────────────────────────────────────

class TestNavigationTool:
    @patch('ros.ros_client.websocket.create_connection')
    def test_navigate_to_pose_constructs_correct_goal(self, mock_create):
        """Test the navigation tool builds proper Nav2 goal geometry."""
        import math
        ws_mock = MagicMock()
        mock_create.return_value = ws_mock

        # Simulate successful navigation
        ws_mock.recv.return_value = json.dumps({
            "op": "action_result",
            "id": "goal_test1234",
            "values": {},
            "status": 4
        })

        # Import after patching
        sys.path.insert(0, 'mcp_server')
        from tools.navigation import navigate_to_pose

        # Patch the goal_id generation for predictability
        with patch('ros.ros_client.uuid.uuid4') as mock_uuid:
            mock_uuid.return_value = MagicMock(hex='test1234abcdef00')
            result = navigate_to_pose(
                robot_id="tb2",
                x=3.0,
                y=-1.5,
                theta=1.57,
                frame_id="map",
                timeout=20.0
            )

        # Verify the goal was sent correctly
        sent_calls = ws_mock.send.call_args_list
        goal_msg = json.loads(sent_calls[0][0][0])

        assert goal_msg["action"] == "/tb2/navigate_to_pose"
        assert goal_msg["action_type"] == "nav2_msgs/action/NavigateToPose"

        pose = goal_msg["args"]["pose"]["pose"]
        assert pose["position"]["x"] == 3.0
        assert pose["position"]["y"] == -1.5
        assert abs(pose["orientation"]["z"] - math.sin(1.57 / 2)) < 0.001
        assert abs(pose["orientation"]["w"] - math.cos(1.57 / 2)) < 0.001
