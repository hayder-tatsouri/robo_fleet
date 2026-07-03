import math
from server import mcp
from ros.ros_client import RosClient

@mcp.tool()
def navigate_to_pose(
    robot_id: str,
    x: float,
    y: float,
    theta: float = 0.0,
    frame_id: str = "map",
    timeout: float = 30.0,
) -> dict:
    """
    Navigate a specific robot to a target 2D pose using Nav2.
    Args:
        robot_id: Robot namespace (e.g. 'tb1' or 'tb3').
        x: Target X position in meters.
        y: Target Y position in meters.
        theta: Target yaw orientation in radians (default: 0).
        frame_id: Coordinate frame (default: "map").
        timeout: Max seconds to wait for navigation completion (default: 30).
    Returns:
        Dict with keys: success (bool), status (int), goal_id (str).
    """
    goal = {
        "pose": {
            "header": {"frame_id": frame_id},
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": math.sin(theta / 2.0),
                    "w": math.cos(theta / 2.0),
                },
            },
        }
    }

    client = RosClient()
    client.connect()
    
    try:
        action = f"/{robot_id}/navigate_to_pose"
        goal_resp = client.send_goal(
            action=action,
            action_type="nav2_msgs/action/NavigateToPose",
            goal=goal,
        )
        goal_id = goal_resp["goal_id"]
        result = client.wait_for_result(
            action=action,
            goal_id=goal_id,
            timeout=timeout,
        )
        return result
    finally:
        client.disconnect()
