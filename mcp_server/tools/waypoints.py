import math
import time
from server import mcp
from ros.ros_client import RosClient


@mcp.tool()
def navigate_waypoints(
    robot_id: str,
    waypoints: list[dict],
    frame_id: str = "map",
    timeout_per_waypoint: float = 60.0,
) -> dict:
    """
    Navigate a robot through a sequence of waypoints.
    Args:
        robot_id: Robot namespace (e.g. 'pearlguard1').
        waypoints: List of dicts with keys: x, y, and optional theta (radians).
                   Example: [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0, "theta": 1.57}]
        frame_id: Coordinate frame (default: "map").
        timeout_per_waypoint: Max seconds per waypoint (default: 60).
    Returns:
        Dict with per-waypoint results and overall success.
    """
    if not waypoints:
        return {"success": False, "error": "No waypoints provided"}

    client = RosClient()
    client.connect()

    results = []
    total_start = time.time()

    try:
        for i, wp in enumerate(waypoints):
            x = wp.get("x", 0.0)
            y = wp.get("y", 0.0)
            theta = wp.get("theta", 0.0)

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

            action = f"/{robot_id}/navigate_to_pose"
            goal_resp = client.send_goal(
                action=action,
                action_type="nav2_msgs/action/NavigateToPose",
                goal=goal,
            )
            goal_id = goal_resp["goal_id"]

            wp_start = time.time()
            result = client.wait_for_result(
                action=action,
                goal_id=goal_id,
                timeout=timeout_per_waypoint,
            )

            wp_time = round(time.time() - wp_start, 2)
            wp_result = {
                "waypoint_index": i,
                "target": {"x": x, "y": y, "theta": theta},
                "success": result.get("success", False) if result else False,
                "time_seconds": wp_time,
                "goal_id": goal_id,
            }

            if result and not result.get("success"):
                wp_result["error"] = result.get("error", "navigation failed")

            results.append(wp_result)

            # Stop if a waypoint fails
            if not wp_result["success"]:
                break

    finally:
        client.disconnect()

    total_time = round(time.time() - total_start, 2)
    completed = sum(1 for r in results if r["success"])

    return {
        "success": completed == len(waypoints),
        "robot_id": robot_id,
        "waypoints_total": len(waypoints),
        "waypoints_completed": completed,
        "total_time_seconds": total_time,
        "results": results,
    }
