import math
from server import mcp
from ros.ros_client import RosClient


@mcp.tool()
def stop_robot(
    robot_id: str,
) -> dict:
    """
    Stop a specific robot immediately. Publishes zero velocity and cancels any active navigation goal.
    Args:
        robot_id: Robot namespace (e.g. 'tb1').
    Returns:
        Dict with success status.
    """
    client = RosClient()
    client.connect()

    try:
        # Publish zero velocity
        client.publish(
            topic=f"/{robot_id}/cmd_vel",
            msg_type="geometry_msgs/msg/TwistStamped",
            data={
                "header": {"frame_id": "base_link"},
                "twist": {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
            }
        )

        # Cancel any active navigation
        client.cancel_action(
            action=f"/{robot_id}/navigate_to_pose",
            goal_id="*"  # Cancel all
        )

        return {
            "success": True,
            "robot_id": robot_id,
            "action": "stopped",
            "message": f"{robot_id} stopped and navigation cancelled",
        }
    finally:
        client.disconnect()


@mcp.tool()
def emergency_stop(
    robot_ids: list[str] = None,
) -> dict:
    """
    Emergency stop ALL robots in the fleet. Immediately halts all motion.
    Args:
        robot_ids: List of robot namespaces. Defaults to ['tb1', 'tb2', 'tb3'].
    Returns:
        Dict with per-robot stop confirmation.
    """
    if robot_ids is None:
        robot_ids = ["tb1", "tb2", "tb3"]

    client = RosClient()
    client.connect()

    results = []
    try:
        for robot_id in robot_ids:
            # Zero velocity
            client.publish(
                topic=f"/{robot_id}/cmd_vel",
                msg_type="geometry_msgs/msg/TwistStamped",
                data={
                    "header": {"frame_id": "base_link"},
                    "twist": {
                        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                    }
                }
            )

            # Cancel navigation
            client.cancel_action(
                action=f"/{robot_id}/navigate_to_pose",
                goal_id="*"
            )

            results.append({"robot_id": robot_id, "stopped": True})

    finally:
        client.disconnect()

    return {
        "success": True,
        "action": "emergency_stop",
        "robots_stopped": len(results),
        "results": results,
        "message": f"All {len(results)} robots stopped immediately",
    }
