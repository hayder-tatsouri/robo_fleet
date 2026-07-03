import math
from server import mcp
from ros.ros_client import RosClient


@mcp.tool()
def get_robot_position(
    robot_id: str,
    timeout: float = 5.0,
) -> dict:
    """
    Get the current position of a specific robot.
    Args:
        robot_id: Robot namespace (e.g. 'tb1', 'tb2', 'tb3').
        timeout: Max seconds to wait for pose data (default: 5).
    Returns:
        Dict with x, y, theta (radians), frame_id, or error message.
    """
    client = RosClient()
    client.connect()

    try:
        msg = client.subscribe_once(
            topic=f"/{robot_id}/amcl_pose",
            msg_type="geometry_msgs/msg/PoseWithCovarianceStamped",
            timeout=timeout,
        )

        if msg is None:
            return {
                "success": False,
                "error": f"No pose received from {robot_id} (timeout after {timeout}s)",
                "robot_id": robot_id,
            }

        pose = msg["pose"]["pose"]
        position = pose["position"]
        orientation = pose["orientation"]

        # Extract yaw from quaternion
        z = orientation.get("z", 0.0)
        w = orientation.get("w", 1.0)
        theta = 2 * math.atan2(z, w)

        return {
            "success": True,
            "robot_id": robot_id,
            "x": round(position["x"], 4),
            "y": round(position["y"], 4),
            "theta": round(theta, 4),
            "frame_id": msg.get("header", {}).get("frame_id", "map"),
        }
    finally:
        client.disconnect()


@mcp.tool()
def get_fleet_status(
    robot_ids: list[str] = None,
    timeout: float = 3.0,
) -> dict:
    """
    Get status of all robots in the fleet (position + battery + status).
    Args:
        robot_ids: List of robot namespaces. Defaults to ['tb1', 'tb2', 'tb3'].
        timeout: Max seconds to wait per robot (default: 3).
    Returns:
        Dict with fleet overview: list of robots with position, battery, status.
    """
    if robot_ids is None:
        robot_ids = ["tb1", "tb2", "tb3"]

    client = RosClient()
    client.connect()

    fleet = []
    try:
        for robot_id in robot_ids:
            robot_info = {"robot_id": robot_id}

            # Get position
            msg = client.subscribe_once(
                topic=f"/{robot_id}/amcl_pose",
                msg_type="geometry_msgs/msg/PoseWithCovarianceStamped",
                timeout=timeout,
            )

            if msg:
                pose = msg["pose"]["pose"]
                position = pose["position"]
                orientation = pose["orientation"]
                z = orientation.get("z", 0.0)
                w = orientation.get("w", 1.0)
                theta = 2 * math.atan2(z, w)

                robot_info["position"] = {
                    "x": round(position["x"], 4),
                    "y": round(position["y"], 4),
                    "theta": round(theta, 4),
                }
                robot_info["online"] = True
            else:
                robot_info["position"] = None
                robot_info["online"] = False

            # Get battery
            battery_msg = client.subscribe_once(
                topic=f"/{robot_id}/battery_state",
                msg_type="sensor_msgs/msg/BatteryState",
                timeout=1.0,
            )

            if battery_msg:
                robot_info["battery_percent"] = round(battery_msg.get("percentage", 0) * 100, 1)
            else:
                robot_info["battery_percent"] = None

            fleet.append(robot_info)

    finally:
        client.disconnect()

    online_count = sum(1 for r in fleet if r.get("online"))
    return {
        "success": True,
        "fleet_size": len(fleet),
        "online": online_count,
        "offline": len(fleet) - online_count,
        "robots": fleet,
    }


@mcp.tool()
def get_battery_level(
    robot_id: str,
    timeout: float = 3.0,
) -> dict:
    """
    Get the battery level of a specific robot.
    Args:
        robot_id: Robot namespace (e.g. 'tb1').
        timeout: Max seconds to wait for battery data (default: 3).
    Returns:
        Dict with battery percentage, voltage, charging status.
    """
    client = RosClient()
    client.connect()

    try:
        msg = client.subscribe_once(
            topic=f"/{robot_id}/battery_state",
            msg_type="sensor_msgs/msg/BatteryState",
            timeout=timeout,
        )

        if msg is None:
            return {
                "success": False,
                "error": f"No battery data from {robot_id} (timeout)",
                "robot_id": robot_id,
            }

        percentage = msg.get("percentage", 0)
        voltage = msg.get("voltage", 0)
        current = msg.get("current", 0)

        # Determine status
        if current > 0:
            status = "charging"
        elif percentage < 0.15:
            status = "critical"
        elif percentage < 0.30:
            status = "low"
        else:
            status = "normal"

        return {
            "success": True,
            "robot_id": robot_id,
            "battery_percent": round(percentage * 100, 1),
            "voltage": round(voltage, 2),
            "current": round(current, 3),
            "status": status,
            "alert": percentage < 0.15,
        }
    finally:
        client.disconnect()
