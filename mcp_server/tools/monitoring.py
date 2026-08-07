import math
import json
from server import mcp
from ros.ros_client import RosClient


@mcp.tool()
def list_capabilities() -> dict:
    """
    List all available agents and their tools in the multi-agent fleet system.
    Returns a complete overview of what the system can do.
    """
    return {
        "system": "Robo_Fleet Multi-Agent System",
        "agents": [
            {
                "name": "navigation_agent",
                "description": "Moves robots to coordinates or waypoint sequences",
                "tools": ["navigate_to_pose(robot_id, x, y, theta)", "navigate_waypoints(robot_id, waypoints)"],
            },
            {
                "name": "monitoring_agent",
                "description": "Reports robot positions, battery, and fleet status",
                "tools": ["get_robot_position(robot_id)", "get_fleet_status(robot_ids)", "get_battery_level(robot_id)", "list_capabilities()"],
            },
            {
                "name": "control_agent",
                "description": "Stops robots immediately",
                "tools": ["stop_robot(robot_id)", "emergency_stop(robot_ids)"],
            },
            {
                "name": "collision_agent",
                "description": "Detects obstacles and predicts robot collisions",
                "tools": ["check_obstacles(robot_id)", "predict_collisions()"],
            },
            {
                "name": "planning_agent",
                "description": "Allocates tasks to robots optimally",
                "tools": ["assign_tasks(tasks)", "dispatch_tasks(tasks)", "get_plan()", "replan()", "set_robot_priority()", "configure_fleet()", "assign_tasks_optimal(tasks)"],
            },
            {
                "name": "queue_agent",
                "description": "Manages the dispatch task queue",
                "tools": ["add_task_to_queue()", "get_queue()", "clear_queue()", "start_auto_dispatch()", "stop_auto_dispatch()"],
            },
            {
                "name": "dashboard_agent",
                "description": "Starts/stops the live visualization",
                "tools": ["start_dashboard(port)", "stop_dashboard()"],
            },
            {
                "name": "natural_lang_agent",
                "description": "Manages named locations and sends nearest robot",
                "tools": ["list_locations()", "add_location()", "remove_location()", "go_to_location()", "send_nearest_to()"],
            },
            {
                "name": "map_viz_agent",
                "description": "Generates ASCII map of robot positions",
                "tools": ["get_map_with_robots()"],
            },
        ],
        "total_agents": 9,
        "total_tools": 30,
    }


@mcp.tool()
def get_robot_position(
    robot_id: str,
    timeout: float = 5.0,
) -> dict:
    """
    Get the current position of a specific robot.
    Args:
        robot_id: Robot namespace (e.g. 'pearlguard1').
        timeout: Max seconds to wait for pose data (default: 5).
    Returns:
        Dict with x, y, theta (radians), frame_id, or error message.
    """
    client = RosClient()
    client.connect()

    try:
        msg = client.subscribe_once(
            topic=f"/{robot_id}/odometry/filtered",
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
        robot_ids: List of robot namespaces. Defaults to ['pearlguard1', 'pearlguard2'].
        timeout: Max seconds to wait per robot (default: 3).
    Returns:
        Dict with fleet overview: list of robots with position, battery, status.
    """
    if robot_ids is None:
        robot_ids = ["pearlguard1", "pearlguard2"]

    client = RosClient()
    client.connect()

    fleet = []
    try:
        for robot_id in robot_ids:
            robot_info = {"robot_id": robot_id}

            # Get position
            msg = client.subscribe_once(
                topic=f"/{robot_id}/odometry/filtered",
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
        robot_id: Robot namespace (e.g. 'pearlguard1').
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
