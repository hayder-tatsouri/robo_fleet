import math
import json
from server import mcp
from ros.ros_client import RosClient


@mcp.tool()
def get_map_with_robots(
    robot_ids: list[str] = None,
    map_width: float = 600.0,
    map_height: float = 600.0,
    timeout: float = 3.0,
) -> dict:
    """
    Get a map visualization showing robot positions on a 600x600 meter grid.
    Builds an approximate ASCII map downsampled to fit the terminal.
    Args:
        robot_ids: List of robot namespaces. Defaults to ['pearlguard1', 'pearlguard2'].
        map_width: Map width in meters (default: 600).
        map_height: Map height in meters (default: 600).
        timeout: Max seconds to wait per robot (default: 3).
    Returns:
        Dict with map bounds and robot positions for rendering.
    """
    if robot_ids is None:
        robot_ids = ["pearlguard1", "pearlguard2"]

    client = RosClient()
    client.connect()

    robots = []
    try:
        for robot_id in robot_ids:
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

                robots.append({
                    "robot_id": robot_id,
                    "x": round(position["x"], 3),
                    "y": round(position["y"], 3),
                    "theta": round(theta, 3),
                    "theta_deg": round(math.degrees(theta), 1),
                    "online": True,
                })
            else:
                robots.append({
                    "robot_id": robot_id,
                    "x": None,
                    "y": None,
                    "theta": None,
                    "online": False,
                })

    finally:
        client.disconnect()

    # Generate approximate ASCII map (downsampled)
    grid_w = 60
    grid_h = 40
    grid = [[" " for _ in range(grid_w)] for _ in range(grid_h)]

    # Draw border
    for x in range(grid_w):
        grid[0][x] = "-"
        grid[grid_h - 1][x] = "-"
    for y in range(grid_h):
        grid[y][0] = "|"
        grid[y][grid_w - 1] = "|"

    # Place robots on grid
    for robot in robots:
        if robot["online"] and robot["x"] is not None:
            gx = int((robot["x"] + map_width / 2) / map_width * (grid_w - 2)) + 1
            gy = int((map_height / 2 - robot["y"]) / map_height * (grid_h - 2)) + 1
            gx = max(1, min(grid_w - 2, gx))
            gy = max(1, min(grid_h - 2, gy))
            symbol = robot["robot_id"][-1]
            grid[gy][gx] = symbol

    ascii_map = "\n".join("".join(row) for row in grid)

    step_x = map_width / grid_w
    step_y = map_height / grid_h

    return {
        "success": True,
        "map_bounds": {
            "x_min": -map_width / 2,
            "x_max": map_width / 2,
            "y_min": -map_height / 2,
            "y_max": map_height / 2,
            "meters_per_cell": round(max(step_x, step_y), 1),
        },
        "robots": robots,
        "ascii_map": ascii_map,
        "legend": {r["robot_id"]: r["robot_id"][-1] for r in robots},
    }
