"""
Natural Language Goal Interface.
Named location registry + resolver for human-friendly navigation commands.
"""

import math
import json
import os
from server import mcp
from ros.ros_client import RosClient
from coordination.fleet_state import FleetStateManager


# ─── Location Registry ───

_LOCATIONS_FILE = os.path.join(os.path.dirname(__file__), "..", "locations.json")

# Default locations
_DEFAULT_LOCATIONS = {
    "origin": {"x": 0.0, "y": 0.0, "description": "Map origin (0, 0)"},
    "charging_station": {"x": 0.0, "y": 0.0, "description": "Charging dock at origin"},
    "warehouse": {"x": 1.0, "y": 1.0, "description": "Warehouse area"},
    "dock": {"x": -1.0, "y": 0.0, "description": "Loading dock"},
    "entrance": {"x": 0.0, "y": 1.0, "description": "Main entrance"},
    "storage": {"x": -1.0, "y": -1.0, "description": "Storage room"},
    "workstation_a": {"x": 1.0, "y": -0.5, "description": "Workstation A"},
    "workstation_b": {"x": -0.5, "y": 0.5, "description": "Workstation B"},
}


def _load_locations():
    """Load locations from file or defaults."""
    if os.path.exists(_LOCATIONS_FILE):
        try:
            with open(_LOCATIONS_FILE, 'r') as f:
                return json.load(f)
        except:
            pass
    return dict(_DEFAULT_LOCATIONS)


def _save_locations(locations):
    """Save locations to file."""
    try:
        with open(_LOCATIONS_FILE, 'w') as f:
            json.dump(locations, f, indent=2)
    except:
        pass


# ─── MCP Tools ───

@mcp.tool()
def list_locations() -> dict:
    """
    List all named locations in the registry.
    Returns:
        Dict with all location names, coordinates, and descriptions.
    """
    locations = _load_locations()
    return {
        "success": True,
        "count": len(locations),
        "locations": {
            name: {"x": loc["x"], "y": loc["y"], "description": loc.get("description", "")}
            for name, loc in locations.items()
        },
    }


@mcp.tool()
def add_location(
    name: str,
    x: float,
    y: float,
    description: str = "",
) -> dict:
    """
    Add or update a named location in the registry.
    Args:
        name: Location name (lowercase, underscore-separated).
        x: X coordinate in meters.
        y: Y coordinate in meters.
        description: Optional description.
    Returns:
        Confirmation with location details.
    """
    locations = _load_locations()
    name = name.lower().replace(" ", "_")

    is_update = name in locations
    locations[name] = {"x": x, "y": y, "description": description}
    _save_locations(locations)

    return {
        "success": True,
        "action": "updated" if is_update else "added",
        "name": name,
        "x": x,
        "y": y,
        "description": description,
    }


@mcp.tool()
def remove_location(name: str) -> dict:
    """
    Remove a named location from the registry.
    Args:
        name: Location name to remove.
    Returns:
        Confirmation or error if not found.
    """
    locations = _load_locations()
    name = name.lower().replace(" ", "_")

    if name not in locations:
        return {"success": False, "error": f"Location '{name}' not found"}

    del locations[name]
    _save_locations(locations)
    return {"success": True, "removed": name}


@mcp.tool()
def go_to_location(
    robot_id: str,
    location_name: str,
    timeout: float = 30.0,
) -> dict:
    """
    Navigate a specific robot to a named location.
    Args:
        robot_id: Robot namespace (e.g. 'tb1').
        location_name: Name of registered location (e.g. 'warehouse', 'charging_station').
        timeout: Navigation timeout in seconds.
    Returns:
        Navigation result.
    """
    locations = _load_locations()
    location_name = location_name.lower().replace(" ", "_")

    if location_name not in locations:
        available = list(locations.keys())
        return {
            "success": False,
            "error": f"Unknown location: '{location_name}'",
            "available_locations": available,
        }

    loc = locations[location_name]
    x, y = loc["x"], loc["y"]

    client = RosClient()
    client.connect()
    try:
        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            }
        }
        resp = client.send_goal(
            f"/{robot_id}/navigate_to_pose",
            "nav2_msgs/action/NavigateToPose",
            goal,
        )
        result = client.wait_for_result(
            f"/{robot_id}/navigate_to_pose",
            resp["goal_id"],
            timeout=timeout,
        )
        success = result and result.get("success", False)
        return {
            "success": success,
            "robot_id": robot_id,
            "location": location_name,
            "target": {"x": x, "y": y},
            "description": loc.get("description", ""),
            "nav_result": result,
        }
    finally:
        client.disconnect()


@mcp.tool()
def send_nearest_to(
    location_name: str,
    group: str = None,
    timeout: float = 30.0,
) -> dict:
    """
    Send the nearest available robot to a named location.
    Automatically picks the closest idle robot with sufficient battery.
    Args:
        location_name: Name of registered location.
        group: Optional - only consider robots in this group.
        timeout: Navigation timeout in seconds.
    Returns:
        Which robot was selected and navigation result.
    """
    locations = _load_locations()
    location_name = location_name.lower().replace(" ", "_")

    if location_name not in locations:
        return {
            "success": False,
            "error": f"Unknown location: '{location_name}'",
            "available_locations": list(locations.keys()),
        }

    loc = locations[location_name]
    x, y = loc["x"], loc["y"]

    # Get fleet state
    manager = FleetStateManager.get_instance()
    nearest = manager.get_nearest_available(x, y, group=group)

    if nearest is None:
        return {
            "success": False,
            "error": "No available robots (all busy or offline)",
            "location": location_name,
        }

    # Navigate the nearest robot
    robot_id = nearest.robot_id
    distance = nearest.distance_to(x, y)

    client = RosClient()
    client.connect()
    try:
        goal = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            }
        }
        resp = client.send_goal(
            f"/{robot_id}/navigate_to_pose",
            "nav2_msgs/action/NavigateToPose",
            goal,
        )
        result = client.wait_for_result(
            f"/{robot_id}/navigate_to_pose",
            resp["goal_id"],
            timeout=timeout,
        )
        success = result and result.get("success", False)
        return {
            "success": success,
            "robot_id": robot_id,
            "location": location_name,
            "target": {"x": x, "y": y},
            "distance": round(distance, 3),
            "description": loc.get("description", ""),
        }
    finally:
        client.disconnect()
