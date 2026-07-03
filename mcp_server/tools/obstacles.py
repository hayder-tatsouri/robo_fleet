import math
from server import mcp
from ros.ros_client import RosClient


@mcp.tool()
def check_obstacles(
    robot_id: str,
    distance_threshold: float = 0.5,
    timeout: float = 3.0,
) -> dict:
    """
    Check for obstacles near a robot using laser scan data.
    Args:
        robot_id: Robot namespace (e.g. 'tb1').
        distance_threshold: Alert if obstacles closer than this (meters, default: 0.5).
        timeout: Max seconds to wait for scan data (default: 3).
    Returns:
        Dict with obstacle info: closest distance, direction, alert status.
    """
    client = RosClient()
    client.connect()

    try:
        msg = client.subscribe_once(
            topic=f"/{robot_id}/scan",
            msg_type="sensor_msgs/msg/LaserScan",
            timeout=timeout,
        )

        if msg is None:
            return {
                "success": False,
                "error": f"No laser scan data from {robot_id} (timeout)",
                "robot_id": robot_id,
            }

        ranges = msg.get("ranges", [])
        angle_min = msg.get("angle_min", 0.0)
        angle_increment = msg.get("angle_increment", 0.01)
        range_min = msg.get("range_min", 0.1)
        range_max = msg.get("range_max", 10.0)

        # Filter valid readings
        valid_readings = []
        for i, r in enumerate(ranges):
            if range_min <= r <= range_max:
                angle = angle_min + i * angle_increment
                valid_readings.append({"distance": r, "angle": angle, "index": i})

        if not valid_readings:
            return {
                "success": True,
                "robot_id": robot_id,
                "obstacle_detected": False,
                "closest_distance": None,
                "message": "No valid readings in scan",
            }

        # Find closest obstacle
        closest = min(valid_readings, key=lambda x: x["distance"])
        closest_distance = closest["distance"]
        closest_angle = closest["angle"]

        # Determine direction
        angle_deg = math.degrees(closest_angle)
        if -30 <= angle_deg <= 30:
            direction = "front"
        elif 30 < angle_deg <= 90:
            direction = "front-left"
        elif -90 <= angle_deg < -30:
            direction = "front-right"
        elif 90 < angle_deg <= 150:
            direction = "left"
        elif -150 <= angle_deg < -90:
            direction = "right"
        else:
            direction = "behind"

        # Count obstacles within threshold
        obstacles_near = [r for r in valid_readings if r["distance"] < distance_threshold]

        alert = closest_distance < distance_threshold

        return {
            "success": True,
            "robot_id": robot_id,
            "obstacle_detected": alert,
            "closest_distance": round(closest_distance, 3),
            "closest_direction": direction,
            "closest_angle_deg": round(angle_deg, 1),
            "obstacles_within_threshold": len(obstacles_near),
            "threshold_meters": distance_threshold,
            "total_valid_readings": len(valid_readings),
            "alert": alert,
            "message": f"{'ALERT: ' if alert else ''}Closest obstacle at {closest_distance:.2f}m ({direction})",
        }
    finally:
        client.disconnect()
