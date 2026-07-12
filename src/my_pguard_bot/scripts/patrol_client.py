#!/usr/bin/env python3
"""PGuard perimeter patrol client.

Loads config/patrol_waypoints.yaml and drives the loop by sending a single
`nav2_msgs/action/FollowGPSWaypoints` goal (whole loop, ordered) to Nav2.
When the goal completes it optionally restarts (`~loop` param, default True),
producing an indefinite perimeter patrol.

Params:
  ~yaml_file      Absolute path to the waypoints YAML.
                  Default = package_share/config/patrol_waypoints.yaml
  ~loop           Restart the patrol after each completion (bool, default True)
  ~loop_delay_s   Delay between laps in seconds (double, default 5.0)
  ~start_delay_s  Wait after startup for Nav2 lifecycle activation (default 8.0)

Usage:
    ros2 run my_pguard_bot patrol_client
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geographic_msgs.msg import GeoPose
from nav2_msgs.action import FollowGPSWaypoints


def yaw_to_quat(yaw_rad: float) -> tuple[float, float, float, float]:
    """ENU yaw (rad) to quaternion (x, y, z, w) — roll/pitch = 0."""
    half = yaw_rad / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def parse_yaml_minimal(text: str) -> list[dict]:
    """Tiny YAML parser tailored to the flat mapping list emitted by
    generate_waypoints.py — avoids adding a PyYAML dependency."""
    out: list[dict] = []
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("- {") or not line.endswith("}"):
            continue
        body = line[3:-1]
        entry: dict = {}
        for kv in body.split(","):
            if ":" not in kv:
                continue
            k, v = kv.split(":", 1)
            entry[k.strip()] = float(v.strip())
        if "lat" in entry and "lon" in entry:
            out.append(entry)
    return out


class PatrolClient(Node):
    def __init__(self) -> None:
        super().__init__("pguard_patrol_client")

        default_yaml = str(Path(get_package_share_directory("my_pguard_bot"))
                           / "config" / "patrol_waypoints.yaml")
        self.declare_parameter("yaml_file", default_yaml)
        self.declare_parameter("loop", True)
        self.declare_parameter("loop_delay_s", 5.0)
        self.declare_parameter("start_delay_s", 8.0)

        self.yaml_file = self.get_parameter("yaml_file").value
        self.loop = self.get_parameter("loop").value
        self.loop_delay_s = self.get_parameter("loop_delay_s").value
        self.start_delay_s = self.get_parameter("start_delay_s").value

        self._client = ActionClient(self, FollowGPSWaypoints,
                                    "/follow_gps_waypoints")
        self._waypoints = self._load_waypoints()

        self.get_logger().info(
            f"Loaded {len(self._waypoints)} GPS waypoints from {self.yaml_file}")
        self.get_logger().info(
            f"Waiting {self.start_delay_s:.1f}s for Nav2 lifecycle activation ...")

        self._timer = self.create_timer(self.start_delay_s, self._kickoff)
        self._lap = 0

    def _load_waypoints(self) -> list[GeoPose]:
        try:
            entries = parse_yaml_minimal(Path(self.yaml_file).read_text())
        except FileNotFoundError:
            self.get_logger().error(f"Waypoints YAML not found: {self.yaml_file}")
            return []
        wps: list[GeoPose] = []
        for e in entries:
            gp = GeoPose()
            gp.position.latitude = float(e["lat"])
            gp.position.longitude = float(e["lon"])
            gp.position.altitude = 0.0
            yaw = math.radians(e.get("yaw_deg", 0.0))
            qx, qy, qz, qw = yaw_to_quat(yaw)
            gp.orientation.x = qx
            gp.orientation.y = qy
            gp.orientation.z = qz
            gp.orientation.w = qw
            wps.append(gp)
        return wps

    def _kickoff(self) -> None:
        self._timer.cancel()
        if not self._waypoints:
            self.get_logger().error("No waypoints to patrol; shutting down.")
            rclpy.shutdown()
            return
        self._send_lap()

    def _send_lap(self) -> None:
        self._lap += 1
        self.get_logger().info(
            f"=== Patrol lap #{self._lap}: sending {len(self._waypoints)} "
            "GPS waypoints ===")

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "/follow_gps_waypoints action server not available. "
                "Is Nav2 up and lifecycle activated?")
            rclpy.shutdown()
            return

        goal = FollowGPSWaypoints.Goal()
        goal.gps_poses = self._waypoints
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"  -> lap {self._lap} progress: waypoint "
            f"{fb.current_waypoint + 1}/{len(self._waypoints)}")

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted; patrol running.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        missed = getattr(result, "missed_waypoints", [])
        self.get_logger().info(
            f"Lap #{self._lap} complete. Missed waypoints: {list(missed)}")
        if self.loop:
            self.get_logger().info(
                f"Restarting patrol in {self.loop_delay_s:.1f}s ...")
            self._relaunch_timer = self.create_timer(
                self.loop_delay_s, self._restart_once)
        else:
            self.get_logger().info("Loop disabled; shutting down.")
            rclpy.shutdown()

    def _restart_once(self) -> None:
        self._relaunch_timer.cancel()
        self._send_lap()


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = PatrolClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
