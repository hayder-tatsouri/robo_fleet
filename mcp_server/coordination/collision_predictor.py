"""
Path Collision Predictor - Linear trajectory extrapolation.
Predicts if two moving robots will come within buffer distance
of each other within a time window.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class CollisionRisk:
    """Represents a predicted collision between two robots."""
    robot_a: str
    robot_b: str
    time_to_collision: float  # seconds until collision
    collision_point: tuple  # (x, y) predicted collision location
    min_distance: float  # predicted minimum distance between robots
    severity: str  # "critical", "warning", "safe"
    resolution: str  # suggested action

    def to_dict(self):
        return {
            "robot_a": self.robot_a,
            "robot_b": self.robot_b,
            "time_to_collision": round(self.time_to_collision, 2),
            "collision_point": {"x": round(self.collision_point[0], 3), "y": round(self.collision_point[1], 3)},
            "min_distance": round(self.min_distance, 3),
            "severity": self.severity,
            "resolution": self.resolution,
        }


class CollisionPredictor:
    """
    Predicts collisions using linear trajectory extrapolation.
    
    For each pair of robots that are navigating:
    1. Extrapolate position based on current velocity or goal direction
    2. Find time of closest approach (TCA)
    3. If min distance < buffer at TCA, flag collision risk
    """

    def __init__(self, fleet_manager, buffer_distance=0.4, time_horizon=5.0, robot_speed=0.3):
        """
        Args:
            fleet_manager: FleetStateManager instance
            buffer_distance: Minimum safe distance (meters)
            time_horizon: How far ahead to predict (seconds)
            robot_speed: Assumed robot speed (m/s)
        """
        self.fleet = fleet_manager
        self.buffer_distance = buffer_distance
        self.time_horizon = time_horizon
        self.robot_speed = robot_speed

    def predict_all(self):
        """
        Check all robot pairs for potential collisions.
        Returns list of CollisionRisk objects.
        """
        risks = []
        robots = list(self.fleet.robots.values())

        # O(N^2) pairwise check
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                risk = self._check_pair(robots[i], robots[j])
                if risk:
                    risks.append(risk)

        # Sort by severity (critical first) then time
        severity_order = {"critical": 0, "warning": 1, "safe": 2}
        risks.sort(key=lambda r: (severity_order.get(r.severity, 3), r.time_to_collision))

        return risks

    def _check_pair(self, robot_a, robot_b):
        """
        Check if two robots are on a collision course.
        Uses linear extrapolation from current position toward goal.
        """
        # Get trajectories
        vel_a = self._get_velocity(robot_a)
        vel_b = self._get_velocity(robot_b)

        # If both stationary, no collision risk from motion
        if vel_a == (0, 0) and vel_b == (0, 0):
            # Check static proximity
            dist = math.sqrt((robot_a.x - robot_b.x)**2 + (robot_a.y - robot_b.y)**2)
            if dist < self.buffer_distance:
                return CollisionRisk(
                    robot_a=robot_a.robot_id,
                    robot_b=robot_b.robot_id,
                    time_to_collision=0.0,
                    collision_point=((robot_a.x + robot_b.x) / 2, (robot_a.y + robot_b.y) / 2),
                    min_distance=dist,
                    severity="critical",
                    resolution=f"Robots already within buffer! Move {robot_b.robot_id} away.",
                )
            return None

        # Find time of closest approach (TCA)
        # Relative position and velocity
        dx = robot_b.x - robot_a.x
        dy = robot_b.y - robot_a.y
        dvx = vel_b[0] - vel_a[0]
        dvy = vel_b[1] - vel_a[1]

        # TCA = -dot(dp, dv) / dot(dv, dv)
        dv_sq = dvx * dvx + dvy * dvy
        if dv_sq < 1e-10:
            # Parallel or same velocity - check current distance
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < self.buffer_distance:
                return CollisionRisk(
                    robot_a=robot_a.robot_id,
                    robot_b=robot_b.robot_id,
                    time_to_collision=0.0,
                    collision_point=((robot_a.x + robot_b.x) / 2, (robot_a.y + robot_b.y) / 2),
                    min_distance=dist,
                    severity="warning",
                    resolution="Robots moving in parallel within buffer distance.",
                )
            return None

        tca = -(dx * dvx + dy * dvy) / dv_sq

        # Only care about future collisions within time horizon
        if tca < 0 or tca > self.time_horizon:
            return None

        # Distance at TCA
        closest_dx = dx + dvx * tca
        closest_dy = dy + dvy * tca
        min_dist = math.sqrt(closest_dx * closest_dx + closest_dy * closest_dy)

        if min_dist >= self.buffer_distance:
            return None  # Safe

        # Collision predicted!
        # Calculate collision point (midpoint at TCA)
        col_x = robot_a.x + vel_a[0] * tca + closest_dx / 2
        col_y = robot_a.y + vel_a[1] * tca + closest_dy / 2

        # Determine severity
        if min_dist < self.buffer_distance * 0.5:
            severity = "critical"
        else:
            severity = "warning"

        # Determine resolution
        resolution = self._suggest_resolution(robot_a, robot_b, tca)

        return CollisionRisk(
            robot_a=robot_a.robot_id,
            robot_b=robot_b.robot_id,
            time_to_collision=tca,
            collision_point=(col_x, col_y),
            min_distance=min_dist,
            severity=severity,
            resolution=resolution,
        )

    def _get_velocity(self, robot):
        """
        Get robot velocity vector.
        If navigating: compute direction toward goal at assumed speed.
        If has explicit velocity: use that.
        Otherwise: stationary.
        """
        if robot.status == "navigating" and robot.goal_x is not None:
            dx = robot.goal_x - robot.x
            dy = robot.goal_y - robot.y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.01:
                return (0.0, 0.0)
            # Unit vector * speed
            vx = (dx / dist) * self.robot_speed
            vy = (dy / dist) * self.robot_speed
            return (vx, vy)

        # Check if robot has stored velocity
        if hasattr(robot, 'vx') and (robot.vx != 0 or robot.vy != 0):
            return (robot.vx, robot.vy)

        return (0.0, 0.0)

    def _suggest_resolution(self, robot_a, robot_b, tca):
        """Suggest collision resolution based on priority."""
        priority_a = getattr(robot_a, 'priority', 0)
        priority_b = getattr(robot_b, 'priority', 0)

        if priority_a >= priority_b:
            lower = robot_b.robot_id
            higher = robot_a.robot_id
        else:
            lower = robot_a.robot_id
            higher = robot_b.robot_id

        if tca < 1.0:
            return f"URGENT: Stop {lower} immediately. {higher} has priority."
        elif tca < 3.0:
            return f"Pause {lower} for {tca:.1f}s to let {higher} pass."
        else:
            return f"Consider rerouting {lower}. Collision in {tca:.1f}s."
