#!/usr/bin/env python3
"""
Robo_Fleet - Simulated Multi-Robot Fleet
─────────────────────────────────────────
Simulates 3 TurtleBot3 robots with:
  - Position tracking (amcl_pose)
  - Velocity commands (cmd_vel)
  - Nav2-compatible navigation (navigate_to_pose action)

This replaces Gazebo for local testing. Connects to rosbridge
and publishes/subscribes like real robots would.

Usage:
  conda activate robo_fleet_ros
  python sim/robot_simulator.py

  Or standalone (no ROS2 needed - uses direct WebSocket):
  python sim/robot_simulator.py --standalone
"""

import json
import math
import time
import threading
import argparse
from dataclasses import dataclass, field

try:
    import websocket
    HAS_WEBSOCKET = True
except ImportError:
    HAS_WEBSOCKET = False


@dataclass
class RobotState:
    """State of a single simulated robot."""
    name: str
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vtheta: float = 0.0
    battery: float = 100.0
    status: str = "idle"  # idle, navigating, error
    goal_x: float = 0.0
    goal_y: float = 0.0
    goal_theta: float = 0.0
    nav_speed: float = 0.3  # m/s


class FleetSimulator:
    """Simulates a fleet of TurtleBot3 robots."""

    def __init__(self, robot_names=None, ws_url="ws://localhost:9090"):
        if robot_names is None:
            robot_names = ["tb1", "tb2", "tb3"]

        self.robots = {}
        # Spread robots out initially
        positions = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
        for i, name in enumerate(robot_names):
            x, y = positions[i] if i < len(positions) else (i * 0.5, 0.0)
            self.robots[name] = RobotState(name=name, x=x, y=y)

        self.ws_url = ws_url
        self.ws = None
        self.running = False
        self.lock = threading.Lock()
        self.standalone = False

    def connect(self):
        """Connect to rosbridge WebSocket."""
        try:
            self.ws = websocket.create_connection(self.ws_url, timeout=5)
            print(f"\n🌐 Connected to rosbridge at {self.ws_url}")
            return True
        except Exception as e:
            print(f"\n⚠️  Cannot connect to rosbridge ({e})")
            print("   Running in standalone mode (no rosbridge)")
            self.standalone = True
            return False

    def start(self):
        """Start the simulation loop."""
        self.running = True

        print(f"""
╔══════════════════════════════════════════════════════╗
║       🤖 Robo_Fleet Simulator Running 🤖            ║
╠══════════════════════════════════════════════════════╣
║  Robots: {', '.join(self.robots.keys()):<41} ║
║  Mode:   {'Standalone (no rosbridge)' if self.standalone else 'Connected to rosbridge':<41} ║
║  Rate:   10 Hz                                       ║
╠══════════════════════════════════════════════════════╣
║  Press Ctrl+C to stop                                ║
╚══════════════════════════════════════════════════════╝
""")

        # Start physics update thread
        physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        physics_thread.start()

        # Start status display thread
        display_thread = threading.Thread(target=self._display_loop, daemon=True)
        display_thread.start()

        if not self.standalone:
            # Subscribe to cmd_vel and navigation goals
            self._setup_subscriptions()
            # Start WebSocket listener
            self._ws_listen_loop()
        else:
            # In standalone mode, run the interactive CLI
            self._interactive_loop()

    def _physics_loop(self):
        """Update robot positions at 10Hz."""
        dt = 0.1  # 100ms
        while self.running:
            with self.lock:
                for robot in self.robots.values():
                    if robot.status == "navigating":
                        # Move toward goal
                        dx = robot.goal_x - robot.x
                        dy = robot.goal_y - robot.y
                        dist = math.sqrt(dx**2 + dy**2)

                        if dist < 0.05:  # Reached goal
                            robot.x = robot.goal_x
                            robot.y = robot.goal_y
                            robot.theta = robot.goal_theta
                            robot.status = "idle"
                            robot.vx = 0
                            robot.vy = 0
                            print(f"  ✅ {robot.name} reached goal ({robot.x:.2f}, {robot.y:.2f})")
                            self._publish_nav_result(robot.name, success=True)
                        else:
                            # Move toward goal at nav_speed
                            angle = math.atan2(dy, dx)
                            robot.vx = robot.nav_speed * math.cos(angle)
                            robot.vy = robot.nav_speed * math.sin(angle)
                            robot.x += robot.vx * dt
                            robot.y += robot.vy * dt
                            robot.theta = angle
                            robot.battery -= 0.01  # Drain battery

                    elif robot.vx != 0 or robot.vy != 0 or robot.vtheta != 0:
                        # Manual cmd_vel control
                        robot.x += robot.vx * dt
                        robot.y += robot.vy * dt
                        robot.theta += robot.vtheta * dt
                        robot.battery -= 0.005

                    # Publish pose (if connected)
                    if not self.standalone:
                        self._publish_pose(robot)

            time.sleep(dt)

    def _display_loop(self):
        """Print robot status every 2 seconds."""
        while self.running:
            time.sleep(2)
            with self.lock:
                print(f"\r  {'─' * 50}")
                for r in self.robots.values():
                    status_icon = {"idle": "⏸️ ", "navigating": "🚗", "error": "❌"}
                    icon = status_icon.get(r.status, "?")
                    print(f"  {icon} {r.name}: pos=({r.x:.2f}, {r.y:.2f}, θ={r.theta:.1f}) "
                          f"bat={r.battery:.0f}% [{r.status}]")

    def _publish_pose(self, robot):
        """Publish robot pose to rosbridge."""
        if self.ws is None:
            return
        try:
            msg = {
                "op": "publish",
                "topic": f"/{robot.name}/amcl_pose",
                "type": "geometry_msgs/msg/PoseWithCovarianceStamped",
                "msg": {
                    "header": {"frame_id": "map", "stamp": {"sec": int(time.time()), "nanosec": 0}},
                    "pose": {
                        "pose": {
                            "position": {"x": robot.x, "y": robot.y, "z": 0.0},
                            "orientation": {
                                "x": 0.0, "y": 0.0,
                                "z": math.sin(robot.theta / 2),
                                "w": math.cos(robot.theta / 2),
                            }
                        },
                        "covariance": [0.0] * 36
                    }
                }
            }
            self.ws.send(json.dumps(msg))
        except Exception:
            pass

    def _publish_nav_result(self, robot_name, success=True):
        """Publish navigation action result."""
        if self.ws is None or self.standalone:
            return
        # The actual result is sent via action protocol - handled in ws listener

    def _setup_subscriptions(self):
        """Subscribe to relevant topics via rosbridge."""
        for name in self.robots:
            # Subscribe to cmd_vel
            self.ws.send(json.dumps({
                "op": "subscribe",
                "topic": f"/{name}/cmd_vel",
                "type": "geometry_msgs/msg/TwistStamped"
            }))
        print("  📡 Subscribed to cmd_vel for all robots")

    def _ws_listen_loop(self):
        """Listen for incoming WebSocket messages (cmd_vel, nav goals)."""
        try:
            while self.running:
                try:
                    self.ws.settimeout(1.0)
                    raw = self.ws.recv()
                    data = json.loads(raw)
                    self._handle_message(data)
                except websocket.WebSocketTimeoutException:
                    continue
                except websocket.WebSocketConnectionClosedException:
                    print("\n⚠️  rosbridge connection closed")
                    break
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False

    def _handle_message(self, data):
        """Handle incoming rosbridge messages."""
        op = data.get("op", "")

        if op == "publish":
            topic = data.get("topic", "")
            # Handle cmd_vel
            for name in self.robots:
                if topic == f"/{name}/cmd_vel":
                    msg = data.get("msg", {})
                    twist = msg.get("twist", msg)  # TwistStamped or Twist
                    with self.lock:
                        robot = self.robots[name]
                        robot.vx = twist.get("linear", {}).get("x", 0.0)
                        robot.vy = twist.get("linear", {}).get("y", 0.0)
                        robot.vtheta = twist.get("angular", {}).get("z", 0.0)

        elif op == "send_action_goal":
            # Handle navigation goal
            action = data.get("action", "")
            goal_id = data.get("id", "")
            args = data.get("args", {})

            for name in self.robots:
                if f"/{name}/navigate_to_pose" in action:
                    pose = args.get("pose", {}).get("pose", {})
                    pos = pose.get("position", {})
                    orient = pose.get("orientation", {})

                    # Extract yaw from quaternion
                    z = orient.get("z", 0.0)
                    w = orient.get("w", 1.0)
                    theta = 2 * math.atan2(z, w)

                    with self.lock:
                        robot = self.robots[name]
                        robot.goal_x = pos.get("x", 0.0)
                        robot.goal_y = pos.get("y", 0.0)
                        robot.goal_theta = theta
                        robot.status = "navigating"

                    print(f"  🎯 {name}: navigating to ({robot.goal_x:.2f}, {robot.goal_y:.2f})")

                    # Send action result when done (handled in physics loop)
                    threading.Thread(
                        target=self._wait_and_send_result,
                        args=(name, goal_id, action),
                        daemon=True
                    ).start()

        elif op == "cancel_action_goal":
            action = data.get("action", "")
            for name in self.robots:
                if f"/{name}/" in action:
                    with self.lock:
                        self.robots[name].status = "idle"
                        self.robots[name].vx = 0
                        self.robots[name].vy = 0
                    print(f"  🛑 {name}: navigation cancelled")

    def _wait_and_send_result(self, robot_name, goal_id, action):
        """Wait for robot to reach goal, then send result."""
        while self.running:
            time.sleep(0.2)
            with self.lock:
                robot = self.robots[robot_name]
                if robot.status != "navigating":
                    break

        # Send action result
        if self.ws and not self.standalone:
            try:
                self.ws.send(json.dumps({
                    "op": "action_result",
                    "id": goal_id,
                    "action": action,
                    "values": {},
                    "status": 4  # SUCCEEDED
                }))
            except Exception:
                pass

    def _interactive_loop(self):
        """Interactive CLI for standalone mode."""
        print("\n  Standalone Commands:")
        print("    nav <robot> <x> <y>    - Navigate robot to position")
        print("    pos                    - Show all positions")
        print("    stop <robot>           - Stop a robot")
        print("    quit                   - Exit")
        print()

        try:
            while self.running:
                try:
                    cmd = input("  > ").strip().split()
                except EOFError:
                    break

                if not cmd:
                    continue

                if cmd[0] == "quit":
                    break
                elif cmd[0] == "pos":
                    pass  # Display loop handles this
                elif cmd[0] == "nav" and len(cmd) >= 4:
                    name = cmd[1]
                    if name in self.robots:
                        with self.lock:
                            robot = self.robots[name]
                            robot.goal_x = float(cmd[2])
                            robot.goal_y = float(cmd[3])
                            robot.goal_theta = float(cmd[4]) if len(cmd) > 4 else 0.0
                            robot.status = "navigating"
                        print(f"  🎯 {name}: navigating to ({cmd[2]}, {cmd[3]})")
                    else:
                        print(f"  ❌ Unknown robot: {name}")
                elif cmd[0] == "stop" and len(cmd) >= 2:
                    name = cmd[1]
                    if name in self.robots:
                        with self.lock:
                            self.robots[name].status = "idle"
                            self.robots[name].vx = 0
                            self.robots[name].vy = 0
                        print(f"  🛑 {name}: stopped")
                else:
                    print("  Unknown command. Try: nav tb1 2.0 3.0")
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            print("\n  Simulator stopped.")


def main():
    parser = argparse.ArgumentParser(description="Robo_Fleet Simulator")
    parser.add_argument("--standalone", action="store_true", help="Run without rosbridge")
    parser.add_argument("--host", default="localhost", help="rosbridge host")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge port")
    parser.add_argument("--robots", nargs="+", default=["tb1", "tb2", "tb3"], help="Robot names")
    args = parser.parse_args()

    sim = FleetSimulator(
        robot_names=args.robots,
        ws_url=f"ws://{args.host}:{args.port}"
    )

    if not args.standalone:
        sim.connect()
    else:
        sim.standalone = True

    sim.start()


if __name__ == "__main__":
    main()
