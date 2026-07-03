#!/usr/bin/env python3
"""
Robo_Fleet - One-Command Launcher
══════════════════════════════════
Starts the entire stack in a single process:
  1. Mock rosbridge (WebSocket server on port 9090)
  2. Fleet simulator (3 TurtleBot3 robots)
  3. MCP server (stdio transport)

Usage:
  python run.py              # Start rosbridge + simulator
  python run.py --with-mcp   # Also start MCP server
  python run.py --test        # Run integration tests after startup

Works on macOS and Linux. No ROS2, Docker, or conda needed.
"""

import asyncio
import json
import math
import subprocess
import time
import threading
import argparse
import signal
import sys
import os

# ═══════════════════════════════════════════════════════════
# MOCK ROSBRIDGE SERVER (replaces ros2 + rosbridge_suite)
# ═══════════════════════════════════════════════════════════

try:
    import websockets
    import websockets.server
except ImportError:
    print("❌ Missing dependency: websockets")
    print("   Fix: pip install websockets")
    sys.exit(1)


class MockRosbridge:
    """
    Lightweight WebSocket server that implements the rosbridge protocol.
    Handles: publish, subscribe, send_action_goal, cancel_action_goal
    """

    def __init__(self, host="0.0.0.0", port=9090):
        self.host = host
        self.port = port
        self.clients = set()
        self.subscriptions = {}  # topic -> set of (client, sub_id)
        self.lock = asyncio.Lock()

    async def handler(self, websocket):
        self.clients.add(websocket)
        try:
            async for message in websocket:
                await self._handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            # Clean up subscriptions
            async with self.lock:
                for topic in list(self.subscriptions.keys()):
                    self.subscriptions[topic] = {
                        (ws, sid) for ws, sid in self.subscriptions[topic]
                        if ws != websocket
                    }

    async def _handle_message(self, websocket, raw):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return

        op = data.get("op", "")

        if op == "subscribe":
            topic = data.get("topic", "")
            sub_id = data.get("id", "")
            async with self.lock:
                if topic not in self.subscriptions:
                    self.subscriptions[topic] = set()
                self.subscriptions[topic].add((websocket, sub_id))

        elif op == "unsubscribe":
            topic = data.get("topic", "")
            sub_id = data.get("id", "")
            async with self.lock:
                if topic in self.subscriptions:
                    self.subscriptions[topic] = {
                        (ws, sid) for ws, sid in self.subscriptions[topic]
                        if not (ws == websocket and sid == sub_id)
                    }

        elif op == "publish":
            # Forward to all subscribers of this topic
            topic = data.get("topic", "")
            async with self.lock:
                subscribers = self.subscriptions.get(topic, set()).copy()
            for ws, _ in subscribers:
                if ws != websocket:
                    try:
                        await ws.send(raw)
                    except:
                        pass

        elif op == "send_action_goal":
            # Forward to all other clients (simulator will pick it up)
            for client in self.clients.copy():
                if client != websocket:
                    try:
                        await client.send(raw)
                    except:
                        pass

        elif op == "cancel_action_goal":
            for client in self.clients.copy():
                if client != websocket:
                    try:
                        await client.send(raw)
                    except:
                        pass

        elif op == "action_result":
            # Forward result back to the client that sent the goal
            for client in self.clients.copy():
                if client != websocket:
                    try:
                        await client.send(raw)
                    except:
                        pass

        elif op == "action_feedback":
            for client in self.clients.copy():
                if client != websocket:
                    try:
                        await client.send(raw)
                    except:
                        pass

    async def start(self):
        server = await websockets.serve(
            self.handler, self.host, self.port,
            ping_interval=20, ping_timeout=20
        )
        return server


# ═══════════════════════════════════════════════════════════
# FLEET SIMULATOR
# ═══════════════════════════════════════════════════════════

class Robot:
    def __init__(self, name, x=0.0, y=0.0):
        self.name = name
        self.x = x
        self.y = y
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.battery = 100.0
        self.voltage = 12.6
        self.status = "idle"
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        self.goal_id = None
        self.speed = 0.5  # m/s navigation speed


class FleetSimulator:
    def __init__(self, robot_names=None, ws_url="ws://localhost:9090"):
        if robot_names is None:
            robot_names = ["tb1", "tb2", "tb3"]
        positions = [(0.0, 0.0), (1.5, 0.0), (0.0, 1.5)]
        self.robots = {}
        for i, name in enumerate(robot_names):
            x, y = positions[i] if i < len(positions) else (0, 0)
            self.robots[name] = Robot(name, x, y)

        self.ws_url = ws_url
        self.ws = None
        self.running = True

    async def connect(self):
        import websockets.client
        retries = 0
        while retries < 10:
            try:
                self.ws = await websockets.client.connect(self.ws_url)
                return True
            except:
                retries += 1
                await asyncio.sleep(0.5)
        return False

    async def run(self):
        if not await self.connect():
            print("  ❌ Simulator: cannot connect to rosbridge")
            return

        # Subscribe to cmd_vel for all robots
        for name in self.robots:
            await self.ws.send(json.dumps({
                "op": "subscribe",
                "topic": f"/{name}/cmd_vel",
                "type": "geometry_msgs/msg/TwistStamped"
            }))

        # Start physics and pose publisher
        asyncio.create_task(self._physics_loop())
        asyncio.create_task(self._pose_publisher())

        # Listen for messages
        try:
            async for raw in self.ws:
                data = json.loads(raw)
                await self._handle(data)
        except:
            pass

    async def _physics_loop(self):
        dt = 0.05  # 20Hz
        while self.running:
            for robot in self.robots.values():
                if robot.status == "navigating":
                    dx = robot.goal_x - robot.x
                    dy = robot.goal_y - robot.y
                    dist = math.sqrt(dx*dx + dy*dy)

                    if dist < 0.05:
                        robot.x = robot.goal_x
                        robot.y = robot.goal_y
                        robot.theta = robot.goal_theta
                        robot.status = "idle"
                        robot.vx = 0
                        robot.vy = 0
                        # Send success result
                        if self.ws and robot.goal_id:
                            await self.ws.send(json.dumps({
                                "op": "action_result",
                                "id": robot.goal_id,
                                "action": f"/{robot.name}/navigate_to_pose",
                                "values": {},
                                "status": 4
                            }))
                            print(f"  ✅ {robot.name} reached ({robot.x:.2f}, {robot.y:.2f})")
                            robot.goal_id = None
                    else:
                        angle = math.atan2(dy, dx)
                        robot.x += robot.speed * math.cos(angle) * dt
                        robot.y += robot.speed * math.sin(angle) * dt
                        robot.theta = angle
                        robot.battery -= 0.002

                elif robot.vx != 0 or robot.vy != 0 or robot.vtheta != 0:
                    robot.x += robot.vx * dt
                    robot.y += robot.vy * dt
                    robot.theta += robot.vtheta * dt

            await asyncio.sleep(dt)

    async def _pose_publisher(self):
        while self.running:
            for robot in self.robots.values():
                if self.ws:
                    # Publish pose
                    pose_msg = {
                        "op": "publish",
                        "topic": f"/{robot.name}/amcl_pose",
                        "type": "geometry_msgs/msg/PoseWithCovarianceStamped",
                        "msg": {
                            "header": {"frame_id": "map"},
                            "pose": {
                                "pose": {
                                    "position": {"x": robot.x, "y": robot.y, "z": 0.0},
                                    "orientation": {
                                        "x": 0.0, "y": 0.0,
                                        "z": math.sin(robot.theta / 2),
                                        "w": math.cos(robot.theta / 2)
                                    }
                                },
                                "covariance": [0.0] * 36
                            }
                        }
                    }
                    try:
                        await self.ws.send(json.dumps(pose_msg))
                    except:
                        pass

                    # Publish battery state
                    robot.voltage = 10.0 + (robot.battery / 100.0) * 2.6
                    battery_msg = {
                        "op": "publish",
                        "topic": f"/{robot.name}/battery_state",
                        "type": "sensor_msgs/msg/BatteryState",
                        "msg": {
                            "percentage": robot.battery / 100.0,
                            "voltage": robot.voltage,
                            "current": -0.5 if robot.status == "navigating" else -0.1,
                            "charge": robot.battery / 100.0 * 2.2,
                            "capacity": 2.2,
                            "present": True,
                        }
                    }
                    try:
                        await self.ws.send(json.dumps(battery_msg))
                    except:
                        pass

                    # Publish simulated laser scan (360 degree, 1 degree resolution)
                    import random
                    num_rays = 360
                    ranges = []
                    for i in range(num_rays):
                        # Simulate some obstacles at random distances
                        base_dist = 3.0 + random.uniform(-0.5, 0.5)
                        # Add closer obstacles in certain directions
                        angle = math.radians(i)
                        # Simulated walls
                        if abs(robot.x) > 4:
                            wall_dist = max(0.2, 5.0 - abs(robot.x))
                            if (angle < 0.5 or angle > 5.8):
                                base_dist = min(base_dist, wall_dist)
                        if abs(robot.y) > 4:
                            wall_dist = max(0.2, 5.0 - abs(robot.y))
                            if 1.3 < angle < 1.8:
                                base_dist = min(base_dist, wall_dist)
                        ranges.append(round(base_dist, 3))

                    scan_msg = {
                        "op": "publish",
                        "topic": f"/{robot.name}/scan",
                        "type": "sensor_msgs/msg/LaserScan",
                        "msg": {
                            "header": {"frame_id": f"{robot.name}/base_scan"},
                            "angle_min": -math.pi,
                            "angle_max": math.pi,
                            "angle_increment": 2 * math.pi / num_rays,
                            "range_min": 0.12,
                            "range_max": 10.0,
                            "ranges": ranges,
                        }
                    }
                    try:
                        await self.ws.send(json.dumps(scan_msg))
                    except:
                        pass

            await asyncio.sleep(0.5)  # 2Hz updates

    async def _handle(self, data):
        op = data.get("op", "")

        if op == "publish":
            topic = data.get("topic", "")
            for name, robot in self.robots.items():
                if topic == f"/{name}/cmd_vel":
                    msg = data.get("msg", {})
                    twist = msg.get("twist", msg)
                    robot.vx = twist.get("linear", {}).get("x", 0.0)
                    robot.vy = twist.get("linear", {}).get("y", 0.0)
                    robot.vtheta = twist.get("angular", {}).get("z", 0.0)

        elif op == "send_action_goal":
            action = data.get("action", "")
            goal_id = data.get("id", "")
            args = data.get("args", {})

            for name, robot in self.robots.items():
                if f"/{name}/navigate_to_pose" in action:
                    pose = args.get("pose", {}).get("pose", {})
                    pos = pose.get("position", {})
                    orient = pose.get("orientation", {})
                    z = orient.get("z", 0.0)
                    w = orient.get("w", 1.0)

                    robot.goal_x = pos.get("x", 0.0)
                    robot.goal_y = pos.get("y", 0.0)
                    robot.goal_theta = 2 * math.atan2(z, w)
                    robot.goal_id = goal_id
                    robot.status = "navigating"
                    print(f"  🎯 {name}: navigating to ({robot.goal_x:.2f}, {robot.goal_y:.2f})")

        elif op == "cancel_action_goal":
            action = data.get("action", "")
            for name, robot in self.robots.items():
                if f"/{name}/" in action:
                    robot.status = "idle"
                    robot.vx = 0
                    robot.vy = 0
                    robot.goal_id = None
                    print(f"  🛑 {name}: cancelled")


# ═══════════════════════════════════════════════════════════
# STATUS DISPLAY
# ═══════════════════════════════════════════════════════════

def status_display(simulator):
    while simulator.running:
        time.sleep(3)
        status_icons = {"idle": "⏸️ ", "navigating": "🚗", "error": "❌"}
        print(f"\n  {'─' * 55}")
        print(f"  {'Robot':<6} {'Position':<22} {'Battery':<10} {'Status'}")
        print(f"  {'─' * 55}")
        for r in simulator.robots.values():
            icon = status_icons.get(r.status, "?")
            pos = f"({r.x:.2f}, {r.y:.2f}, θ={r.theta:.1f})"
            print(f"  {icon} {r.name:<4} {pos:<22} {r.battery:.0f}%       {r.status}")
        print()


# ═══════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════

async def main():
    parser = argparse.ArgumentParser(description="Robo_Fleet - Full Stack Launcher")
    parser.add_argument("--port", type=int, default=9090, help="WebSocket port (default: 9090)")
    parser.add_argument("--with-mcp", action="store_true", help="Also start MCP server")
    parser.add_argument("--test", action="store_true", help="Run integration tests after startup")
    parser.add_argument("--robots", nargs="+", default=["tb1", "tb2", "tb3"])
    args = parser.parse_args()

    print("""
╔══════════════════════════════════════════════════════════╗
║          🤖 Robo_Fleet - Full Stack Launcher             ║
╠══════════════════════════════════════════════════════════╣
║  Components:                                             ║
║    ✓ Mock rosbridge (WebSocket server)                   ║
║    ✓ Fleet simulator (3 robots)                          ║
║    ✓ Pose publisher + navigation handler                 ║
╠══════════════════════════════════════════════════════════╣
║  Press Ctrl+C to stop                                    ║
╚══════════════════════════════════════════════════════════╝
""")

    # Start rosbridge
    # Kill any existing process on the port
    try:
        result = subprocess.run(
            ["lsof", "-ti", f":{args.port}"],
            capture_output=True, text=True
        )
        if result.stdout.strip():
            pids = result.stdout.strip().split("\n")
            for pid in pids:
                subprocess.run(["kill", "-9", pid], capture_output=True)
            print(f"  ⚠️  Killed existing process(es) on port {args.port}")
            import asyncio as _a
            await asyncio.sleep(0.5)
    except Exception:
        pass

    print(f"  🌐 Starting mock rosbridge on ws://0.0.0.0:{args.port}...")
    bridge = MockRosbridge(port=args.port)
    server = await bridge.start()
    print(f"  ✅ rosbridge ready on port {args.port}")

    # Start simulator
    print(f"  🤖 Starting fleet simulator ({', '.join(args.robots)})...")
    simulator = FleetSimulator(robot_names=args.robots, ws_url=f"ws://localhost:{args.port}")
    asyncio.create_task(simulator.run())
    await asyncio.sleep(1)
    print(f"  ✅ Simulator connected")

    # Start status display
    display_thread = threading.Thread(target=status_display, args=(simulator,), daemon=True)
    display_thread.start()

    # Start MCP server if requested
    if args.with_mcp:
        print("  🔌 Starting MCP server (stdio)...")
        import subprocess
        mcp_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mcp_server")
        mcp_proc = subprocess.Popen(
            [sys.executable, "index.py"],
            cwd=mcp_dir,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
        )
        print(f"  ✅ MCP server running (PID: {mcp_proc.pid})")

    # Run tests if requested
    if args.test:
        print("\n  🧪 Running integration tests in 2s...\n")
        await asyncio.sleep(2)
        test_proc = await asyncio.create_subprocess_exec(
            sys.executable, "sim/test_integration.py",
            cwd=os.path.dirname(os.path.abspath(__file__))
        )
        await test_proc.wait()

    print("""
  ═══════════════════════════════════════════════════════
  ✅ Stack is running! You can now:

    • Run tests:     python sim/test_integration.py
    • Start MCP:     cd mcp_server && python index.py
    • Use MCP tool:  navigate_to_pose("tb1", 2.0, 3.0)

  Press Ctrl+C to stop all components.
  ═══════════════════════════════════════════════════════
""")

    # Keep running until Ctrl+C
    try:
        await asyncio.Future()  # Run forever
    except asyncio.CancelledError:
        pass
    finally:
        simulator.running = False
        server.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n  👋 Shutting down...")
