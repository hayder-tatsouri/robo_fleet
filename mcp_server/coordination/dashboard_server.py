"""
Live Fleet Dashboard - WebSocket server that streams fleet state to browser.
Runs on port 8080. Connects to FleetStateManager and pushes updates at 5Hz.
"""

import asyncio
import json
import math
import time
import threading
import os

try:
    import websockets
    import websockets.server
    import websocket as ws_client
    import uuid
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False


class DashboardServer:
    """WebSocket server that streams fleet state to connected browsers."""

    def __init__(self, fleet_manager, host="0.0.0.0", port=8080, update_rate=5):
        """
        Args:
            fleet_manager: FleetStateManager instance
            host: Bind address
            port: WebSocket port
            update_rate: Updates per second (Hz)
        """
        self.fleet = fleet_manager
        self.host = host
        self.port = port
        self.update_interval = 1.0 / update_rate
        self.clients = set()
        self._running = False
        self._server = None
        self._thread = None
        self.rosbridge_url = "ws://localhost:9090"
        self.chat_agent = None  # Set externally if LLM is configured
        self._command_websocket = None  # Track websocket that sent command

    def start(self):
        """Start dashboard server in background thread."""
        if not HAS_WEBSOCKETS:
            return {"success": False, "error": "websockets package not installed"}
        if self._running:
            return {"success": True, "status": "already_running", "url": f"ws://{self.host}:{self.port}"}

        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        # Get the HTML path
        html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dashboard", "live_dashboard.html")

        return {
            "success": True,
            "status": "started",
            "ws_url": f"ws://localhost:{self.port}",
            "dashboard_html": os.path.abspath(html_path) if os.path.exists(html_path) else None,
            "message": f"Dashboard streaming on ws://localhost:{self.port}. Open live_dashboard.html in browser.",
        }

    def stop(self):
        """Stop the dashboard server."""
        self._running = False
        return {"success": True, "status": "stopped"}

    def _run_loop(self):
        """Run the asyncio event loop in a thread."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._serve())

    async def _serve(self):
        """Main server coroutine."""
        async with websockets.serve(self._handler, self.host, self.port):
            # Start broadcast loop
            broadcast_task = asyncio.create_task(self._broadcast_loop())
            while self._running:
                await asyncio.sleep(0.5)
            broadcast_task.cancel()

    async def _handler(self, websocket):
        """Handle new WebSocket connection."""
        self.clients.add(websocket)
        try:
            async for message in websocket:
                # Handle commands from dashboard (e.g., send goal)
                try:
                    data = json.loads(message)
                    await self._handle_command(data)
                except json.JSONDecodeError:
                    pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)

    async def _broadcast_loop(self):
        """Broadcast fleet state to all connected clients."""
        while self._running:
            if self.clients:
                state = self._get_fleet_state()
                message = json.dumps(state)
                disconnected = set()
                for client in self.clients.copy():
                    try:
                        await client.send(message)
                    except:
                        disconnected.add(client)
                self.clients -= disconnected
            await asyncio.sleep(self.update_interval)

    def _get_fleet_state(self):
        """Build fleet state JSON for dashboard."""
        robots = []
        for robot in self.fleet.robots.values():
            robots.append({
                "id": robot.robot_id,
                "x": round(robot.x, 3),
                "y": round(robot.y, 3),
                "theta": round(robot.theta, 3),
                "battery": round(robot.battery, 1),
                "status": robot.status,
                "online": robot.is_online,
                "goal": {"x": robot.goal_x, "y": robot.goal_y} if robot.goal_x is not None else None,
            })

        return {
            "type": "fleet_state",
            "timestamp": time.time(),
            "robots": robots,
        }

    async def _handle_command(self, data):
        """Handle commands from the dashboard."""
        cmd = data.get("command")

        if cmd == "navigate":
            robot_id = data.get("robot_id")
            x = data.get("x", 0)
            y = data.get("y", 0)
            # Dispatch navigation via rosbridge (non-blocking)
            if robot_id and self.fleet.robots.get(robot_id):
                self.fleet.set_robot_status(robot_id, "navigating", goal_x=x, goal_y=y)
                # Send actual Nav2 goal in background
                threading.Thread(
                    target=self._send_nav_goal,
                    args=(robot_id, x, y),
                    daemon=True
                ).start()

        elif cmd == "chat":
            message = data.get("message", "")
            if self.chat_agent and message:
                # Run chat in background thread to avoid blocking
                threading.Thread(
                    target=self._handle_chat,
                    args=(message,),
                    daemon=True
                ).start()
            elif not self.chat_agent:
                # No LLM configured - send error
                await self._broadcast_chat_response(
                    "LLM not configured. Set ANTHROPIC_API_KEY or use --provider bedrock.", None)

    async def _broadcast_chat_response(self, message, tool_used):
        """Send chat response to all connected clients."""
        payload = json.dumps({"type": "chat_response", "message": message, "tool_used": tool_used})
        for client in self.clients.copy():
            try:
                await client.send(payload)
            except:
                pass

    def _send_nav_goal(self, robot_id, x, y):
        """Send a Nav2 NavigateToPose goal via rosbridge."""
        try:
            from ros.ros_client import RosClient
            # Parse host/port from rosbridge_url
            url = self.rosbridge_url  # e.g. ws://192.168.0.8:9090
            host = url.replace("ws://", "").split(":")[0]
            port = int(url.replace("ws://", "").split(":")[1]) if ":" in url.replace("ws://", "") else 9090

            client = RosClient(host=host, port=port, max_retries=2)
            client.connect()

            goal = {
                "pose": {
                    "header": {"frame_id": "map"},
                    "pose": {
                        "position": {"x": x, "y": y, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    }
                }
            }

            resp = client.send_goal(
                action=f"/{robot_id}/navigate_to_pose",
                action_type="nav2_msgs/action/NavigateToPose",
                goal=goal,
            )
            goal_id = resp["goal_id"]

            result = client.wait_for_result(
                f"/{robot_id}/navigate_to_pose", goal_id, timeout=60.0
            )
            client.disconnect()
            self.fleet.set_robot_status(robot_id, "idle")

        except Exception as e:
            print(f"  Dashboard nav error: {e}")
            self.fleet.set_robot_status(robot_id, "idle")

    def _handle_chat(self, message):
        """Handle chat message in background thread."""
        try:
            response = self.chat_agent.chat(message)
            # Broadcast response to all clients
            import asyncio
            loop = asyncio.new_event_loop()
            loop.run_until_complete(self._broadcast_chat_response(response, None))
            loop.close()
        except Exception as e:
            import asyncio
            loop = asyncio.new_event_loop()
            loop.run_until_complete(self._broadcast_chat_response(f"Error: {e}", None))
            loop.close()


# Singleton instance
_dashboard = None


def get_dashboard(fleet_manager=None, port=8080):
    """Get or create dashboard server singleton."""
    global _dashboard
    if _dashboard is None and fleet_manager is not None:
        _dashboard = DashboardServer(fleet_manager, port=port)
    return _dashboard
