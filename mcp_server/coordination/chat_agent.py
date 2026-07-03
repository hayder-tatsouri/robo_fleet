"""
Fleet Chat Agent - LLM-powered chatbot that controls robots via natural language.
Supports: Anthropic Claude API or AWS Bedrock.

Usage:
    agent = FleetChatAgent(fleet_manager, provider="anthropic", api_key="sk-...")
    response = agent.chat("Send the nearest robot to the warehouse")
"""

import json
import math
import os
import time
from typing import Optional

# Try importing LLM providers
try:
    import anthropic
    HAS_ANTHROPIC = True
except ImportError:
    HAS_ANTHROPIC = False

try:
    import boto3
    HAS_BOTO3 = True
except ImportError:
    HAS_BOTO3 = False


# ═══════════════════════════════════════════════════════════
# TOOL DEFINITIONS (for LLM function calling)
# ═══════════════════════════════════════════════════════════

FLEET_TOOLS = [
    {
        "name": "navigate_robot",
        "description": "Navigate a specific robot to a target position on the map. Use when user says 'send robot to X' or 'move tb1 to position'.",
        "input_schema": {
            "type": "object",
            "properties": {
                "robot_id": {"type": "string", "description": "Robot name (e.g. 'tb1', 'tb3')"},
                "x": {"type": "number", "description": "Target X coordinate in meters"},
                "y": {"type": "number", "description": "Target Y coordinate in meters"},
            },
            "required": ["robot_id", "x", "y"]
        }
    },
    {
        "name": "send_nearest_to_location",
        "description": "Send the nearest available robot to a named location. Locations: origin(0,0), charging_station(0,0), warehouse(1,1), dock(-1,0), entrance(0,-1), storage(-1,1), workstation_a(1,-1), workstation_b(0.5,0.5).",
        "input_schema": {
            "type": "object",
            "properties": {
                "location": {"type": "string", "description": "Named location (e.g. 'warehouse', 'dock', 'charging_station')"},
            },
            "required": ["location"]
        }
    },
    {
        "name": "get_fleet_status",
        "description": "Get current positions, battery levels, and status of all robots in the fleet.",
        "input_schema": {
            "type": "object",
            "properties": {},
        }
    },
    {
        "name": "stop_robot",
        "description": "Immediately stop a specific robot. Use for emergency or when user says 'stop'.",
        "input_schema": {
            "type": "object",
            "properties": {
                "robot_id": {"type": "string", "description": "Robot to stop (e.g. 'tb1'). Use 'all' to stop all robots."},
            },
            "required": ["robot_id"]
        }
    },
    {
        "name": "check_obstacles",
        "description": "Check for obstacles near a robot using laser scan data.",
        "input_schema": {
            "type": "object",
            "properties": {
                "robot_id": {"type": "string", "description": "Robot to check (e.g. 'tb1')"},
            },
            "required": ["robot_id"]
        }
    },
    {
        "name": "assign_tasks",
        "description": "Optimally assign multiple navigation tasks to available robots. The system picks the best robot for each task.",
        "input_schema": {
            "type": "object",
            "properties": {
                "tasks": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "priority": {"type": "integer", "description": "Higher = more urgent"}
                        },
                        "required": ["x", "y"]
                    },
                    "description": "List of target positions"
                }
            },
            "required": ["tasks"]
        }
    },
    {
        "name": "navigate_waypoints",
        "description": "Navigate a robot through a sequence of waypoints in order.",
        "input_schema": {
            "type": "object",
            "properties": {
                "robot_id": {"type": "string", "description": "Robot to send"},
                "waypoints": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {"x": {"type": "number"}, "y": {"type": "number"}},
                        "required": ["x", "y"]
                    }
                }
            },
            "required": ["robot_id", "waypoints"]
        }
    },
]


# ═══════════════════════════════════════════════════════════
# TOOL EXECUTOR
# ═══════════════════════════════════════════════════════════

class ToolExecutor:
    """Executes fleet tools and returns results."""

    def __init__(self, fleet_manager, rosbridge_host="localhost", rosbridge_port=9090):
        self.fleet = fleet_manager
        self.ros_host = rosbridge_host
        self.ros_port = rosbridge_port

    def _get_ros_client(self):
        from ros.ros_client import RosClient
        client = RosClient(host=self.ros_host, port=self.ros_port, max_retries=2)
        client.connect()
        return client

    def execute(self, tool_name, tool_input):
        """Execute a tool and return the result as a string."""
        try:
            if tool_name == "navigate_robot":
                return self._navigate(tool_input["robot_id"], tool_input["x"], tool_input["y"])
            elif tool_name == "send_nearest_to_location":
                return self._send_nearest(tool_input["location"])
            elif tool_name == "get_fleet_status":
                return self._fleet_status()
            elif tool_name == "stop_robot":
                return self._stop(tool_input["robot_id"])
            elif tool_name == "check_obstacles":
                return self._check_obstacles(tool_input["robot_id"])
            elif tool_name == "assign_tasks":
                return self._assign_tasks(tool_input["tasks"])
            elif tool_name == "navigate_waypoints":
                return self._navigate_waypoints(tool_input["robot_id"], tool_input["waypoints"])
            else:
                return json.dumps({"error": f"Unknown tool: {tool_name}"})
        except Exception as e:
            return json.dumps({"error": str(e)})

    def _navigate(self, robot_id, x, y):
        """Send navigation goal - non-blocking. Returns immediately after sending."""
        import threading
        client = self._get_ros_client()
        goal = {
            "pose": {"header": {"frame_id": "map"},
                     "pose": {"position": {"x": x, "y": y, "z": 0.0},
                              "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}
        }
        self.fleet.set_robot_status(robot_id, "navigating", goal_x=x, goal_y=y)
        resp = client.send_goal(f"/{robot_id}/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
        goal_id = resp["goal_id"]

        # Wait for result in background thread (don't block the LLM)
        def _wait():
            try:
                result = client.wait_for_result(f"/{robot_id}/navigate_to_pose", goal_id, timeout=60)
                client.disconnect()
                self.fleet.set_robot_status(robot_id, "idle")
            except:
                self.fleet.set_robot_status(robot_id, "idle")

        threading.Thread(target=_wait, daemon=True).start()
        return json.dumps({"success": True, "robot_id": robot_id, "target": {"x": x, "y": y},
                          "status": "navigating", "goal_id": goal_id,
                          "message": f"{robot_id} is now navigating to ({x:.2f}, {y:.2f})"})

    def _send_nearest(self, location):
        from tools.natural_language import _load_locations
        locations = _load_locations()
        loc = locations.get(location.lower().replace(" ", "_"))
        if not loc:
            return json.dumps({"error": f"Unknown location: {location}. Available: {list(locations.keys())}"})

        robot = self.fleet.get_nearest_available(loc["x"], loc["y"])
        if not robot:
            return json.dumps({"error": "No available robots"})

        return self._navigate(robot.robot_id, loc["x"], loc["y"])

    def _fleet_status(self):
        states = self.fleet.get_all_states()
        return json.dumps({"robots": states, "total": len(states),
                          "online": sum(1 for s in states if s["online"])})

    def _stop(self, robot_id):
        client = self._get_ros_client()
        if robot_id == "all":
            for rid in self.fleet.robots:
                client.publish(f"/{rid}/cmd_vel", "geometry_msgs/msg/Twist",
                             {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}})
                self.fleet.set_robot_status(rid, "idle")
            client.disconnect()
            return json.dumps({"success": True, "stopped": list(self.fleet.robots.keys())})
        else:
            client.publish(f"/{robot_id}/cmd_vel", "geometry_msgs/msg/Twist",
                         {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}})
            self.fleet.set_robot_status(robot_id, "idle")
            client.disconnect()
            return json.dumps({"success": True, "stopped": robot_id})

    def _check_obstacles(self, robot_id):
        client = self._get_ros_client()
        msg = client.subscribe_once(f"/{robot_id}/scan", "sensor_msgs/msg/LaserScan", timeout=3.0)
        client.disconnect()
        if not msg:
            return json.dumps({"error": "No scan data"})
        ranges = msg.get("ranges", [])
        valid = [r for r in ranges if 0.12 <= r <= 10.0]
        closest = min(valid) if valid else None
        return json.dumps({"robot_id": robot_id, "closest_obstacle_m": closest,
                          "alert": closest < 0.5 if closest else False})

    def _assign_tasks(self, tasks):
        """Assign AND dispatch tasks in parallel - all robots move simultaneously."""
        from coordination.task_planner import TaskPlanner
        planner = TaskPlanner(self.fleet)
        task_objs = [planner.create_task(x=t["x"], y=t["y"], priority=t.get("priority", 0)) for t in tasks]
        assignments = planner.allocate(task_objs)

        results = []
        for a in assignments:
            # Send each robot navigating (non-blocking)
            nav_result = json.loads(self._navigate(a.robot_id, a.task.x, a.task.y))
            results.append({
                "robot_id": a.robot_id,
                "target": {"x": a.task.x, "y": a.task.y},
                "cost": round(a.cost, 2),
                "status": "navigating"
            })

        return json.dumps({
            "assigned": len(results),
            "all_robots_moving": True,
            "assignments": results,
            "message": f"Dispatched {len(results)} robots simultaneously"
        })

    def _navigate_waypoints(self, robot_id, waypoints):
        """Navigate through waypoints - runs in background thread so LLM isn't blocked."""
        import threading

        def _run_waypoints():
            try:
                client = self._get_ros_client()
                completed = 0
                for wp in waypoints:
                    goal = {"pose": {"header": {"frame_id": "map"},
                                     "pose": {"position": {"x": wp["x"], "y": wp["y"], "z": 0.0},
                                              "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}}
                    self.fleet.set_robot_status(robot_id, "navigating", goal_x=wp["x"], goal_y=wp["y"])
                    resp = client.send_goal(f"/{robot_id}/navigate_to_pose", "nav2_msgs/action/NavigateToPose", goal)
                    result = client.wait_for_result(f"/{robot_id}/navigate_to_pose", resp["goal_id"], timeout=60)
                    if result and result.get("success"):
                        completed += 1
                    else:
                        break
                client.disconnect()
                self.fleet.set_robot_status(robot_id, "idle")
            except:
                self.fleet.set_robot_status(robot_id, "idle")

        threading.Thread(target=_run_waypoints, daemon=True).start()
        return json.dumps({
            "robot_id": robot_id,
            "waypoints": len(waypoints),
            "status": "navigating",
            "message": f"{robot_id} is now following {len(waypoints)} waypoints"
        })


# ═══════════════════════════════════════════════════════════
# CHAT AGENT
# ═══════════════════════════════════════════════════════════

SYSTEM_PROMPT = """You are the Robo_Fleet AI assistant controlling a fleet of TurtleBot3 robots.

Available robots: {robots}
Map bounds: -1.5m to 1.5m (x and y)

Named locations: origin(0,0), charging_station(0,0), warehouse(1,1), dock(-1,0), entrance(0,-1), storage(-1,1), workstation_a(1,-1), workstation_b(0.5,0.5)

You can:
- Navigate robots to positions or named locations
- Check fleet status and battery levels
- Stop robots in emergencies
- Check for obstacles
- Assign multiple tasks optimally
- Navigate robots through waypoint sequences

Be concise. After executing a command, confirm what happened.
Keep coordinates within map bounds (-1.3 to 1.3).
"""


class FleetChatAgent:
    """LLM-powered chat agent for fleet control."""

    def __init__(self, fleet_manager, provider="anthropic", api_key=None,
                 model=None, rosbridge_host="localhost", rosbridge_port=9090):
        """
        Args:
            fleet_manager: FleetStateManager instance
            provider: "anthropic" or "bedrock"
            api_key: API key (for Anthropic). For Bedrock, uses AWS credentials.
            model: Model name. Default: claude-sonnet-4-20250514 (Anthropic) or anthropic.claude-sonnet-4-20250514-v1:0 (Bedrock)
            rosbridge_host: rosbridge host for tool execution
            rosbridge_port: rosbridge port
        """
        self.fleet = fleet_manager
        self.provider = provider
        self.executor = ToolExecutor(fleet_manager, rosbridge_host, rosbridge_port)
        self.messages = []  # Conversation history

        if provider == "anthropic":
            if not HAS_ANTHROPIC:
                raise ImportError("pip install anthropic")
            self.api_key = api_key or os.environ.get("ANTHROPIC_API_KEY", "")
            self.model = model or "claude-sonnet-4-20250514"
            self.client = anthropic.Anthropic(api_key=self.api_key)

        elif provider == "bedrock":
            if not HAS_BOTO3:
                raise ImportError("pip install boto3")
            self.model = model or "anthropic.claude-sonnet-4-20250514-v1:0"
            self.client = boto3.client("bedrock-runtime", region_name=os.environ.get("AWS_REGION", "us-east-1"))

        else:
            raise ValueError(f"Unknown provider: {provider}. Use 'anthropic' or 'bedrock'.")

    def chat(self, user_message):
        """
        Send a message and get a response (with tool use if needed).
        Returns the assistant's text response.
        """
        self.messages.append({"role": "user", "content": user_message})

        # Build system prompt with current robot info
        robot_info = ", ".join(f"{r.robot_id}({r.x:.1f},{r.y:.1f})" for r in self.fleet.robots.values())
        system = SYSTEM_PROMPT.format(robots=robot_info)

        # Call LLM
        response = self._call_llm(system)

        # Handle tool use loop
        max_loops = 5
        loops = 0
        while loops < max_loops:
            loops += 1

            # Check if response has tool calls
            tool_calls = self._extract_tool_calls(response)
            if not tool_calls:
                break

            # Execute tools
            tool_results = []
            for tc in tool_calls:
                result = self.executor.execute(tc["name"], tc["input"])
                tool_results.append({"tool_use_id": tc["id"], "content": result})

            # Send tool results back to LLM
            self.messages.append({"role": "assistant", "content": response["content"]})
            self.messages.append({"role": "user", "content": [
                {"type": "tool_result", "tool_use_id": tr["tool_use_id"], "content": tr["content"]}
                for tr in tool_results
            ]})

            response = self._call_llm(system)

        # Extract final text
        text = self._extract_text(response)
        self.messages.append({"role": "assistant", "content": text})
        return text

    def _call_llm(self, system):
        """Call the LLM provider."""
        if self.provider == "anthropic":
            response = self.client.messages.create(
                model=self.model,
                max_tokens=1024,
                system=system,
                tools=FLEET_TOOLS,
                messages=self.messages,
            )
            return {"content": response.content, "stop_reason": response.stop_reason}

        elif self.provider == "bedrock":
            # Bedrock InvokeModel with anthropic_version uses Anthropic Messages API format
            # but tools need "type": "custom" wrapping for newer Bedrock models
            bedrock_tools = []
            for t in FLEET_TOOLS:
                bedrock_tools.append({
                    "name": t["name"],
                    "description": t["description"],
                    "input_schema": t["input_schema"],
                })

            body = {
                "anthropic_version": "bedrock-2023-05-31",
                "max_tokens": 1024,
                "system": [{"type": "text", "text": system}],
                "tools": bedrock_tools,
                "messages": self.messages,
            }

            resp = self.client.invoke_model(
                modelId=self.model,
                body=json.dumps(body),
                contentType="application/json",
                accept="application/json",
            )
            result = json.loads(resp["body"].read())
            return {"content": result.get("content", []), "stop_reason": result.get("stop_reason")}

    def _extract_tool_calls(self, response):
        """Extract tool_use blocks from response."""
        calls = []
        content = response.get("content", [])
        if isinstance(content, str):
            return []
        for block in content:
            if hasattr(block, "type") and block.type == "tool_use":
                calls.append({"id": block.id, "name": block.name, "input": block.input})
            elif isinstance(block, dict) and block.get("type") == "tool_use":
                calls.append({"id": block["id"], "name": block["name"], "input": block["input"]})
        return calls

    def _extract_text(self, response):
        """Extract text from response."""
        content = response.get("content", [])
        if isinstance(content, str):
            return content
        texts = []
        for block in content:
            if hasattr(block, "type") and block.type == "text":
                texts.append(block.text)
            elif isinstance(block, dict) and block.get("type") == "text":
                texts.append(block["text"])
        return " ".join(texts) if texts else "Done."

    def reset(self):
        """Clear conversation history."""
        self.messages = []
