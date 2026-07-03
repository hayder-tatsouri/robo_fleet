# 🤖 Robo_Fleet - MCP Server for ROS2 Robot Fleet Control

An MCP (Model Context Protocol) server that exposes ROS2 multi-robot operations as AI-callable tools. Enables LLM agents to control a fleet of TurtleBot3 robots via natural language.

## Architecture

```
┌─────────────┐    stdio     ┌───────────────┐   WebSocket    ┌────────────┐
│  AI Agent   │◄────────────►│  MCP Server   │◄──────────────►│  rosbridge │
│  (Claude)   │   FastMCP    │  (Python 3.12)│   ws://9090    │   (ROS2)   │
└─────────────┘              └───────────────┘                └────────────┘
                                                                    │
                                                          ┌─────────┼─────────┐
                                                          │         │         │
                                                        [tb1]    [tb2]    [tb3]
                                                      TurtleBot3 fleet (Gazebo)
```

## Prerequisites

| Component | Version | Purpose |
|-----------|---------|---------|
| Python | 3.10+ | MCP server runtime |
| ROS2 | Humble/Iron | Robot middleware |
| Gazebo | 11+ | Robot simulation |
| rosbridge_suite | latest | WebSocket-ROS2 bridge |
| Nav2 | latest | Autonomous navigation |
| TurtleBot3 | latest | Robot simulation package |

## Quick Start

### 1. Environment Setup

```bash
chmod +x setup.sh
./setup.sh
```

Or manually:
```bash
cd mcp_server
python3 -m venv mcp_env
source mcp_env/bin/activate
pip install -r ../requirements.txt
```

### 2. Launch ROS2 Simulation

Terminal 1 - Launch multi-robot world:
```bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch tb3_multi_robot multi_robot_world.launch.py
```

Terminal 2 - Start rosbridge:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 3. Run Tests

```bash
# Unit tests (no ROS needed)
pytest tests/ -v

# Integration test (requires rosbridge running)
cd mcp_server
python tools/exemples.py
```

### 4. Start MCP Server

```bash
cd mcp_server
python index.py
```

### 5. Connect to AI Agent

Add to your MCP client config (e.g. Claude Desktop):
```json
{
  "mcpServers": {
    "robots_mcp": {
      "command": "python",
      "args": ["mcp_server/index.py"],
      "cwd": "/path/to/Robo_Fleet"
    }
  }
}
```

## Available MCP Tools

| Tool | Description | Parameters |
|------|-------------|------------|
| `navigate_to_pose` | Navigate robot to (x, y, theta) | robot_id, x, y, theta, frame_id, timeout |
| `monitoring` | *(planned)* Fleet status | - |

## Project Structure

```
Robo_Fleet/
├── mcp_server/
│   ├── index.py           # Entry point
│   ├── server.py          # FastMCP initialization
│   ├── ros/
│   │   └── ros_client.py  # WebSocket rosbridge client
│   └── tools/
│       ├── navigation.py  # navigate_to_pose tool
│       ├── monitoring.py  # (planned) fleet monitoring
│       └── exemples.py    # Integration tests
├── robot_ws/
│   └── src/
│       └── tb3_multi_robot  # ROS2 multi-robot package (submodule)
├── tests/
│   └── test_ros_client.py   # pytest unit tests
├── requirements.txt
├── setup.sh
└── README.md
```

## Example AI Interaction

```
User: "Move robot tb1 to position (2.0, 3.0) facing north"

AI calls: navigate_to_pose(robot_id="tb1", x=2.0, y=3.0, theta=1.5708)

Response: {"success": true, "status": 4, "goal_id": "goal_a1b2c3d4"}
```

## Troubleshooting

| Issue | Fix |
|-------|-----|
| `Connection refused ws://localhost:9090` | Start rosbridge: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| `No message received (timeout)` | Ensure Nav2 is running: `ros2 launch nav2_bringup navigation_launch.py` |
| `Goal status != 4` | Check costmap - robot may be stuck. Try different coordinates. |

## License

MIT
