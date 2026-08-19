# Robo_Fleet - Setup Guide

Complete setup instructions for the Robo_Fleet multi-robot fleet control system: a
ROS 2 / Gazebo simulation layer (PearlGuard outdoor platform) plus an MCP server that
exposes fleet operations to AI agents.

---

## Overview

Two layers:

1. **Simulation** — a ROS 2 Jazzy + Gazebo Harmonic + Nav2 outdoor world running two
   PearlGuard robots. Runs inside a Docker container (headless-safe) or natively on
   Ubuntu 24.04.
2. **MCP + dashboard** — an MCP server (`mcp_server/`) and a live web dashboard that the
   agents drive through natural language.

---

## Prerequisites

| Requirement | Version | Notes |
|-------------|---------|-------|
| ROS 2 Jazzy + Gazebo Harmonic + Nav2 | Jazzy | Docker image or native Ubuntu 24.04 |
| Python | 3.10+ | For the MCP layer / dashboard |
| pip | Latest | `pip install --upgrade pip` |
| Git | Any | Version control |
| Anthropic API key (optional) | — | For the AI chat agent (or AWS Bedrock) |

---

## Option A — Docker (recommended for headless / CI)

Build the extended image once (~15 min):

```bash
docker build -t outdoor-sim:jazzy .
```

Open a shell in the container:

```bash
./scripts/ros2-shell.sh
# or directly:
docker run --rm -it --network host -v "$PWD":/workspace -w /workspace \
    outdoor-sim:jazzy bash
```

See [`README_outdoor_sim.md`](README_outdoor_sim.md) for details and GUI limitations.

---

## Option B — Native Ubuntu 24.04

One-time machine installer (idempotent, requires sudo):

```bash
./scripts/setup_ubuntu_24_04.sh            # full setup (apt + ROS2 + Gazebo + Python)
./scripts/setup_ubuntu_24_04.sh --no-build # skip the colcon build
./scripts/setup_ubuntu_24_04.sh --check    # verify the install only
```

This installs ROS 2 Jazzy Desktop, Nav2, `robot_localization`, rosbridge, Foxglove,
Gazebo Harmonic, and the Python deps for the MCP server + dashboard.

---

## Build the ROS 2 workspace (native only)

```bash
colcon build --symlink-install
source install/setup.bash
```

(In Docker, the build step is handled inside the image / `--no-build` skips it.)

---

## Python environment (MCP layer + dashboard)

The MCP/dashboard code runs from a plain venv — a system-wide ROS 2 install is **not**
required for this layer.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

Optional extras for the AI chat agent:

```bash
pip install boto3       # AWS Bedrock provider
pip install anthropic   # Anthropic API provider
pip install scipy       # optimal (Hungarian) task allocation; greedy fallback works without it
```

---

## Quick Start

### 1. Launch the simulation + navigation

```bash
ros2 launch my_pguard_bot full_stack.launch.py
```

Starts Gazebo with the Novation City world, spawns **pearlguard1** and **pearlguard2**,
runs the dual-EKF localizers, and brings up Nav2 for each robot.

### 2. Launch rosbridge + the topic adapter

```bash
ros2 launch my_pguard_bot robofleet.launch.py
```

Starts the **rosbridge WebSocket server on port 9090** plus the `robo_fleet_adapter`
that bridges the robot topics to the fleet layer.

### 3. Launch the dashboard + AI chat

```bash
python start_dashboard.py --rosbridge localhost --robots pearlguard1 pearlguard2 --open
```

Open `dashboard/live_dashboard.html` in your browser (auto-opened with `--open`). Click
the map to send robots to a position, or use the chat panel to drive the fleet in
natural language.

### 4. (Optional) Visualization

```bash
ros2 launch my_pguard_bot viz.launch.py   # Foxglove bridge, port 8765
ros2 launch my_pguard_bot patrol.launch.py  # GPS perimeter patrol client
```

---

## Other Launch Files

| Launch file | What it starts |
|-------------|----------------|
| `full_stack.launch.py` | **Main entry**: sim + dual-EKF localization + Nav2 for both robots |
| `sim.launch.py` | Gazebo (headless by default, `use_gui:=true` for GUI), state publisher, spawn, `ros_gz_bridge` |
| `localization.launch.py` | Dual-EKF outdoor localizer (odom + IMU + RTK GPS) per robot |
| `robofleet.launch.py` | Static `map → odom` transforms, topic adapter, rosbridge on port 9090 |
| `patrol.launch.py` | GPS perimeter patrol client |
| `viz.launch.py` | Foxglove bridge for browser visualization (port 8765) |

---

## MCP Server

### Start the MCP server (stdio)

```bash
cd mcp_server
python index.py
```

### Start the MCP server (streamable-HTTP)

```bash
cd mcp_server
python index.py --transport http --host 0.0.0.0 --port 8766
```

### Connect an AI client

Add to your MCP client config (Claude Desktop, Cursor, LangGraph Studio, etc.):

```json
{
  "mcpServers": {
    "robots_mcp": {
      "command": "/path/to/Robo_Fleet/.venv/bin/python",
      "args": ["/path/to/Robo_Fleet/mcp_server/index.py"]
    }
  }
}
```

### LangGraph (multi-agent graph)

The supervisor→agents graph is defined in `mcp_server/langgraph.json` and built in
`mcp_server/graph/graph.py`. Run it in [LangGraph Studio](https://studio.langchain.com)
or behind the dashboard. Requires `ANTHROPIC_API_KEY` (or Bedrock credentials) to
instantiate the agents.

### Available MCP tools (32 total)

| Category | Tools |
|----------|-------|
| Navigation | `navigate_to_pose` |
| Waypoints | `navigate_waypoints` |
| Monitoring | `list_capabilities`, `get_robot_position`, `get_fleet_status`, `get_battery_level` |
| Obstacles | `check_obstacles` |
| Map | `get_map_with_robots` |
| Control | `stop_robot`, `emergency_stop` |
| Coordination | `assign_tasks`, `dispatch_tasks`, `get_plan`, `replan`, `set_robot_priority`, `configure_fleet` |
| Natural language | `go_to_location`, `send_nearest_to`, `add_location`, `remove_location`, `list_locations` |
| Advanced | `predict_collisions`, `add_task_to_queue`, `get_queue`, `clear_queue`, `start_auto_dispatch`, `stop_auto_dispatch`, `assign_tasks_optimal`, `start_dashboard`, `stop_dashboard` |
| Chat | `robot_chat` |

---

## Testing

### Unit tests (no simulation or network required)

```bash
pytest tests/ -v
```

| File | Coverage |
|------|----------|
| `tests/test_ros_client.py` | RosClient WebSocket: connect/publish/subscribe/actions |
| `tests/test_new_tools.py` | Monitoring, control, waypoints, obstacles, map viz |
| `tests/test_next_steps.py` | Collision predictor, task queue, dashboard, natural language, Hungarian allocation |

---

## Dashboard

Served by `start_dashboard.py` on `ws://localhost:8080`; open
`dashboard/live_dashboard.html`. It streams live fleet state and provides the chat
panel that talks to the MCP agents.

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `Address already in use (9090)` | Kill the previous rosbridge: `lsof -ti :9090 \| xargs kill -9` |
| `Address already in use (8080)` | Kill the previous dashboard: `lsof -ti :8080 \| xargs kill -9` |
| Gazebo GUI not starting | Use headless mode (`-s`) or run natively with a GPU |
| `ModuleNotFoundError: boto3` | `pip install boto3` |
| `ModuleNotFoundError: anthropic` | `pip install anthropic` |
| `scipy not found` | `pip install scipy` (optional — greedy fallback works) |
| Chat agent disabled at startup | Set `ANTHROPIC_API_KEY` or use `--provider bedrock` |
| Dashboard shows "Disconnected" | Start `start_dashboard.py` first, then open the HTML |

---

## Network Configuration

| Service | Default Port | Configurable |
|---------|-------------|--------------|
| rosbridge (ROS2) | 9090 | `--port` |
| Dashboard WebSocket | 8080 | `--dashboard-port` |
| MCP server (HTTP) | 8766 | `--port` (with `--transport http`) |
| Foxglove bridge | 8765 | `viz.launch.py` |

---

## Project Structure

```
robo_fleet/
├── mcp_server/                 # MCP server: agents, tools, coordination, graph
│   ├── server.py / index.py    # FastMCP instance + entry point
│   ├── agents/                 # ReAct agents + system prompts
│   ├── graph/                  # supervisor -> agents graph
│   ├── tools/                  # @mcp.tool() implementations
│   ├── ros/                    # rosbridge WebSocket client
│   └── coordination/           # fleet state, planner, queue, collision, dashboard
├── src/
│   ├── my_pguard_bot/          # world, maps, configs, launch files, scripts
│   └── pearlguard_description/ # real PearlGuard model (meshes, URDF)
├── dashboard/                  # live web dashboard
├── tests/                      # pytest unit tests (mock-based)
├── start_dashboard.py          # dashboard + chat agent launcher
├── scripts/setup_ubuntu_24_04.sh  # native Ubuntu 24.04 installer
├── setup.sh                    # venv + Python deps for MCP/dashboard layer
├── requirements.txt
├── pytest.ini
└── README.md
```

---

*Updated 2026-08-19*