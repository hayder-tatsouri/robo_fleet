# Robo_Fleet - Setup Guide

Complete setup instructions for the Robo_Fleet MCP Server for Multi-Robot Fleet Control.

---

## Prerequisites

| Requirement | Version | Notes |
|-------------|---------|-------|
| Python | 3.10+ | Tested on 3.13.2 (macOS) |
| pip | Latest | `pip install --upgrade pip` |
| Git | Any | For version control |
| AWS Account (optional) | Bedrock access | For AI chat (Claude) |

**No ROS2, Docker, or conda required** for development and testing.

---

## Quick Start (2 minutes)

```bash
# Clone or navigate to the project
cd ~/Downloads/Robo_Fleet

# Run setup
chmod +x setup.sh
./setup.sh

# Activate environment
source .venv/bin/activate

# Start everything (mock rosbridge + 3 simulated robots)
python run.py
```

In a second terminal:
```bash
cd ~/Downloads/Robo_Fleet
source .venv/bin/activate
python sim/test_integration.py
```

Expected output: **12/12 tests passed**

---

## Detailed Setup

### 1. Create Virtual Environment

```bash
cd ~/Downloads/Robo_Fleet
python3 -m venv .venv
source .venv/bin/activate
```

### 2. Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

Core dependencies installed:
- `mcp[cli]` - Model Context Protocol server
- `websocket-client` - ROS bridge communication
- `websockets` - Mock rosbridge server
- `pydantic` - Data validation
- `numpy` - Math operations
- `pytest` - Testing

### 3. Install Optional Dependencies

```bash
# For AI chat agent (Bedrock)
pip install boto3

# For AI chat agent (Anthropic direct)
pip install anthropic

# For optimal task allocation (Hungarian algorithm)
pip install scipy
```

---

## Running Modes

### Mode A: Local Simulator (no hardware needed)

Starts a mock rosbridge + 3 simulated robots with physics.

```bash
python run.py
```

What starts:
- Mock rosbridge WebSocket server on `ws://0.0.0.0:9090`
- Fleet simulator (tb1, tb2, tb3) with pose, battery, and laser scan publishing
- Physics loop at 20Hz

Options:
```bash
python run.py --port 9090          # Custom port
python run.py --robots tb1 tb2 tb3 tb4 tb5  # More robots
python run.py --test               # Auto-run integration tests after startup
```

### Mode B: Real Rosbridge (remote robots)

Connect to a real ROS2 + rosbridge setup on your network.

```bash
# Verify connectivity first
python -c "
import websocket
ws = websocket.create_connection('ws://192.168.0.8:9090', timeout=5)
print('Connected')
ws.close()
"

# Discover what topics exist
python sim/diagnose_remote.py --host 192.168.0.8
```

### Mode C: Live Dashboard with AI Chat

```bash
# Kill any old dashboard process
lsof -ti :8080 | xargs kill -9 2>/dev/null

# With AWS Bedrock (uses your AWS credentials)
AWS_PROFILE=sandbox AWS_REGION=us-east-1 python start_dashboard.py \
  --rosbridge 192.168.0.8 \
  --robots tb1 tb3 \
  --provider bedrock \
  --model anthropic.claude-3-haiku-20240307-v1:0 \
  --open

# With Anthropic API directly
ANTHROPIC_API_KEY="sk-ant-..." python start_dashboard.py \
  --rosbridge 192.168.0.8 \
  --robots tb1 tb3 \
  --open
```

Then open `dashboard/live_dashboard.html` in your browser.

### Mode D: Docker (full ROS2 stack)

```bash
docker compose up
```

Requires Docker Desktop running. Pulls `osrf/ros:humble-desktop-full` (~3GB first time).

---

## MCP Server

### Start the MCP Server (stdio)

```bash
cd mcp_server
python index.py
```

### Connect to an AI Client

Add to your MCP client config (Claude Desktop, Cursor, etc.):

```json
{
  "mcpServers": {
    "robots_mcp": {
      "command": "/Users/tastouri/Downloads/Robo_Fleet/.venv/bin/python",
      "args": ["/Users/tastouri/Downloads/Robo_Fleet/mcp_server/index.py"]
    }
  }
}
```

### Available MCP Tools (26 total)

| Category | Tools |
|----------|-------|
| Navigation | `navigate_to_pose`, `navigate_waypoints` |
| Monitoring | `get_robot_position`, `get_fleet_status`, `get_battery_level`, `check_obstacles`, `get_map_with_robots` |
| Control | `stop_robot`, `emergency_stop` |
| Coordination | `assign_tasks`, `dispatch_tasks`, `get_plan`, `replan`, `set_robot_priority`, `configure_fleet` |
| Natural Language | `go_to_location`, `send_nearest_to`, `add_location`, `remove_location`, `list_locations` |
| Advanced | `predict_collisions`, `add_task_to_queue`, `get_queue`, `clear_queue`, `start_auto_dispatch`, `stop_auto_dispatch`, `assign_tasks_optimal`, `start_dashboard`, `stop_dashboard` |

---

## Testing

### Unit Tests (no network required)

```bash
# All unit tests
pytest tests/ -v

# Specific test files
pytest tests/test_ros_client.py -v       # 12 tests - core client
pytest tests/test_new_tools.py -v        # 12 tests - monitoring/control
pytest tests/test_next_steps.py -v       # 25 tests - advanced features
```

### Integration Tests (requires `python run.py` in another terminal)

```bash
python sim/test_integration.py           # 12 tests against local simulator
python sim/test_coordination.py          # 8 tests for fleet coordination
```

### Remote Tests (requires real rosbridge)

```bash
# Basic remote test
python sim/test_remote.py --host 192.168.0.8 --robots tb1 tb3

# Advanced features on real hardware
python sim/test_new_features_remote.py --host 192.168.0.8 --robots tb1 tb3

# Full scenarios (long nav, waypoints, simultaneous, dispatch, return home)
python sim/test_full_scenario.py --host 192.168.0.8 --robots tb1 tb3 --nav-timeout 90

# Coordination (fleet state, allocation, collision, dispatch, groups)
python sim/test_coordination.py --host 192.168.0.8 --robots tb1 tb3
```

### Scale Benchmark

```bash
python sim/scale_test.py --robots 10    # Benchmark at 10 robots
python sim/scale_test.py --robots 20    # Benchmark at 20 robots
```

### Diagnostic Tool

```bash
# Discover topics on a remote rosbridge
python sim/diagnose_remote.py --host 192.168.0.8
```

---

## AWS Bedrock Setup (for AI Chat)

### 1. Get AWS Credentials

```bash
ada credentials update --account 730335219206 --provider isengard --role Administrator --profile sandbox --once
```

### 2. Verify Model Access

```bash
aws bedrock list-foundation-models --profile sandbox --region us-east-1 \
  --query "modelSummaries[?contains(modelId, 'claude')].[modelId]" --output table
```

### 3. Environment Variables

```bash
export AWS_PROFILE=sandbox
export AWS_REGION=us-east-1
```

### Available Models

| Model | Speed | Cost | Best For |
|-------|-------|------|----------|
| `anthropic.claude-3-haiku-20240307-v1:0` | Fast | Low | Quick robot commands |
| `anthropic.claude-sonnet-4-20250514-v1:0` | Medium | Medium | Complex coordination |

---

## Project Structure

```
Robo_Fleet/
├── run.py                          # One-command launcher
├── start_dashboard.py              # Dashboard + AI chat launcher
├── setup.sh                        # Environment setup
├── requirements.txt                # Python dependencies
├── PROJECT_STATUS.md               # Current status + roadmap
├── README.md                       # Project overview
├── features.html                   # Visual feature tracker
├── docker-compose.yml              # Docker setup
├── Dockerfile.mcp                  # MCP server container
├── pytest.ini                      # Test config
├── .gitignore
│
├── mcp_server/                     # MCP Server (core)
│   ├── server.py                   # FastMCP initialization
│   ├── index.py                    # Tool registration (26 tools)
│   ├── locations.json              # Named locations registry
│   ├── ros/
│   │   ├── __init__.py
│   │   └── ros_client.py          # WebSocket client (auto-reconnect)
│   ├── tools/
│   │   ├── navigation.py          # navigate_to_pose
│   │   ├── waypoints.py           # navigate_waypoints
│   │   ├── monitoring.py          # position, fleet, battery
│   │   ├── control.py             # stop, emergency_stop
│   │   ├── obstacles.py           # check_obstacles (laser scan)
│   │   ├── map_viz.py             # get_map_with_robots
│   │   ├── coordination.py        # assign/dispatch/plan/replan
│   │   ├── natural_language.py    # Named locations + go_to
│   │   └── advanced.py            # Collision, queue, dashboard, optimal
│   └── coordination/
│       ├── fleet_state.py          # FleetStateManager (singleton)
│       ├── task_planner.py         # Greedy linear allocation
│       ├── collision_predictor.py  # Path collision prediction
│       ├── task_queue.py           # Auto-dispatch priority queue
│       ├── hungarian.py            # Optimal assignment (scipy)
│       ├── dashboard_server.py     # WebSocket server for browser
│       └── chat_agent.py           # LLM chat (Bedrock/Anthropic)
│
├── dashboard/
│   └── live_dashboard.html         # Browser UI (map + chat)
│
├── sim/                            # Simulation + Tests
│   ├── test_integration.py         # 12 local tests
│   ├── test_remote.py              # 10 real hardware tests
│   ├── test_coordination.py        # 8 coordination tests
│   ├── test_full_scenario.py       # 5 scenario tests
│   ├── test_new_features_remote.py # Advanced feature tests
│   ├── scale_test.py              # Benchmark tool
│   ├── diagnose_remote.py         # rosbridge diagnostic
│   └── robot_simulator.py         # Standalone simulator
│
└── tests/                          # Unit Tests (pytest)
    ├── conftest.py
    ├── __init__.py
    ├── test_ros_client.py          # 12 tests
    ├── test_new_tools.py           # 12 tests
    └── test_next_steps.py          # 25 tests
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `Address already in use (9090)` | `lsof -ti :9090 \| xargs kill -9` |
| `Address already in use (8080)` | `lsof -ti :8080 \| xargs kill -9` |
| `Connection timed out` to remote | Check if rosbridge is running on remote machine |
| `No pose received (timeout)` | Robots may not be publishing - check `python sim/diagnose_remote.py` |
| `Nav2 status=6 (ABORTED)` | Goal is outside map bounds - keep within -1.3 to 1.3 |
| `ModuleNotFoundError: boto3` | `pip install boto3` |
| `ModuleNotFoundError: anthropic` | `pip install anthropic` |
| `scipy not found` | `pip install scipy` (optional - greedy fallback works) |
| `pip install websockets` | Required for `run.py` and dashboard |
| Dashboard shows "Disconnected" | Start dashboard server first, then open HTML |

---

## Network Configuration

| Service | Default Port | Configurable |
|---------|-------------|--------------|
| rosbridge (ROS2 bridge) | 9090 | `--port` |
| Dashboard WebSocket | 8080 | `--dashboard-port` |
| MCP Server | stdio | N/A |

For remote rosbridge, ensure port 9090 is accessible from your machine:
```bash
# Test connectivity
nc -zv 192.168.0.8 9090
```

---

*Generated 2026-07-03*
