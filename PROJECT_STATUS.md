# Robo_Fleet - Project Status

> MCP Server for Multi-Robot Fleet Control with AI-Powered Coordination

## Architecture

```
┌──────────────────┐    chat     ┌───────────────────┐   WebSocket    ┌────────────┐
│  LLM (Bedrock/   │◄──────────►│   MCP Server      │◄──────────────►│  rosbridge │
│  Claude)         │  tool calls │   (26 tools)      │   ws://9090    │   (ROS2)   │
└──────────────────┘             └───────────────────┘                └────────────┘
        │                               │                                   │
        ▼                               ▼                            ┌──────┼──────┐
┌──────────────────┐    ┌───────────────────────┐                    ▼      ▼      ▼
│  Web Dashboard   │    │  FleetStateManager    │                 [tb1]  [tb2]  [tb3]
│  (live map +     │    │  (persistent conn,    │
│   chat panel)    │    │   in-memory cache)    │
└──────────────────┘    └───────────────────────┘
```

---

## What's Working Now

### Core Infrastructure (5 features)
- [x] FastMCP server with stdio transport (26 registered tools)
- [x] ROS WebSocket client with auto-reconnect (exponential backoff, health monitor)
- [x] Mock rosbridge + fleet simulator (`python run.py` - one command)
- [x] Remote rosbridge support (`--host 192.168.0.8`)
- [x] Docker setup (docker-compose.yml for full ROS2 stack)

### Navigation (2 tools)
- [x] `navigate_to_pose` - send any robot to (x, y, theta)
- [x] `navigate_waypoints` - sequential multi-point navigation

### Monitoring (5 tools)
- [x] `get_robot_position` - current pose from amcl_pose
- [x] `get_fleet_status` - all robots positions + battery + status
- [x] `get_battery_level` - battery %, voltage, health status
- [x] `check_obstacles` - 360-degree laser scan, closest obstacle + direction
- [x] `get_map_with_robots` - fleet positions on coordinate grid

### Control (2 tools)
- [x] `stop_robot` - zero velocity + cancel navigation
- [x] `emergency_stop` - halt ALL robots immediately

### Multi-Robot Coordination (6 tools)
- [x] `FleetStateManager` - singleton persistent WebSocket, O(1) state queries
- [x] `TaskPlanner` - greedy linear allocation, battery-aware cost function
- [x] Collision avoidance - zone buffer (O(N^2) pairwise), priority-based resolution
- [x] `assign_tasks` - optimal allocation of N tasks to M robots
- [x] `dispatch_tasks` - allocate AND navigate (parallel)
- [x] `get_plan` / `replan` - view/force re-optimization
- [x] `set_robot_priority` / `configure_fleet` - priorities and groups/zones

### Advanced Features (implemented in latest build)
- [x] **Path Collision Prediction** - linear trajectory extrapolation, head-on detection
- [x] **Task Queue with Auto-Dispatch** - priority queue, background thread dispatches to idle robots
- [x] **Natural Language Goals** - named location registry (8 defaults), `go_to_location`, `send_nearest_to`
- [x] **Hungarian Optimal Allocation** - scipy.optimize.linear_sum_assignment (greedy fallback)
- [x] **Scale Benchmark** - validated 10 robots x 10 tasks < 1ms allocation

### Live Web Dashboard
- [x] WebSocket server streams fleet state at 5Hz
- [x] 2D canvas with robot positions, direction arrows, trails, battery bars
- [x] Click-to-navigate (click map to send nearest robot)
- [x] **Chat panel with LLM** (Claude via Bedrock or Anthropic API)
- [x] Non-blocking navigation (all robots move simultaneously)
- [x] Tool calling - LLM invokes fleet tools via natural language

### Testing (validated)
- [x] 25 unit tests (mocked WebSocket) - all passing
- [x] 12 integration tests (local simulator) - all passing
- [x] 10 remote tests (real rosbridge at 192.168.0.8) - all passing
- [x] 5 full scenarios (real hardware: long nav, waypoints, simultaneous, dispatch, return home)
- [x] 8 coordination tests (fleet state, allocation, collision, dispatch, groups)
- [x] Scale benchmark (10 robots, < 100ms)

### Real Hardware Tested
- [x] Connected to rosbridge at `ws://192.168.0.8:9090`
- [x] 2 TurtleBot3 robots: tb1, tb3
- [x] Topics confirmed: amcl_pose, odom, scan, navigate_to_pose action
- [x] Navigation goals succeed (Nav2 status=4)
- [x] Laser scan: 360 rays, obstacle detection working
- [x] Multi-robot simultaneous navigation verified

---

## Project Files

```
Robo_Fleet/
├── run.py                      # One-command launcher (rosbridge + sim)
├── start_dashboard.py          # Dashboard + chat launcher
├── setup.sh                    # Install script
├── requirements.txt
├── docker-compose.yml
├── Dockerfile.mcp
├── README.md
├── PROJECT_STATUS.md           # This file
├── features.html               # Visual feature tracker
│
├── mcp_server/
│   ├── server.py               # FastMCP init
│   ├── index.py                # Registers all 26 tools
│   ├── ros/
│   │   └── ros_client.py       # WebSocket client (auto-reconnect)
│   ├── tools/
│   │   ├── navigation.py       # navigate_to_pose
│   │   ├── waypoints.py        # navigate_waypoints
│   │   ├── monitoring.py       # position, fleet status, battery
│   │   ├── control.py          # stop, emergency_stop
│   │   ├── obstacles.py        # check_obstacles
│   │   ├── map_viz.py          # get_map_with_robots
│   │   ├── coordination.py     # assign/dispatch/plan/replan/priority/configure
│   │   ├── natural_language.py # go_to_location, send_nearest_to, add/remove/list
│   │   └── advanced.py         # predict_collisions, start/stop dashboard, queue tools, optimal
│   └── coordination/
│       ├── fleet_state.py      # FleetStateManager (singleton, persistent conn)
│       ├── task_planner.py     # TaskPlanner (greedy + collision)
│       ├── collision_predictor.py  # Path collision prediction
│       ├── task_queue.py       # Auto-dispatch priority queue
│       ├── hungarian.py        # Optimal assignment (scipy)
│       ├── dashboard_server.py # WebSocket server for browser
│       └── chat_agent.py       # LLM chat (Bedrock/Anthropic + tool calling)
│
├── dashboard/
│   └── live_dashboard.html     # Browser UI (map + chat)
│
├── sim/
│   ├── test_integration.py     # 12 tests (local)
│   ├── test_remote.py          # 10 tests (real hardware)
│   ├── test_coordination.py    # 8 tests (fleet + planner)
│   ├── test_full_scenario.py   # 5 scenarios (real robots)
│   ├── test_new_features_remote.py  # Advanced features (remote)
│   ├── scale_test.py           # Benchmark tool
│   └── diagnose_remote.py      # rosbridge diagnostic
│
└── tests/
    ├── test_ros_client.py      # 12 unit tests
    ├── test_new_tools.py       # 12 unit tests
    └── test_next_steps.py      # 25 unit tests
```

---

## How to Run

```bash
# Setup (one time)
cd ~/Downloads/Robo_Fleet
chmod +x setup.sh && ./setup.sh
source .venv/bin/activate
pip install boto3 anthropic  # For chat agent

# Local simulator
python run.py

# Dashboard with AI chat (against real robots)
AWS_PROFILE=sandbox AWS_REGION=us-east-1 python start_dashboard.py \
  --rosbridge 192.168.0.8 --robots tb1 tb3 --provider bedrock \
  --model anthropic.claude-3-haiku-20240307-v1:0 --open

# Run tests
pytest tests/ -v                                          # Unit tests
python sim/test_remote.py --host 192.168.0.8              # Real hardware
python sim/test_full_scenario.py --host 192.168.0.8       # Full scenarios
```

---

## What's Next

### Short Term (next session)
1. **Fix Bedrock tool calling format** - resolve the ValidationException with tool definitions
2. **Dashboard click-to-navigate** - verify it works end-to-end with real robots
3. **Chat agent testing** - validate full conversation loop (user -> LLM -> tool -> robot -> result)

### Medium Term
4. **Path collision prediction in real-time** - integrate into navigation (auto-pause before collision)
5. **Persistent task queue** - save queue to disk, survive restarts
6. **Multi-robot patrol missions** - define routes, loop indefinitely, report anomalies
7. **Fleet analytics** - track distance, goals completed, errors per robot over time

### Long Term
8. **LangGraph multi-agent orchestrator** - one agent per task with claim/release/retry lifecycle
9. **Camera feed + vision** - "What does tb1 see?" with multimodal LLM
10. **SLAM integration** - explore unknown environments, build maps
11. **Multi-floor / outdoor** - elevator coordination, GPS
12. **Production hardening** - auth, rate limiting, audit logging, OTA updates

---

## Tech Stack

| Component | Technology |
|-----------|-----------|
| MCP Server | FastMCP (Python) |
| Transport | stdio (LLM), WebSocket (rosbridge) |
| Robots | TurtleBot3 Burger (Nav2, ROS2 Humble) |
| Simulation | Pure Python (no Gazebo needed) |
| LLM | Claude via AWS Bedrock or Anthropic API |
| Dashboard | HTML5 Canvas + WebSocket |
| Allocation | Greedy linear + Hungarian (scipy) |
| Collision | Zone buffer + trajectory prediction |
| Testing | pytest + real hardware integration |
| Deployment | pip install (cross-platform) or Docker |

---

*Last updated: 2026-07-03 03:11 GMT*
