# robo_fleet — AI-driven ROS 2 multi-robot fleet (Enova site)

**robo_fleet** is a full multi-robot security-fleet stack: a realistic **ROS 2 / Gazebo simulation of the Enova site** built from **real OpenStreetMap data**, where **PearlGuard** robots navigate autonomously with **Nav2**, and everything is driven remotely through an **AI multi-agent layer** exposed over the **Model Context Protocol (MCP)**.

Two layers, one repo:

1. **ROS 2 simulation & the robot platform** — the world, the robot, the sensors, and multi-robot Nav2 navigation.
2. **Multi-agent layer & MCP** — a supervisor orchestrating specialist AI agents that command the fleet through tool calls.

---

## Section 1 — ROS 2 simulation & the robot platform

### 1.1 Simulating the Enova site from OpenStreetMap

Instead of hand-drawing a mock world, the site is reconstructed from **real geospatial data**: the **Technopole de Sousse / Novation City** campus (origin datum lat `35.8173`, lon `10.5912`), pulled straight from **OpenStreetMap** via the Overpass API.

The pipeline is three scripts:

| File | What it does |
|---|---|
| `src/my_pguard_bot/scripts/fetch_osm.py` | Queries Overpass API (~600 m radius) for building footprints, roads, and named offices/amenities; projects them to a local ENU frame; emits `worlds/sousse_buildings.sdf` + `worlds/sousse_pois.json`. Named POIs (Enova, Proxym, VEO…) are matched to their building polygons. |
| `src/my_pguard_bot/scripts/build_world.py` | Inlines the OSM buildings into `worlds/novation_city.template.sdf` → final `worlds/novation_city.sdf`. |
| `src/my_pguard_bot/scripts/build_map.py` | Rasterizes the SDF into a **3-tier Nav2 occupancy grid** (1200×1200 m, 1 m/cell): buildings **LETHAL**, off-road/grass **NO_INFORMATION** (crossable but costly), roads **FREE**. Outputs `maps/novation_city.pgm` + `maps/novation_city.yaml`. |

Deep dive: [`docs/MAP_GENERATION.md`](docs/MAP_GENERATION.md) and [`docs/COSTMAP.md`](docs/COSTMAP.md).

### 1.2 The robot — two models

The robot is modeled after the **Enova Robotics PGuard**, an outdoor security/patrol robot (1.50 m × 1.10 m × 1.60 m, ~250 kg, 4 driven wheels, skid-steer, up to 3.3 m/s).

Two successive models:

1. **Custom 3D PGuard-like model** — `src/my_pguard_bot/description/pguard.urdf.xacro` + `pguard.gazebo.xacro`. A simplified box chassis with turret, beacon, and 4 wheels. Sensors: **RTK GNSS** (~2 cm), **IMU** (100 Hz), **4× ultrasonic rangefinders** (front/rear/left/right), and a **forward camera**.
2. **Real PearlGuard model** — package `src/pearlguard_description` (`pguard.xacro` + `urdf/*.xacro`). Uses the actual **PGuard CAD meshes** (`meshes/pguard_x/chassis.dae`, `dome.dae`, wheels) and a **VLP-16 3D LiDAR** (`meshes/VLP-16/*`). Sensors: **VLP-16 3D LiDAR**, **RTK GPS (NavSat)**, **IMU**, and diff-drive odometry. This is the model we call **pearlguard**.

### 1.3 Multi-robot: two robots, easily extensible

The launch system runs **two namespaced robots** — `pearlguard1` and `pearlguard2` — defined in a `ROBOTS` list at the top of the launch files. Adding more robots is just a matter of extending that list.

The PearlGuard xacro accepts a `namespace` argument so every link, plugin, and topic gets prefixed (`/pearlguardN/...`), which prevents topic collisions between robots.

| Launch file | What it starts |
|---|---|
| `launch/full_stack.launch.py` | **Main entry**: sim + dual-EKF localization + Nav2 for both robots. |
| `launch/sim.launch.py` | Gazebo (headless by default, `use_gui:=true` for the GUI), robot_state_publisher, spawn, `ros_gz_bridge`. |
| `launch/localization.launch.py` | Dual-EKF outdoor localizer (odom + IMU + RTK GPS) per robot. |
| `launch/robofleet.launch.py` | Static `map → odom` transforms, the **robo_fleet topic adapter**, and **rosbridge WebSocket on port 9090**. |
| `launch/patrol.launch.py` | GPS perimeter patrol client. |
| `launch/viz.launch.py` | Foxglove bridge for browser-based visualization (port 8765). |

### 1.4 Navigation & sensing per robot

Every robot runs a **full Nav2 stack** — planner, controller, and costmaps — with per-robot namespaced configs:

- `config/nav2_params_pearlguard1.yaml`, `config/nav2_params_pearlguard2.yaml`
- `config/ekf_pearlguard1.yaml`, `config/ekf_pearlguard2.yaml`
- A Nav2 map server publishing `maps/novation_city` + a static `map → odom` transform per robot.

**Sensors on each PearlGuard:**

- **VLP-16 3D LiDAR** — obstacle detection and costmap input
- **RTK GPS (NavSat)** — absolute positioning (~2 cm horizontal)
- **IMU** — orientation / odometry fusion
- **Diff-drive odometry** — wheel encoders

**Helper scripts:**

- `scripts/robo_fleet_adapter.py` — bridges the namespaced sim topics to the fleet layer's expected layout (e.g. `/{robot}/cmd_vel`, `/{robot}/navigate_to_pose`, `/{robot}/scan`) so the MCP tools work unchanged.
- `scripts/patrol_client.py` + `scripts/generate_waypoints.py` — build a GPS waypoint loop from the OSM buildings and drive an indefinite perimeter patrol.

Sim environment notes (headless container, Gazebo GUI/RViz caveats): [`README_outdoor_sim.md`](README_outdoor_sim.md).

---

## Section 2 — Multi-agent layer & MCP

### 2.1 What it is

`mcp_server/` is a **Model Context Protocol** server that turns ROS 2 fleet operations into **AI-callable tools**. Any MCP client can drive the fleet through natural language.

- `server.py` — FastMCP instance
- `index.py` — entry point (transports: **stdio** and **streamable-http**)
- A **supervisor → specialist agents** graph (`graph/graph.py`, `graph/state.py`, `agents/react.py`): each request is routed to one or more ReAct agents (LLM + tools), which execute the actual fleet operations.

### 2.2 Architecture

- **`coordination/fleet_state.py`** — `FleetStateManager`: keeps a single persistent rosbridge WebSocket, subscribes once to every robot's topics, and caches live robot state (position, battery, status) for O(1) tool queries.
- **`ros/ros_client.py`** — low-level rosbridge WebSocket client (publish, subscribe, action goals).
- **`graph/graph.py` + `agents/react.py`** — a supervisor LLM plans a sequence of agents; each specialist agent is a ReAct loop with a curated tool set and its own system prompt.

### 2.3 The agents

| Agent | Role | Tools |
|---|---|---|
| **navigation_agent** | Moves robots to coordinates / waypoint sequences | `navigate_to_pose`, `navigate_waypoints` |
| **monitoring_agent** | Reports positions, battery, fleet status | `get_robot_position`, `get_fleet_status`, `get_battery_level` |
| **control_agent** | Stops robots immediately | `stop_robot`, `emergency_stop` |
| **collision_agent** | Detects obstacles and predicts robot collisions | `check_obstacles`, `predict_collisions` |
| **planning_agent** | Allocates and dispatches tasks optimally | `assign_tasks`, `dispatch_tasks`, `get_plan`, `replan`, `set_robot_priority`, `configure_fleet`, `assign_tasks_optimal` |
| **queue_agent** | Manages the task dispatch queue | `add_task_to_queue`, `get_queue`, `clear_queue`, `start_auto_dispatch`, `stop_auto_dispatch` |
| **dashboard_agent** | Starts/stops the live dashboard | `start_dashboard`, `stop_dashboard` |
| **natural_lang_agent** | Manages named locations and sends the nearest robot | `list_locations`, `add_location`, `remove_location`, `go_to_location`, `send_nearest_to` |
| **map_viz_agent** | Draws an ASCII map of robot positions | `get_map_with_robots` |

### 2.4 Tools & coordination

Tools are organized by concern in `mcp_server/tools/` (`navigation.py`, `monitoring.py`, `control.py`, `obstacles.py`, `coordination.py`, `advanced.py`, `natural_language.py`, `map_viz.py`, `chat.py`).

Underneath them, `coordination/` holds the fleet brain:

- `task_planner.py` — greedy, battery-aware task assignment
- `task_queue.py` — auto-dispatch queue
- `hungarian.py` — globally optimal assignment
- `collision_predictor.py` — pairwise trajectory-collision prediction
- `dashboard_server.py` — WebSocket server streaming fleet state to the browser
- `chat_agent.py` — the dashboard's chat backend (itself an MCP client)

---

## Section 3 — Quick Start

### 1. Build / enter the workspace

The ROS 2 stack runs inside a Docker container (ROS 2 Jazzy + Gazebo Harmonic + Nav2). See [`README_outdoor_sim.md`](README_outdoor_sim.md) and [`SETUP.md`](SETUP.md) for the container/setup details.

### 2. Launch the simulation + navigation

```bash
ros2 launch my_pguard_bot full_stack.launch.py
```

This starts Gazebo with the Novation City world, spawns **both PearlGuard robots**, runs the dual-EKF localizers, and brings up **Nav2 for each robot**.

### 3. Launch rosbridge + the topic adapter

```bash
ros2 launch my_pguard_bot robofleet.launch.py
```

This starts the **rosbridge WebSocket server on port 9090** plus the `robo_fleet_adapter` that bridges the robot topics to the fleet layer.

### 4. Launch the dashboard

```bash
python3 start_dashboard.py --rosbridge localhost --robots pearlguard1 pearlguard2 --open
```

You should now **see both robots** moving on the live dashboard. You can click the map to send robots to a position.

### 5. (Optional) Drive the fleet with the AI agents

Use the **chat panel in the dashboard** — the multi-agent layer (Section 2) runs behind it, so you can ask for things like:

```
Where is pearlguard1?
Send pearlguard2 to coordinates 25, -112
Fleet status
Check obstacles near pearlguard1
```

Or connect any MCP client to the server (`mcp_server/index.py` — stdio by default, `--transport http` for streamable-http).

---

## Section 4 — Layout & references

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
├── start_dashboard.py          # dashboard + chat agent launcher
├── run.py / run_tests.py       # standalone sim / test harness
└── README.md
```

**Deep dives:** [`docs/MAP_GENERATION.md`](docs/MAP_GENERATION.md) · [`docs/COSTMAP.md`](docs/COSTMAP.md) · [`docs/PROJECT.md`](docs/PROJECT.md) · [`docs/outdoor-sim-guide.md`](docs/outdoor-sim-guide.md) · [`README_outdoor_sim.md`](README_outdoor_sim.md) · [`SETUP.md`](SETUP.md)

## License

MIT
