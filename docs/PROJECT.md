# PGuard Outdoor Robot Simulation — Project Documentation

**Enova PGuard security robot patrolling Novation City / Technopôle de Sousse, Tunisia**
**Stack:** ROS 2 Jazzy · Gazebo Harmonic · Nav2 · robot_localization · rosbridge · MCP (Model Context Protocol) · AWS Bedrock / Anthropic

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [System Architecture](#2-system-architecture)
3. [Repository Layout](#3-repository-layout)
4. [Component Deep-Dive](#4-component-deep-dive)
5. [Setup — Amazon Linux 2 / Any Linux Host (Docker)](#5-setup--amazon-linux-2--any-linux-host-docker)
6. [Setup — Ubuntu 24.04 Native (no Docker)](#6-setup--ubuntu-2404-native-no-docker)
7. [Running the Full Stack](#7-running-the-full-stack)
8. [Web Dashboard](#8-web-dashboard)
9. [MCP Server Setup](#9-mcp-server-setup)
10. [LLM (Bedrock / Anthropic) Configuration](#10-llm-bedrock--anthropic-configuration)
11. [Remote Access via SSH Tunnel](#11-remote-access-via-ssh-tunnel)
12. [Session Change-Log — What Was Fixed](#12-session-change-log--what-was-fixed)
13. [Troubleshooting](#13-troubleshooting)
14. [Reference — Ports, Topics, Actions](#14-reference--ports-topics-actions)
15. [File-by-File Index](#15-file-by-file-index)

---

## 1. Executive Summary

This workspace simulates a **250 kg Ackermann-drive security robot** (PGuard) patrolling a **1200 m × 1200 m outdoor campus** modelled from real OpenStreetMap data of **Novation City / Technopôle de Sousse, Tunisia** (26 buildings, ~1000 road segments, real GPS/UTM coords).

The simulator is fully **headless** — Gazebo Harmonic runs as `gz sim -s` inside a Docker container on Amazon Linux 2 (no GPU, no display). All visualization goes through:

- A **live web dashboard** (custom canvas + WebSocket) that renders the OSM occupancy grid, robot pose, planned path, and a **real 3D chase-cam preview** streamed from Gazebo.
- **rosbridge_suite** (WebSocket → ROS 2 bridge) on port 9090 for any browser/Python peer.
- **foxglove_bridge** as an alternative (port 8765).

The fleet is controllable **from any AI agent** through a **Model Context Protocol (MCP)** server exposing 29 tools (navigation, monitoring, task planning, waypoint following, natural-language locations, dashboard control). The dashboard's chat panel is itself an MCP client backed by Claude Sonnet 4 on AWS Bedrock (or Anthropic API).

### What runs where

| Component | Where | Why |
|---|---|---|
| Gazebo Harmonic | Docker container (`pguard_sim`) | Headless physics — no host GPU |
| ROS 2 Jazzy stack (Nav2, EKF, adapter) | Same container | ROS 2 Jazzy not packaged for AL2 |
| rosbridge (:9090) | Container | Any browser/Python peer |
| MCP HTTP (:8766) | Container | Any HTTP MCP client (Continue, LangChain, curl) |
| MCP stdio | Container via `docker exec -i` | Cursor / Claude Desktop / local subprocesses |
| Dashboard WS (:8090) | Container | Streams fleet state + planned path to browser |
| Dashboard HTTP (:8091) | **Host** | Serves the HTML/JS/PNGs statically |
| Cam-refresh loop | Host | Periodically grabs Gazebo frames via `docker exec` |

### End-user surfaces

1. **Browser dashboard** — `http://<host>:8091/live_dashboard.html?ws=ws://<host>:8090`
2. **AI chat** in the dashboard sidebar (LLM controls the fleet through MCP)
3. **Cursor / Claude Desktop** MCP client (`pguard-fleet` server in `~/.cursor/mcp.json`)
4. **Any HTTP MCP client** at `http://<host>:8766/mcp`
5. **Direct ROS 2 CLI** inside the container


---

## 2. System Architecture

```
┌───────────────────────────────────────────────────────────────────────┐
│                         Docker container: pguard_sim                  │
│                                                                       │
│   ┌───────────────┐   /model/pguard/*   ┌───────────────────────┐    │
│   │   Gazebo      │◄──────────────────►│  ros_gz_bridge        │    │
│   │   Harmonic    │   (sensor + cmd)    │  bridge.yaml          │    │
│   │   (gz sim -s) │                     └──────────┬────────────┘    │
│   │ novation_city │                                │                 │
│   └───────┬───────┘                                ▼                 │
│           │ world_cam images                ┌───────────┐            │
│           ▼                                  │  /cmd_vel │            │
│    /world_cam/chase                          │  /odom    │            │
│    /world_cam/top                            │  /sonar/* │            │
│                                              └─────┬─────┘            │
│                                                    │                  │
│   ┌────────────────────────────────────────────────▼──────┐          │
│   │             robo_fleet_adapter (ROS 2 node)           │          │
│   │  /odometry/filtered → /pguard/amcl_pose               │          │
│   │  /pguard/cmd_vel(Stamped) → /cmd_vel(Twist)           │          │
│   │  /sonar/{front,rear,left,right} → /pguard/scan        │          │
│   │  /pguard/navigate_to_pose ─ relay → /navigate_to_pose │          │
│   │  Synthetic /pguard/battery_state @ 1Hz                │          │
│   └────────────────────────┬──────────────────────────────┘          │
│                            │                                          │
│   ┌───────────┐   ┌────────▼──────────┐   ┌──────────────────┐       │
│   │ EKF       │   │  Nav2 (full stack) │   │ map_server        │       │
│   │ (odom→map)│──►│  planner_server    │◄──│ novation_city.pgm │       │
│   │ ekf.yaml  │   │  controller_server │   │ 1200×1200 @ 1m   │       │
│   └───────────┘   │  bt_navigator      │   └──────────────────┘       │
│                   │  behavior_server   │                              │
│                   └────────┬───────────┘                              │
│                            │                                          │
│   ┌────────────────────────▼──────────────────────────────┐          │
│   │              rosbridge_server (:9090)                  │          │
│   │      any WS peer can pub/sub / call actions            │          │
│   └────────────┬──────────────┬────────────┬───────────────┘          │
│                │              │            │                          │
│   ┌────────────▼─────┐ ┌──────▼────────┐ ┌─▼───────────────┐         │
│   │ robo_fleet MCP    │ │ Dashboard WS  │ │ Fleet state     │         │
│   │ server (:8766)    │ │ server(:8090) │ │ manager         │         │
│   │ 29 tools, stdio+  │ │ /plan + status│ │ (in-memory      │         │
│   │ streamable-http   │ │ + fleet_state │ │  cache)         │         │
│   └────────┬──────────┘ └───────┬───────┘ └─────────────────┘         │
│            │                    │                                     │
└────────────┼────────────────────┼─────────────────────────────────────┘
             │                    │
     ┌───────┴────────┐   ┌───────┴───────────────────────┐
     │  Cursor        │   │  Browser dashboard            │
     │  Claude Desktop│   │  live_dashboard.html          │
     │  (stdio via    │   │  + Bedrock chat via WS        │
     │  mcp_pguard.sh)│   │  + PGuard 3D preview PNG      │
     └────────────────┘   └───────────────────────────────┘
```

### Data flow (a click on the map)

1. User clicks map canvas → JS converts pixel to world coord `(wx, wy)`.
2. Dashboard sends `{command:'navigate', robot_id:'pguard', x:wx, y:wy}` over WS to `dashboard_server.py`.
3. `dashboard_server._send_nav_goal` opens a rosbridge connection and calls `send_goal` on `/pguard/navigate_to_pose`.
4. `robo_fleet_adapter` relays that goal to Nav2's `/navigate_to_pose`.
5. Nav2 planner_server produces `/plan`; controller_server publishes `/cmd_vel`.
6. Adapter converts `/pguard/cmd_vel(Stamped)` → `/cmd_vel(Twist)` for the ros_gz_bridge.
7. Gazebo applies velocity; odometry publishes back; EKF fuses to `/odometry/filtered`.
8. The subscriber thread in `dashboard_server` forwards `/plan` and `/navigate_to_pose/_action/status` to all browser clients as `{type:'plan'}` and updates `_goal_status[robot_id]`.
9. Browser draws the green path + updates the sidebar's status pill.


---

## 3. Repository Layout

```
ros2_outdoor_sim/
├── Dockerfile                        # osrf/ros:jazzy-desktop-full + gz-harmonic + nav2 + deps
├── README.md                         # host quickstart
├── mcp_pguard.sh                     # stdio MCP wrapper (docker exec -i pguard_sim)
├── run_dashboard.py                  # small launcher (unused by current flow)
├── scripts/
│   └── setup_ubuntu_24_04.sh         # automated installer for native Ubuntu 24.04 (§6.0)
├── run_tests.py                      # in-tree test harness for MCP tools + Nav2
├── pguard_dashboard.html             # legacy single-file dashboard (superseded)
│
├── src/my_pguard_bot/                # ROS 2 package (colcon)
│   ├── CMakeLists.txt                # installs launch/config/description/maps/... to share/
│   ├── package.xml
│   ├── launch/
│   │   ├── sim.launch.py             # gz sim + ros_gz_bridge + world_cams
│   │   ├── localization.launch.py    # dual EKF (odom + map) + navsat_transform
│   │   ├── viz.launch.py             # rosbridge + foxglove bridge + map_server + amcl
│   │   ├── full_stack.launch.py      # sim + localization + Nav2 + adapter
│   │   ├── robofleet.launch.py       # full_stack + robo_fleet_adapter
│   │   └── patrol.launch.py          # waypoint patrol demo
│   ├── config/
│   │   ├── nav2_params.yaml          # 3-layer global costmap (static + obstacle + inflation)
│   │   ├── ekf.yaml                  # dual EKF: local (odom) + global (map with GPS)
│   │   ├── bridge.yaml               # ros_gz_bridge topic map
│   │   └── patrol_waypoints.yaml     # perimeter patrol coordinates
│   ├── description/                  # xacro + meshes (chassis, wheels, sensors)
│   ├── urdf/                         # generated URDFs
│   ├── worlds/
│   │   ├── novation_city.template.sdf
│   │   ├── novation_city.sdf         # generated: template + buildings + roads
│   │   ├── sousse_buildings.sdf      # 26 OSM building models + ~1000 road segments
│   │   └── sousse_pois.json          # OSM POI cache
│   ├── maps/
│   │   ├── novation_city.pgm         # 1200×1200 3-tier occupancy grid
│   │   └── novation_city.yaml        # origin=[-600,-600,0], res=1.0 m/cell
│   ├── scripts/
│   │   ├── fetch_osm.py              # Overpass API → buildings + roads + POIs
│   │   ├── build_world.py            # OSM → SDF models
│   │   ├── build_map.py              # OSM → PGM (3-tier: building/road/off-road)
│   │   ├── generate_waypoints.py     # perimeter waypoints from map
│   │   ├── robo_fleet_adapter.py     # ROS 2 node bridging PGuard ↔ robo_fleet
│   │   ├── patrol_client.py          # NavigateThroughPoses action client
│   │   ├── save_camera_frame.py      # capture gz.msgs.Image → PNG
│   │   ├── annotate_top_view.py      # overlay robot pose on top-cam frame
│   │   └── render_pguard.py          # synthetic 2D chassis render (legacy)
│   ├── rviz/                         # RViz configs
│   └── foxglove/                     # Foxglove Studio layouts
│
├── robo_fleet/                       # MCP server + dashboard (Python package)
│   ├── requirements.txt              # mcp[cli], websockets, uvicorn, pydantic, ...
│   ├── start_dashboard.py            # launches dashboard WS + LLM chat agent
│   ├── serve_http.sh                 # runs MCP over streamable-http inside container
│   ├── README.md
│   ├── dashboard/                    # BROWSER ASSETS (served by python -m http.server 8091)
│   │   ├── live_dashboard.html       # main UI shell
│   │   ├── live_dashboard.js         # canvas render + WS client + zoom/pan/chat
│   │   ├── novation_city.png         # grayscale 1200×1200 map (from PGM)
│   │   ├── novation_city_color.png   # colored RGBA map (buildings/roads/off-road)
│   │   ├── pguard_chase.png          # LIVE Gazebo chase-cam frame (auto-refreshed 3s)
│   │   ├── pguard_top.png            # LIVE Gazebo top-cam frame
│   │   └── refresh_cam.sh            # background loop grabbing gz frames via docker exec
│   └── mcp_server/
│       ├── index.py                  # entrypoint - picks --transport stdio|http
│       ├── server.py                 # FastMCP instance
│       ├── locations.json            # named location registry
│       ├── ros/ros_client.py         # rosbridge WebSocket client (pub/sub/actions)
│       ├── tools/                    # @mcp.tool() implementations (see §4.5)
│       │   ├── navigation.py         # navigate_to_pose
│       │   ├── waypoints.py          # navigate_waypoints
│       │   ├── monitoring.py         # get_robot_position, get_fleet_status, get_battery_level
│       │   ├── control.py            # stop_robot, emergency_stop
│       │   ├── obstacles.py          # check_obstacles
│       │   ├── map_viz.py            # get_map_with_robots
│       │   ├── coordination.py       # assign_tasks, dispatch_tasks, get_plan, replan, ...
│       │   ├── advanced.py           # predict_collisions, task queue, dashboard control, hungarian
│       │   ├── natural_language.py   # list/add/remove_location, go_to_location, send_nearest_to
│       │   └── exemples.py           # test helpers
│       └── coordination/
│           ├── fleet_state.py        # rosbridge subscriber → in-memory fleet cache
│           ├── dashboard_server.py   # WS server + /plan+status subscriber + Bedrock chat proxy
│           ├── chat_agent.py         # LLM MCP client (Anthropic / AWS Bedrock)
│           ├── task_planner.py       # nearest-neighbor task assignment
│           ├── task_queue.py         # priority queue + auto-dispatch
│           ├── hungarian.py          # optimal assignment (Kuhn–Munkres)
│           └── collision_predictor.py# forward-simulation collision check
│
└── docs/
    ├── PROJECT.md                    # THIS FILE
    ├── outdoor-sim-guide.md
    ├── 3d_*.png, robot_*.png         # gallery renders
    └── novation_map_before_after.png # proof of the map-fix change
```


---

## 4. Component Deep-Dive

### 4.1 The world — `novation_city.sdf`

Modelled from real OpenStreetMap data of Technopôle de Sousse (approx 35.860 N, 10.598 E).

- Ground plane 1200 × 1200 m, tan/sand texture
- **26 buildings** (`bld_*` models) — extruded oriented bounding boxes from OSM `building=*` polygons
- **~1000 road segments** (`road_*` models) — extruded from OSM `highway=*` linestrings with proper width per class
- **Two chase cameras** in the world (moved at runtime via `gz service /world/novation_city/set_pose`):
  - `chase_cam` publishing `/world_cam/chase` (960×540) — repositioned to follow PGuard
  - `top_cam` publishing `/world_cam/top` (1024×1024) — high-altitude top-down
- Sun + skybox for lighting

Generated by:
```bash
python3 src/my_pguard_bot/scripts/fetch_osm.py   # writes sousse_buildings.sdf + sousse_pois.json
python3 src/my_pguard_bot/scripts/build_world.py # merges into novation_city.sdf
python3 src/my_pguard_bot/scripts/build_map.py   # writes maps/novation_city.pgm+.yaml
```

### 4.2 The robot — PGuard

Enova PGuard specs modelled here:

| Property | Value |
|---|---|
| Chassis | 1.5 m × 1.1 m × 0.6 m, 250 kg |
| Drive | Ackermann-like (differential in sim) |
| Cruise speed | 1.5 m/s (5.4 km/h) |
| Sensors | 4 sonars (F/R/L/R, 5 m range) + IMU + RTK GPS |
| Localization | dual EKF (odom-frame + map-frame) + navsat_transform |
| Radius (circumscribing) | 0.95 m — used by Nav2 as `robot_radius` |

Sonars are synthesized into a 4-ray LaserScan (`/pguard/scan`) by `robo_fleet_adapter.py` so Nav2's obstacle_layer can consume them.

### 4.3 Nav2 configuration — `config/nav2_params.yaml`

**Global costmap (1400×1400 m, 1 m/cell, static)** with three layers:
- `static_layer` — subscribes to `/map`; `trinary_costmap: false` preserves the intermediate 100-value off-road cells as a cost gradient
- `obstacle_layer` — 4 sonars mark/clear cells within 5 m
- `inflation_layer` — `cost_scaling_factor: 3.0`, `inflation_radius: 1.5`

**Local costmap (15×15 m, 0.15 m/cell, rolling window)**:
- Same obstacle + inflation layers, no static layer

**Controller** — `RegulatedPurePursuitController`
- `desired_linear_vel: 1.5`
- `lookahead: 3.0 m` (1.0–6.0)
- `use_collision_detection: false` (only 4 short sonars; can't populate a full check envelope)
- `xy_goal_tolerance: 0.15` (RTK-precision)

**Behavior tree** — default `bt_navigator` graph with `navigate_to_pose_w_replanning_and_recovery.xml`.

### 4.4 The occupancy grid — `maps/novation_city.pgm`

3-tier encoding (not the usual binary):

| PGM pixel | Meaning | Nav2 occupancy | % of map |
|---:|---|---:|---:|
| 0   | Building (LETHAL) | 100 | 1.26 % |
| 100 | Off-road (mid-cost) | ≈61 | 92.82 % |
| 254 | Road (FREE) | 0 | 5.92 % |

YAML thresholds: `free_thresh: 0.20`, `occupied_thresh: 0.65` — so the off-road tier lands **between** thresholds and is preserved as a **cost gradient** rather than becoming binary free/occupied. This produces the "prefer roads but cross off-road when needed" planner behavior.

Origin `[-600.0, -600.0]` places world (0,0) at the map's centre pixel (600, 600).

### 4.5 MCP server — 29 tools

`robo_fleet/mcp_server/tools/` — every function decorated with `@mcp.tool()` is auto-registered by FastMCP and exposed via **both** stdio and streamable-http.

| Group | Tools | Purpose |
|---|---|---|
| Navigation | `navigate_to_pose`, `navigate_waypoints` | Send a single goal / a list of waypoints |
| Monitoring | `get_robot_position`, `get_fleet_status`, `get_battery_level` | Read live cached state (O(1)) |
| Control | `stop_robot`, `emergency_stop` | Cancel goals + zero cmd_vel |
| Sensing | `check_obstacles` | Query nearest scan return |
| Visualization | `get_map_with_robots` | Return PNG bytes of the map with robot markers |
| Coordination | `assign_tasks`, `dispatch_tasks`, `get_plan`, `replan`, `set_robot_priority`, `configure_fleet` | Task planner primitives |
| Advanced | `predict_collisions`, `add_task_to_queue`, `get_queue`, `clear_queue`, `start_auto_dispatch`, `stop_auto_dispatch`, `start_dashboard`, `stop_dashboard`, `assign_tasks_optimal` | Task queue + dashboard + Hungarian assignment |
| Locations | `list_locations`, `add_location`, `remove_location`, `go_to_location`, `send_nearest_to` | Named waypoints registry (`locations.json`) |

All tools use `ros/ros_client.py` — a WebSocket client that speaks the rosbridge JSON protocol (`op: publish/subscribe/call_service/send_action_goal`).

### 4.6 Fleet state manager — `coordination/fleet_state.py`

Singleton with a **persistent WebSocket** to rosbridge that subscribes to:
- `/{robot_id}/amcl_pose` → updates `robot.x, y, theta`
- `/{robot_id}/battery_state` → updates `robot.battery`
- `/{robot_id}/scan` → updates `robot.scan_closest`

All MCP monitoring tools query this in-memory cache (O(1)) instead of `subscribe_once` per call. Also runs a health-monitor thread that flips robots to `"offline"` if not seen for 5 s.

### 4.7 Dashboard server — `coordination/dashboard_server.py`

- **WebSocket server (:8090)** — pushes `{type:'fleet_state', robots:[...]}` at 5 Hz
- **rosbridge subscriber thread** (added this session) — subscribes to `/plan`, `/{rid}/plan`, `/navigate_to_pose/_action/status`, `/{rid}/navigate_to_pose/_action/status`
  - Forwards planned paths as `{type:'plan', robot_id, poses}` to browsers
  - Sets `_goal_status[rid]` from real Nav2 GoalStatusArray codes (executing/succeeded/aborted)
- **Chat proxy** — receives `{command:'chat', message}` from browser, forwards to `chat_agent.py`, broadcasts `{type:'chat_response'}` back
- **Nav command handler** — `{command:'navigate'}` opens rosbridge action goal on `/{rid}/navigate_to_pose`

### 4.8 Chat agent — `coordination/chat_agent.py`

- Uses the **official MCP client** — connects to itself via stdio and lists tools dynamically (no hardcoded schemas)
- Supports **Anthropic API** (`ANTHROPIC_API_KEY`) and **AWS Bedrock** (`--provider bedrock --model us.anthropic.claude-sonnet-4-20250514-v1:0`)
- Registers all 29 MCP tools as function-calling schema, then loops the model → tool_call → tool_result until final answer

### 4.9 Browser client — `dashboard/live_dashboard.{html,js}`

Custom canvas renderer (~350 LOC), no libraries:

- **World-metre coordinate system**, viewport = `(cx, cy, span)` — mouse wheel zooms around cursor, drag pans
- **Map background** — `novation_city_color.png` drawn with correct scale using yaml origin + resolution
- **Robot marker** — rotated true-scale rectangle (1.5 m × 1.1 m) with heading triangle + high-contrast highlight ring
- **Trail** (breadcrumbs), **planned path** (green polyline), **goal marker** (dashed line + target circle)
- **Sidebar** — robot cards with position/heading/goal/distance/status/battery + **live 3D chase-cam preview PNG** (auto-refresh every 3.5 s)
- **Chat panel** — WS-driven AI chat
- **Buttons** — `+` `−` `Fit` `Follow`
- **Click to navigate** — canvas pixel → world coord → nav goal on nearest online robot


---

## 5. Setup — Amazon Linux 2 / Any Linux Host (Docker)

This is the **primary supported path**. ROS 2 Jazzy and Gazebo Harmonic are not packaged for Amazon Linux 2, so we run everything in an Ubuntu-based container.

### 5.1 Prerequisites (host)

```bash
# 1. Docker
sudo yum install -y docker
sudo systemctl start docker
sudo usermod -aG docker "$USER"        # log out/in for group change
docker --version                        # confirm

# 2. Python 3.10+ (for the host cam-refresh loop & optional MCP scripts)
python3 --version

# 3. Build tools (optional, only for host-side helpers)
sudo yum install -y git curl unzip
```

### 5.2 Clone + build the image

```bash
cd ~
git clone <your-remote> ros2_outdoor_sim         # or scp your workspace
cd ros2_outdoor_sim

# ~15 min build. Installs ROS 2 Jazzy + Gazebo Harmonic + Nav2 + robot_localization
# + rosbridge_server + foxglove_bridge + Python MCP deps.
docker build -t outdoor-sim:jazzy .
```

### 5.3 Run the container

The container name **must be `pguard_sim`** (all scripts and Cursor MCP registration assume this):

```bash
docker run -d --name pguard_sim \
    --network host \
    -v "$PWD":/workspace \
    -w /workspace \
    --restart unless-stopped \
    outdoor-sim:jazzy \
    tail -f /dev/null
```

**Why `--network host`**: rosbridge on 9090, MCP HTTP on 8766, and Nav2's internal DDS all bind on the host network so external browsers/clients can reach them without port-mapping. On Docker Desktop (Mac/Win) `--network host` is not supported — publish ports explicitly instead:

```bash
docker run -d --name pguard_sim \
    -p 9090:9090 -p 8090:8090 -p 8766:8766 -p 8765:8765 \
    -v "$PWD":/workspace -w /workspace \
    --restart unless-stopped \
    outdoor-sim:jazzy tail -f /dev/null
```

### 5.4 Build the ROS 2 workspace (one-time)

```bash
docker exec -it pguard_sim bash -c '
    source /opt/ros/jazzy/setup.bash && \
    cd /workspace && \
    colcon build --symlink-install
'
```

`--symlink-install` means edits to Python scripts / launch files in `src/` take effect immediately on next launch — no rebuild needed for scripts. C++ code and CMakeLists changes still need a rebuild.

### 5.5 Verify the install

```bash
docker exec pguard_sim bash -c '
    source /opt/ros/jazzy/setup.bash && \
    source /workspace/install/setup.bash && \
    ros2 pkg list | grep my_pguard_bot && \
    gz sim --version && \
    ros2 pkg list | grep -E "nav2_bringup|rosbridge_server|ros_gz_bridge"
'
```

Expected output includes `my_pguard_bot`, `Gazebo Sim, version 8.x`, and the three ROS/Gazebo bridge packages.

---

## 6. Setup — Ubuntu 24.04 Native (no Docker)

Ubuntu 24.04 (Noble) is the **officially supported host** for ROS 2 Jazzy and Gazebo Harmonic — everything runs natively with GPU acceleration and the full Gazebo GUI.

### 6.0 One-command install (recommended)

The repo ships with an automated installer that runs every step in §6.1–§6.5 for you, idempotently:

```bash
# From the repo root, on a fresh Ubuntu 24.04 machine:
./scripts/setup_ubuntu_24_04.sh
```

Useful flags:

| Flag              | Effect                                                                  |
|-------------------|-------------------------------------------------------------------------|
| `--dry-run`       | Print every command that would be executed, without running any of them |
| `--check`         | Verify what's already installed (ROS 2, Gazebo, colcon, ROS packages, workspace build) |
| `--no-python`     | Skip `pip install -r robo_fleet/requirements.txt`                       |
| `--no-build`      | Skip the final `colcon build --symlink-install`                         |
| `-h`, `--help`    | Show usage                                                              |

The script:

1. Checks you're on Ubuntu Noble (warns otherwise, keeps going)
2. Installs base system packages (curl, git, cmake, build-essential, imagemagick, …)
3. Adds the ROS 2 apt repo, installs Jazzy Desktop + Nav2 + robot_localization + rosbridge + foxglove-bridge + colcon
4. Adds the OSRF apt repo, installs Gazebo Harmonic + `ros_gz_bridge` + `ros_gz_sim`
5. Installs GDAL (for OSM map generation)
6. Installs Python deps from `robo_fleet/requirements.txt` (with `--break-system-packages --user` to satisfy PEP 668)
7. Appends the ROS 2 and workspace `setup.bash` lines to your `~/.bashrc`
8. Runs `colcon build --symlink-install`

At the end it prints the exact `ros2 launch` command to bring the stack up with Gazebo GUI + RViz2. Re-run any time — every install step checks whether it's already done, so re-runs are cheap.

If you'd rather do it by hand (or need to understand what the script does), the manual steps are below.

### 6.1 System packages

```bash
sudo apt update && sudo apt install -y \
    curl gnupg lsb-release ca-certificates \
    software-properties-common python3-pip \
    build-essential cmake git
```

### 6.2 ROS 2 Jazzy

```bash
# ROS 2 apt repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update && sudo apt install -y \
    ros-jazzy-desktop-full \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-robot-localization ros-jazzy-slam-toolbox \
    ros-jazzy-xacro \
    ros-jazzy-rosbridge-server \
    ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions python3-rosdep

sudo rosdep init && rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 6.3 Gazebo Harmonic + ROS bridge

```bash
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt update && sudo apt install -y \
    gz-harmonic \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
```

### 6.4 Python + Python deps

```bash
sudo apt install -y python3-pip gdal-bin python3-gdal
cd ~/ros2_outdoor_sim
pip install --break-system-packages -r robo_fleet/requirements.txt
```

### 6.5 Build the workspace

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_outdoor_sim
colcon build --symlink-install
source install/setup.bash
```

### 6.6 GPU note

Gazebo Harmonic uses OGRE 2 which needs OpenGL 3.3+. If you have no GPU, force software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=offscreen   # if you also want no GUI
```

But performance will be very slow — this is why the Amazon Linux 2 devdesk runs headless.


---

## 7. Running the Full Stack

The order matters: **sim → localization → Nav2 → adapter → rosbridge → MCP server → dashboard**. The `full_stack.launch.py` handles the first five with proper `TimerAction` delays.

### 7.1 Terminal 1 — sim + Nav2 + rosbridge

```bash
docker exec -it pguard_sim bash -c '
    source /opt/ros/jazzy/setup.bash && \
    source /workspace/install/setup.bash && \
    ros2 launch my_pguard_bot full_stack.launch.py
'
```

Wait ~25 s for Gazebo to spawn PGuard, then ~12 s more for Nav2 lifecycle nodes to activate. Successful startup shows:

- `[gz sim] Loaded 26 building models, 1090 road segments`
- `[map_server]: New map published on /map`
- `[bt_navigator-*]: Configured. Activating.`
- `[static_layer]: Subscribing to map topic`

### 7.2 Terminal 2 — MCP over HTTP (optional)

Only needed for HTTP MCP clients (Continue, LangChain, curl). Cursor / Claude Desktop use stdio directly via `mcp_pguard.sh` and don't need this.

```bash
./robo_fleet/serve_http.sh
# Binds 0.0.0.0:8766 - MCP endpoint at http://<host>:8766/mcp
```

### 7.3 Terminal 3 — Dashboard WS + LLM chat + HTTP static

Split into two commands:

```bash
# 3a. HTTP server for the browser (host, not container - simpler + independent of ROS)
cd ~/ros2_outdoor_sim/robo_fleet/dashboard && \
    python3 -m http.server 8091 --bind 0.0.0.0 &

# 3b. Dashboard WS + chat agent (Bedrock example)
export AWS_REGION=us-east-1
cd ~/ros2_outdoor_sim/robo_fleet
python3 -u start_dashboard.py \
    --rosbridge localhost --port 9090 \
    --dashboard-port 8090 \
    --robots pguard \
    --provider bedrock \
    --model us.anthropic.claude-sonnet-4-20250514-v1:0
```

Anthropic API alternative:
```bash
ANTHROPIC_API_KEY=sk-ant-... python3 -u start_dashboard.py \
    --rosbridge localhost --port 9090 --dashboard-port 8090 \
    --robots pguard --provider anthropic
```

No-LLM mode (map+control still work, chat panel is disabled):
```bash
python3 -u start_dashboard.py --rosbridge localhost --dashboard-port 8090 --robots pguard
```

### 7.4 Terminal 4 — camera refresh loop

Streams the real Gazebo 3D chase-cam frames into `dashboard/pguard_chase.png` every 3 s so the browser preview stays live:

```bash
~/ros2_outdoor_sim/robo_fleet/dashboard/refresh_cam.sh
```

### 7.5 Verify all ports

```bash
ss -tlnp | grep -E ':(8090|8091|8765|8766|9090)'
```

Expected:
```
LISTEN  0.0.0.0:8090  (dashboard WS)
LISTEN  0.0.0.0:8091  (static HTTP)
LISTEN  0.0.0.0:8765  (foxglove bridge, optional)
LISTEN  0.0.0.0:8766  (MCP streamable-http, optional)
LISTEN  0.0.0.0:9090  (rosbridge WebSocket)
```

### 7.6 Verify the sim

```bash
docker exec pguard_sim bash -c '
    source /opt/ros/jazzy/setup.bash && \
    source /workspace/install/setup.bash && \
    ros2 action list | grep navigate_to_pose && \
    ros2 topic hz /map --window 1 && \
    ros2 topic hz /odometry/filtered --window 5
'
```


---

## 8. Web Dashboard

### 8.1 Open in browser

Local (same machine as sim):
```
http://localhost:8091/live_dashboard.html?ws=ws://localhost:8090
```

Remote (via SSH tunnel — see §11):
```
http://localhost:8091/live_dashboard.html?ws=ws://localhost:18090
```

### 8.2 Features

| Element | Description |
|---|---|
| Map canvas | 1200×1200 m colored OSM background (buildings=red, roads=tan, off-road=blue-grey) |
| Robot marker | True-scale 1.5×1.1 m rotated rectangle with heading triangle + highlight ring |
| Green polyline | Live Nav2 planned path from `/plan` |
| Blue dashed line + circle | Active goal + goal→robot line |
| Trail | Purple breadcrumbs of PGuard's actual path (last 500 waypoints) |
| Sidebar - robot card | Position (x,y,θ), goal coords, distance-to-goal, real Nav2 status (idle/navigating/offline), battery bar |
| Sidebar - PGuard preview | LIVE 3D chase-cam PNG, auto-refreshes every 3.5 s, manual `↻` refresh button |
| Chat panel | AI chat backed by Bedrock/Anthropic through the MCP tool set |
| Buttons | `+` `−` zoom, `Fit` (whole city), `◎` (follow robot) |
| Interactions | Mouse wheel = zoom around cursor, drag = pan, click = send nearest robot to that world coord |

### 8.3 Message protocol (dashboard WS)

Server → client:
```json
{"type":"fleet_state", "timestamp":1783872212.9, "robots":[
  {"id":"pguard", "x":5.17, "y":5.10, "theta":2.85, "battery":100.0,
   "status":"navigating", "online":true, "goal":{"x":10.0,"y":10.0}}
]}

{"type":"plan", "robot_id":"pguard", "poses":[{"x":5.0,"y":5.0}, {"x":6.0,"y":6.0}, ...]}

{"type":"chat_response", "message":"PGuard is at (5.2, 5.1) heading NE.", "tool_used":"get_robot_position"}
```

Client → server:
```json
{"command":"navigate", "robot_id":"pguard", "x":10.0, "y":10.0}
{"command":"chat", "message":"where is pguard?"}
```


---

## 9. MCP Server Setup

The `robo_fleet` MCP server exposes 29 ROS 2 fleet operations to any MCP client. It ships **two transports out of the same code**:

### 9.1 Transport A — stdio (Cursor, Claude Desktop, subprocess clients)

The stdio wrapper `mcp_pguard.sh` executes the server inside the container:

```bash
#!/bin/bash
exec docker exec -i pguard_sim bash -c '
    cd /workspace/robo_fleet/mcp_server && \
    PYTHONUNBUFFERED=1 python3 -u index.py
'
```

**Register in Cursor** (`~/.cursor/mcp.json`):

```json
{
  "mcpServers": {
    "pguard-fleet": {
      "command": "/home/tastouri/ros2_outdoor_sim/mcp_pguard.sh"
    }
  }
}
```

**Register in Claude Desktop** (`~/Library/Application Support/Claude/claude_desktop_config.json` on macOS, `%APPDATA%\Claude\claude_desktop_config.json` on Windows):

```json
{
  "mcpServers": {
    "pguard-fleet": {
      "command": "ssh",
      "args": ["devdesk", "/home/tastouri/ros2_outdoor_sim/mcp_pguard.sh"]
    }
  }
}
```

(Replace `devdesk` with your SSH host alias if the sim runs remotely.)

Restart Cursor / Claude Desktop after registering. Tools should appear as `mcp_pguard-fleet_navigate_to_pose`, `mcp_pguard-fleet_get_robot_position`, etc.

### 9.2 Transport B — streamable-http

```bash
./robo_fleet/serve_http.sh
# equivalent to:
docker exec -it pguard_sim bash -c '
    cd /workspace/robo_fleet/mcp_server && \
    python3 index.py --transport http --host 0.0.0.0 --port 8766
'
```

Endpoint: `http://<host>:8766/mcp`

Python client example:
```python
import asyncio
from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client

async def main():
    async with streamablehttp_client("http://localhost:8766/mcp") as (r, w, _):
        async with ClientSession(r, w) as session:
            await session.initialize()
            tools = await session.list_tools()
            print(f"{len(tools.tools)} tools available")
            res = await session.call_tool("get_robot_position", {"robot_id": "pguard"})
            print(res.content[0].text)

asyncio.run(main())
```

curl example (bare JSON-RPC over HTTP):
```bash
curl -X POST http://localhost:8766/mcp \
    -H "Content-Type: application/json" \
    -H "Accept: application/json, text/event-stream" \
    -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}'
```

### 9.3 Environment variables

Read by `robo_fleet/mcp_server/ros/ros_client.py`:

| Var | Default | Purpose |
|---|---|---|
| `ROBOFLEET_ROSBRIDGE_HOST` | `localhost` | rosbridge WebSocket host |
| `ROBOFLEET_ROSBRIDGE_PORT` | `9090` | rosbridge WebSocket port |
| `ROBOFLEET_TIMEOUT` | `10.0` | Per-call timeout in seconds |


---

## 10. LLM (Bedrock / Anthropic) Configuration

The dashboard chat is an **MCP client** that queries the same 29 tools programmatically — no hardcoded schemas.

### 10.1 Anthropic API

```bash
export ANTHROPIC_API_KEY=sk-ant-xxx
python3 start_dashboard.py --provider anthropic --model claude-3-5-sonnet-20241022 \
    --rosbridge localhost --dashboard-port 8090 --robots pguard
```

### 10.2 AWS Bedrock

Cross-region **inference profiles are mandatory** for Claude Sonnet 4 — the raw model ID `anthropic.claude-sonnet-4-20250514-v1:0` errors with:
```
ValidationException: Invocation of model ID ... with on-demand throughput isn't supported.
```

Use the `us.` prefix (US inference profile):

```bash
# Refresh AWS creds (Amazon devdesks use ada)
ada

# Confirm identity
aws sts get-caller-identity

# List available inference profiles
aws bedrock list-inference-profiles --region us-east-1 \
    --query 'inferenceProfileSummaries[?contains(inferenceProfileId, `sonnet-4`)]'

# Run
export AWS_REGION=us-east-1
python3 -u start_dashboard.py \
    --provider bedrock \
    --model us.anthropic.claude-sonnet-4-20250514-v1:0 \
    --rosbridge localhost --dashboard-port 8090 --robots pguard
```

### 10.3 Try in the chat panel

- `where is pguard?` → calls `get_robot_position`
- `send pguard to (50, 30)` → calls `navigate_to_pose`
- `patrol the perimeter` → calls `navigate_waypoints` with `patrol_waypoints.yaml`
- `fleet status` → calls `get_fleet_status`
- `stop everything` → calls `emergency_stop`

---

## 11. Remote Access via SSH Tunnel

When the sim runs on a remote devdesk and you want to open the dashboard from your laptop:

### 11.1 The tunnel

```bash
# From your laptop
ssh -N \
    -L 8091:localhost:8091 \
    -L 18090:localhost:8090 \
    -L 18766:localhost:8766 \
    devdesk
```

Note the **local-side port remap** for `8090` → `18090` — this avoids collision with a common laptop service that binds `:8090`.

If ports `8091` or `18090` are already in use on the laptop, pick different local ports and update the URL accordingly.

### 11.2 Open the dashboard

```
http://localhost:8091/live_dashboard.html?ws=ws://localhost:18090
```

The `?ws=` query string tells the browser JS which WebSocket URL to connect to. The static assets (`novation_city_color.png`, `pguard_chase.png`, `live_dashboard.js`) all come from `:8091` on your laptop → tunnel → `:8091` on the devdesk.

### 11.3 Register the MCP server remotely (Cursor on laptop)

```json
{
  "mcpServers": {
    "pguard-fleet": {
      "command": "ssh",
      "args": ["devdesk", "/home/tastouri/ros2_outdoor_sim/mcp_pguard.sh"]
    }
  }
}
```


---

## 12. Session Change-Log — What Was Fixed

This section records the concrete engineering work done in the most recent session (July 12, 2026).

### 12.1 Robot could drive through buildings and off-road

**Symptom**: PGuard's Nav2 paths cut straight across buildings and off-road terrain.

**Root cause** — three separate bugs:

1. `scripts/build_map.py` only rasterized building polygons; roads were never stamped, so the map was 98.74 % "free" and Nav2 had no reason to prefer roads.
2. `config/nav2_params.yaml` was missing `static_layer` from `global_costmap.plugins` — so even after fixing the map, Nav2 wasn't consuming it.
3. `CMakeLists.txt`'s `install(DIRECTORY ...)` list didn't include `maps/`, so the yaml/pgm never made it into `install/share/my_pguard_bot/maps/` and `map_server` failed with "bad file novation_city.yaml".

**Fix**:
- Rewrote `build_map.py` to parse both `bld_*` and `road_*` from `sousse_buildings.sdf` and rasterize 3 tiers (0=LETHAL, 100=NO_INFO, 254=FREE). New distribution: 1.26 % buildings / 5.92 % roads / 92.82 % off-road.
- Added `static_layer` to `global_costmap.plugins` with `trinary_costmap: false` so off-road stays as a cost gradient (planner avoids but can cross).
- Added `maps` to the CMakeLists install list; rebuilt with `colcon build --symlink-install`.

**Verification**: `ros2 topic info /map` shows publisher=1 subscription=1 (map_server → static_layer wired).

### 12.2 Dashboard robot marker invisible on 400 m map

**Symptom**: after fixing the map, the dashboard's tiny 4 m × 4 m viewport hid the robot at real-world scale.

**Fix**: full dashboard rewrite (`live_dashboard.html` + new `live_dashboard.js`):
- Viewport now in **world metres**, default 400 m span, zoom/pan supported
- Loads `novation_city_color.png` as background at correct scale using yaml origin + resolution
- Robot rendered at **true 1.5×1.1 m dimensions** with heading triangle + highlight ring so it stays visible at any zoom
- Sidebar shows position, heading, goal, distance-to-goal in metres

### 12.3 "Idle but keeps running"

**Symptom**: dashboard showed PGuard as "idle" while the robot was actively navigating.

**Root cause**: `dashboard_server._send_nav_goal` flipped `robot.status = "idle"` after its own 60 s `wait_for_result` timeout, decoupled from actual Nav2 state.

**Fix**: added a persistent rosbridge subscriber thread (`_ros_subscriber_loop`) in `dashboard_server.py` that:
- Subscribes to `/pguard/navigate_to_pose/_action/status` (GoalStatusArray)
- Maps status codes (executing → `"navigating"`, succeeded/aborted/canceled → `"idle"`)
- `_get_fleet_state` now prefers this real Nav2 status over the stale cache

### 12.4 No planned-path visualization

**Symptom**: user couldn't see WHY the robot took a strange path.

**Fix**: the same subscriber thread now also subscribes to `/plan` and `/pguard/plan`, forwards them to browsers as `{type:'plan', robot_id, poses:[...]}`, and the JS renders them as a **green polyline**. On goal-terminal (success/abort/cancel) the plan is cleared and the goal marker removed.

### 12.5 Cannot see the real 3D robot

**Symptom**: user wanted to see PGuard's actual 3D chassis, not just a top-down rectangle.

**Fix**:
- `scripts/grab_gz_frame.py` (host-side helper stashed in `/tmp`) captures `gz.msgs.Image` frames from `/world_cam/chase` via `gz topic -e --json-output` (base64 → numpy → PNG).
- `scripts/aim_chase_cam.py` computes a look-at quaternion from the camera position to the robot's live pose and moves `chase_cam` via `gz service /world/novation_city/set_pose`.
- `robo_fleet/dashboard/refresh_cam.sh` background loop repositions + re-captures every 3 s → `pguard_chase.png`.
- Dashboard sidebar shows the PNG with a `↻` button and auto-refreshes every 3.5 s.

### 12.6 Bedrock inference-profile error

**Symptom**: `ValidationException: Invocation of model ID anthropic.claude-sonnet-4-20250514-v1:0 with on-demand throughput isn't supported.`

**Fix**: use the cross-region inference profile ID `us.anthropic.claude-sonnet-4-20250514-v1:0` (discovered via `aws bedrock list-inference-profiles`).


---

## 13. Troubleshooting

### 13.1 Container starts but ROS commands fail

Symptom: `bash: ros2: command not found` or `install/setup.bash: No such file or directory`.

Fix — always source both:
```bash
docker exec -it pguard_sim bash -c '
    source /opt/ros/jazzy/setup.bash && \
    source /workspace/install/setup.bash && \
    <your command>'
```

### 13.2 `map_server` fails: "bad file novation_city.yaml"

The `maps/` directory wasn't installed. Ensure `CMakeLists.txt` contains `maps` in the `install(DIRECTORY ...)` list, then:
```bash
docker exec pguard_sim bash -c 'cd /workspace && colcon build --symlink-install'
```

### 13.3 `/map` has publisher=1 subscription=0

`static_layer` isn't in `global_costmap.plugins`. Check `nav2_params.yaml`:
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

### 13.4 Robot invisible on the dashboard

- Check dashboard opened with `?ws=ws://...` param (default is `ws://localhost:8080` — wrong port)
- Try the `Fit` button to reset viewport to whole city
- Check `MAP_MIN/MAX` are gone — the new JS uses `view.span` (default 400 m). If you see 4×4 m, you're on the old cached HTML — hard-refresh (Ctrl-Shift-R).

### 13.5 Chat panel says "LLM not configured"

`ANTHROPIC_API_KEY` unset AND `--provider bedrock` not passed. See §10.

### 13.6 Bedrock: model ID doesn't support on-demand throughput

Use `us.anthropic.claude-sonnet-4-20250514-v1:0` (with `us.` prefix). See §10.2.

### 13.7 SSH tunnel: "bind: address already in use"

Some port on the laptop is taken. Remap local side:
```bash
ssh -N -L 18091:localhost:8091 -L 28090:localhost:8090 devdesk
# then use http://localhost:18091/live_dashboard.html?ws=ws://localhost:28090
```

### 13.8 Chase-cam PNG never updates

- Confirm the refresh loop is running: `pgrep -f refresh_cam.sh`
- Check its log: `tail -f /tmp/refresh_cam.log`
- Verify `docker exec pguard_sim gz topic -l | grep chase` returns `/world_cam/chase`
- Confirm PGuard's pose is publishing: `docker exec pguard_sim bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic echo --once /pguard/amcl_pose'`

### 13.9 Container exit 137

That's SIGKILL — the container itself was force-killed, usually because the host ran out of memory. Look at `journalctl -u docker` or `dmesg | tail`.

### 13.10 Gazebo core dumps (`core.NNNN` files)

The workspace root sometimes accumulates `core.2540`, `core.3242` from crashed `gz sim` processes. Safe to delete:
```bash
rm -f ~/ros2_outdoor_sim/core.*
```

---

## 14. Reference — Ports, Topics, Actions

### 14.1 Network ports

| Port | Service | Container/Host |
|---:|---|---|
| 9090 | rosbridge WebSocket | Container |
| 8765 | foxglove_bridge | Container |
| 8766 | robo_fleet MCP HTTP | Container |
| 8090 | dashboard WS | Container |
| 8091 | dashboard static HTTP | Host |

### 14.2 Key ROS 2 topics

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 → Gazebo velocity |
| `/pguard/cmd_vel` | `geometry_msgs/msg/TwistStamped` | External command in |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF-fused odom |
| `/pguard/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Fleet-state input |
| `/sonar/{front,rear,left,right}` | `sensor_msgs/msg/Range` | Raw ultrasonics |
| `/pguard/scan` | `sensor_msgs/msg/LaserScan` | 4-ray synth from sonars |
| `/pguard/battery_state` | `sensor_msgs/msg/BatteryState` | Synthetic 100 % |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 1200×1200 static map |
| `/plan` | `nav_msgs/msg/Path` | Nav2 planned path |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Merged cost layers |

### 14.3 ROS 2 actions

| Action | Type |
|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |
| `/pguard/navigate_to_pose` | Same (relayed by adapter) |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` |
| `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` |

### 14.4 Gazebo topics

| Topic | Purpose |
|---|---|
| `/world_cam/chase` | 960×540 chase camera (moved to follow PGuard) |
| `/world_cam/top` | 1024×1024 top-down camera |
| `/world/novation_city/set_pose` | Service to teleport any model/camera |


---

## 15. File-by-File Index

### 15.1 Root

| File | Purpose |
|---|---|
| `Dockerfile` | Base image: ROS 2 Jazzy + Gazebo Harmonic + Nav2 + rosbridge + Python MCP deps |
| `README.md` | Quickstart |
| `mcp_pguard.sh` | Cursor / Claude stdio MCP wrapper (docker exec -i) |
| `run_tests.py` | Full test harness for MCP tools + Nav2 goals |
| `run_dashboard.py` | Small launcher helper (legacy) |
| `pguard_dashboard.html` | Legacy monolithic dashboard (superseded by `robo_fleet/dashboard/`) |
| `scripts/setup_ubuntu_24_04.sh` | **Automated Ubuntu 24.04 installer** (ROS 2 Jazzy + Gazebo Harmonic + Nav2 + Python deps + colcon build). See §6.0. Supports `--dry-run`, `--check`, `--no-build`, `--no-python`. |

### 15.2 `src/my_pguard_bot/`

| File | Purpose |
|---|---|
| `CMakeLists.txt` | Installs `launch/ config/ description/ worlds/ maps/ scripts/ rviz/ foxglove/` to `share/` |
| `package.xml` | ROS 2 package metadata |
| `launch/full_stack.launch.py` | sim + localization + Nav2 + adapter (canonical entry point) |
| `launch/sim.launch.py` | Just Gazebo + ros_gz_bridge + world cameras |
| `launch/localization.launch.py` | Dual EKF (odom + map) + navsat_transform |
| `launch/robofleet.launch.py` | full_stack + robo_fleet_adapter |
| `launch/viz.launch.py` | rosbridge + foxglove + map_server + amcl |
| `launch/patrol.launch.py` | Waypoint patrol demo |
| `config/nav2_params.yaml` | 3-layer global costmap, RegulatedPurePursuit controller |
| `config/ekf.yaml` | Two EKF instances: local (odom) + global (map + GPS) |
| `config/bridge.yaml` | ros_gz_bridge topic map |
| `config/patrol_waypoints.yaml` | Named perimeter waypoints |
| `worlds/novation_city.sdf` | Generated world (buildings + roads + cameras) |
| `worlds/sousse_buildings.sdf` | Generated: 26 buildings + ~1000 road segments |
| `maps/novation_city.pgm` | 3-tier occupancy grid (0/100/254) |
| `maps/novation_city.yaml` | resolution=1.0, origin=[-600,-600,0], thresholds 0.20/0.65 |
| `scripts/fetch_osm.py` | Overpass API → OSM buildings + roads + POIs |
| `scripts/build_world.py` | OSM → SDF `bld_*` + `road_*` models |
| `scripts/build_map.py` | OSM → 3-tier PGM (this session's rewrite) |
| `scripts/robo_fleet_adapter.py` | ROS 2 node bridging PGuard ↔ robo_fleet conventions |
| `scripts/patrol_client.py` | NavigateThroughPoses action client |
| `scripts/save_camera_frame.py` | gz.msgs.Image → PNG helper |

### 15.3 `robo_fleet/`

| File | Purpose |
|---|---|
| `requirements.txt` | `mcp[cli]`, `websockets`, `websocket-client`, `uvicorn`, `starlette`, `pydantic`, `numpy` |
| `start_dashboard.py` | Launches dashboard WS + chat agent (Anthropic / Bedrock) |
| `serve_http.sh` | Runs MCP streamable-http (:8766) inside `pguard_sim` |
| `README.md` | Package-level docs |
| `dashboard/live_dashboard.html` | Browser UI shell (canvas + sidebar + chat) |
| `dashboard/live_dashboard.js` | Canvas renderer, WS client, zoom/pan, chat |
| `dashboard/novation_city_color.png` | RGBA 1200×1200 map (buildings=red/roads=tan/off-road=grey) |
| `dashboard/pguard_chase.png` | LIVE 960×540 Gazebo chase-cam frame (auto-updated) |
| `dashboard/pguard_top.png` | LIVE 1024×1024 top-down frame |
| `dashboard/refresh_cam.sh` | Background loop grabbing Gazebo frames every 3 s |
| `mcp_server/index.py` | Entrypoint — `--transport stdio` or `--transport http --host --port` |
| `mcp_server/server.py` | FastMCP instance (auto-collects `@mcp.tool()` decorators) |
| `mcp_server/locations.json` | Named locations registry |
| `mcp_server/ros/ros_client.py` | rosbridge JSON-RPC client (pub/sub/service/action) |
| `mcp_server/tools/navigation.py` | `navigate_to_pose` |
| `mcp_server/tools/waypoints.py` | `navigate_waypoints` |
| `mcp_server/tools/monitoring.py` | `get_robot_position`, `get_fleet_status`, `get_battery_level` |
| `mcp_server/tools/control.py` | `stop_robot`, `emergency_stop` |
| `mcp_server/tools/obstacles.py` | `check_obstacles` |
| `mcp_server/tools/map_viz.py` | `get_map_with_robots` (returns PNG bytes) |
| `mcp_server/tools/coordination.py` | `assign_tasks`, `dispatch_tasks`, `get_plan`, `replan`, `set_robot_priority`, `configure_fleet` |
| `mcp_server/tools/advanced.py` | `predict_collisions`, task queue, dashboard control, Hungarian assignment |
| `mcp_server/tools/natural_language.py` | Named locations: list/add/remove/go_to/send_nearest_to |
| `mcp_server/coordination/fleet_state.py` | Singleton rosbridge subscriber → in-memory fleet cache |
| `mcp_server/coordination/dashboard_server.py` | WS server + `/plan` subscriber + status forwarder + chat proxy |
| `mcp_server/coordination/chat_agent.py` | LLM MCP client (Anthropic + Bedrock) — no hardcoded schemas |
| `mcp_server/coordination/task_planner.py` | Nearest-neighbor task assignment |
| `mcp_server/coordination/task_queue.py` | Priority queue + auto-dispatch worker |
| `mcp_server/coordination/hungarian.py` | Optimal robot ↔ task assignment (Kuhn–Munkres) |
| `mcp_server/coordination/collision_predictor.py` | Forward-sim collision check between paths |

### 15.4 `docs/`

| File | Purpose |
|---|---|
| `PROJECT.md` | **THIS FILE** |
| `outdoor-sim-guide.md` | Earlier design notes |
| `novation_map_before_after.png` | Proof of the map-fix change (98.74 %-free → 3-tier) |
| `3d_gallery.png`, `3d_chase.png`, `3d_top.png`, `3d_isometric.png` | Real Gazebo 3D captures |
| `01_novation_full.png` … `05_costmap_stack.png` | Synthetic top-down gallery |
| `robot_*.png` | Various robot-in-map render iterations |

---

## Appendix — One-liner smoke test

```bash
# From anywhere on the LAN with the tunnel up:
python3 - << 'PY'
import websocket, json, time, threading
msgs=[]
ws=websocket.WebSocketApp("ws://localhost:18090", on_message=lambda w,m: msgs.append(m))
threading.Thread(target=ws.run_forever, daemon=True).start()
time.sleep(2)
ws.send(json.dumps({"command":"navigate","robot_id":"pguard","x":15.0,"y":10.0}))
time.sleep(8); ws.close()
plans = [m for m in msgs if '"type": "plan"' in m]
states = [m for m in msgs if '"type": "fleet_state"' in m]
print(f"OK  plans={len(plans)}  fleet_state={len(states)}")
PY
```

Expect `plans>=1` and `fleet_state>=20` — that's a full end-to-end confirmation that Gazebo is running, Nav2 planned a path, rosbridge forwarded it, and the dashboard WS pushed it to the client.


---

## 16. Running Natively on Linux WITH the GUIs (no Docker)

If you clone this workspace on a **GPU-capable Ubuntu 24.04** host, everything from
Gazebo Harmonic's full 3D GUI to RViz2's Nav2 panel works out of the box — Docker is
only used on the Amazon Linux 2 devdesk because ROS 2 Jazzy isn't packaged there.

### 16.1 Prerequisites

Follow §6 (Setup — Ubuntu 24.04 Native) first. Confirm you have a working display:

```bash
echo $DISPLAY                     # should be :0 or :1
glxinfo | grep 'OpenGL renderer'  # should mention your GPU, not "llvmpipe"
```

If `llvmpipe` is your renderer, you have no GPU acceleration — Gazebo GUI will
technically run but very slowly on a 1200 m × 1200 m world. In that case stick
with the headless dashboard flow (§7).

### 16.2 Build the workspace

```bash
cd ~/ros2_outdoor_sim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 16.3 One-liner: sim + Nav2 + Gazebo GUI + RViz2

```bash
ros2 launch my_pguard_bot full_stack.launch.py use_gui:=true rviz:=true
```

That single command starts, in order:

1. **`gz sim -s`** — the physics + sensor server
2. **`gz sim -g`** — the Gazebo GUI window (thanks to `use_gui:=true`)
3. `robot_state_publisher` + `ros_gz_sim create` — spawns PGuard at (0, 0, 0.4)
4. `ros_gz_bridge` — mirrors Gazebo ↔ ROS 2 topics per `config/bridge.yaml`
5. `viz.launch.py` — `rosbridge_server` (:9090), `foxglove_bridge` (:8765), `map_server`, `amcl`
6. Dual EKF (`ekf.yaml`) after a 3 s delay
7. Full Nav2 stack (7 lifecycle nodes) after a 6 s delay
8. **RViz2** with `rviz/pguard_nav2.rviz` (thanks to `rviz:=true`) showing the
   robot model, /map, both costmaps, /plan, sonar rays, and TF frames

The Nav2 panel appears in RViz automatically because `pguard_nav2.rviz` lists
`nav2_rviz_plugins/Navigation 2` in its `Panels:` section. Use the `Nav2 Goal`
toolbar button (or press `G`) to click a goal in the 3D view.

### 16.4 What each GUI gives you

**Gazebo GUI** (`gz sim -g` window):
- Free-fly 3D view of Novation City with all 26 buildings and ~1000 road segments
- Right-click any object → **Move To** to teleport your camera
- **Right panel → Entity Tree** shows every model (buildings, roads, `pguard`,
  `chase_cam`, `top_cam`)
- Bottom timeline for play / pause / reset simulation
- **Component Inspector** lets you edit poses of anything at runtime — useful for
  spot-checking the OSM-generated buildings

**RViz2** (`pguard_nav2.rviz`):
- Robot model rendered from `pguard.urdf.xacro` in the correct pose
- `/map` (the 3-tier PGM) drawn under the world, correctly aligned via the yaml
  origin at (-600, -600)
- **Global Costmap** and **Local Costmap** overlays — you can see the inflation
  layers and off-road cost gradient live
- `/plan` (green line) and `/local_plan` (cyan line)
- Sonar scan points from `/pguard/scan`
- **Toolbar buttons**:
  - `2D Pose Estimate` publishes `/initialpose` (feeds AMCL)
  - `2D Goal Pose` publishes `/goal_pose` and starts a NavigateToPose action
  - `Publish Point` publishes `/clicked_point` (useful for waypoints)
- **Nav2 panel** (bottom-left) exposes:
  - `Startup` / `Reset` / `Pause` / `Resume` for the lifecycle manager
  - `Feedback` with distance-remaining, ETA, and BT node currently running
  - `Waypoint mode` — click multiple goals in sequence
  - Cancel current goal button

### 16.5 Just Gazebo GUI, no Nav2

Useful for world editing / spawn testing:
```bash
ros2 launch my_pguard_bot sim.launch.py use_gui:=true
```

### 16.6 Just RViz2 against a running headless sim

If the sim is running headless on this machine or a remote:
```bash
rviz2 -d $(ros2 pkg prefix my_pguard_bot)/share/my_pguard_bot/rviz/pguard_nav2.rviz
```

For a remote sim, either enable X11 forwarding or use rosbridge on port 9090
with Foxglove Studio at `ws://<host>:9090`.

### 16.7 Foxglove Studio

Foxglove is a good middle ground — much lighter than RViz, works over WebSocket
so you can point it at a headless remote sim:

1. Install Foxglove Studio (AppImage on Linux)
2. Open connection → **Rosbridge (ROS 1 & 2)** → `ws://<host>:9090`
3. Load `src/my_pguard_bot/foxglove/*.json` layouts if present

### 16.8 Sending goals without a GUI

You always have three programmatic options in addition to the GUIs:

```bash
# 1. Command-line
ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped \
    '{header: {frame_id: map}, pose: {position: {x: 15.0, y: 10.0}, orientation: {w: 1.0}}}'

# 2. Web dashboard (see §7-8)
python3 robo_fleet/start_dashboard.py --robots pguard --dashboard-port 8090

# 3. MCP client (Cursor, Claude Desktop, any HTTP client — see §9)
```

### 16.9 Comparison: Docker (headless) vs Native (GUI)

| Feature | Docker on AL2 (headless) | Native Ubuntu 24.04 (GUI) |
|---|:---:|:---:|
| Gazebo physics + topics | ✅ | ✅ |
| Nav2 planning + control | ✅ | ✅ |
| ROS 2 CLI (`ros2 topic`, `ros2 action`) | ✅ | ✅ |
| rosbridge / MCP / web dashboard | ✅ | ✅ |
| **Gazebo 3D GUI** | ❌ | ✅ |
| **RViz2 GUI + Nav2 panel** | ❌ | ✅ |
| **Interactive goal clicks in RViz** | ❌ | ✅ |
| **Foxglove Studio (native)** | ❌ (use WS remotely) | ✅ |
| GPU / OpenGL 3.3+ required | no | yes |

Everything **built** for this project (launch files, Nav2 params, map, MCP tools,
adapter, dashboard) is identical across both paths — only the "how do I look at
the sim" layer changes.

