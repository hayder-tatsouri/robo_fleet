# robo_fleet — MCP server for ROS 2 robot fleets

A [Model Context Protocol](https://modelcontextprotocol.io) server that exposes
ROS 2 multi-robot operations as AI-callable tools. Any MCP client (Cursor,
Claude Desktop, Continue, LangChain, custom code, etc.) can drive the fleet.

Ships two transports out of the same server:

| Transport | Endpoint | Who uses it |
|---|---|---|
| **stdio** | subprocess pipe | Cursor, Claude Desktop, `mcp` CLI, subprocess clients |
| **streamable-http** | `http://<host>:8766/mcp` | Continue, LangChain, curl, remote peers, browser inspectors |

## Layout

```
robo_fleet/
├── mcp_server/
│   ├── index.py                # entrypoint - picks transport via --transport
│   ├── server.py               # FastMCP instance
│   ├── ros/ros_client.py       # rosbridge WebSocket client
│   ├── tools/                  # @mcp.tool() implementations (29 tools)
│   └── coordination/
│       ├── chat_agent.py       # LLM chat agent - real MCP client (see below)
│       ├── dashboard_server.py # WebSocket server for the web dashboard
│       ├── fleet_state.py      # live fleet-state manager
│       └── task_planner.py, task_queue.py, hungarian.py, collision_predictor.py
├── dashboard/live_dashboard.html
├── start_dashboard.py          # launches dashboard + chat agent
├── serve_http.sh               # starts the MCP server over HTTP inside pguard_sim
├── requirements.txt
└── README.md
```

## Transports

### stdio (Cursor, Claude Desktop, subprocess clients)

`~/ros2_outdoor_sim/mcp_pguard.sh` wraps `docker exec -i pguard_sim` so
Cursor and friends see a plain local stdio MCP server.

`~/.cursor/mcp.json` already registers it as `pguard-fleet`.

Manual invocation:

```bash
python3 mcp_server/index.py                       # default
python3 mcp_server/index.py --transport stdio     # explicit
```

### streamable-http (any HTTP MCP client, LAN peers)

```bash
# From the host - hits the running container
./robo_fleet/serve_http.sh                        # binds 0.0.0.0:8766

# Or manually inside the container
docker exec -it pguard_sim bash -c '
  cd /workspace/robo_fleet/mcp_server && \
  python3 index.py --transport http --host 0.0.0.0 --port 8766
'
```

Reachable at:

- `http://localhost:8766/mcp` (loopback on the sim host)
- `http://<LAN-IP>:8766/mcp` (any peer on the same network)

Example client (works from anywhere on the LAN):

```python
import asyncio
from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client

async def main():
    async with streamablehttp_client("http://<host>:8766/mcp") as (r, w, _):
        async with ClientSession(r, w) as session:
            await session.initialize()
            tools = await session.list_tools()
            print(len(tools.tools), "tools")
            res = await session.call_tool("get_robot_position", {"robot_id": "pguard"})
            print(res.content[0].text)

asyncio.run(main())
```

## Prerequisites

The `outdoor-sim:jazzy` Docker image (built from `../Dockerfile`) already
includes everything: ROS 2 Jazzy, Gazebo Harmonic, Nav2, robot_localization,
rosbridge_server, and this package's Python deps (`requirements.txt`).

The container bind-mounts `~/ros2_outdoor_sim` to `/workspace`, so this
folder lives at `/workspace/robo_fleet` inside the container. **Edit files
from the host; changes are visible in the container immediately** — no
`docker cp`, no rebuild.

## Available tools

29 tools grouped by concern (see `mcp_server/tools/`):

| Group | Tools |
|---|---|
| Navigation | `navigate_to_pose`, `navigate_waypoints` |
| Monitoring | `get_robot_position`, `get_fleet_status`, `get_battery_level` |
| Control | `stop_robot`, `emergency_stop` |
| Sensing | `check_obstacles` |
| Visualization | `get_map_with_robots` |
| Coordination | `assign_tasks`, `dispatch_tasks`, `get_plan`, `replan`, `set_robot_priority`, `configure_fleet` |
| Advanced | `predict_collisions`, `add_task_to_queue`, `get_queue`, `clear_queue`, `start_auto_dispatch`, `stop_auto_dispatch`, `start_dashboard`, `stop_dashboard`, `assign_tasks_optimal` |
| Locations | `list_locations`, `add_location`, `remove_location`, `go_to_location`, `send_nearest_to` |

## Web dashboard

```bash
ANTHROPIC_API_KEY=sk-ant-... python3 start_dashboard.py \
    --rosbridge 240.10.0.3 --robots pguard --open
```

The dashboard's chat panel uses `coordination/chat_agent.py`, which is
itself an **MCP client** — no hardcoded tool schemas. New `@mcp.tool()`
decorators appear in the chat automatically on the next launch.

## License

MIT
