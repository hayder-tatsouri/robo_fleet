"""Entrypoint for the robo_fleet MCP server.

Supports two transports (any MCP client can connect):

    # stdio (Cursor, Claude Desktop, `mcp` CLI, subprocess clients)
    python3 index.py                          # default
    python3 index.py --transport stdio

    # streamable-http (Continue, LangChain, curl, another host on the LAN,
    #                  browser inspector, mobile clients, etc.)
    python3 index.py --transport http --host 0.0.0.0 --port 8766
    #   -> POST http://<host>:8766/mcp

    # SSE (older HTTP transport, kept for compatibility)
    python3 index.py --transport sse  --host 0.0.0.0 --port 8766
    #   -> GET  http://<host>:8766/sse

Environment overrides (used when CLI args are absent):

    ROBOFLEET_TRANSPORT=stdio|http|sse
    ROBOFLEET_HOST=0.0.0.0
    ROBOFLEET_PORT=8766
"""

import argparse
import os
import sys

from server import mcp

# Register all tools by importing the modules (each uses @mcp.tool()).
from tools.navigation import navigate_to_pose            # noqa: F401
from tools.waypoints import navigate_waypoints           # noqa: F401
from tools.monitoring import (                            # noqa: F401
    get_robot_position, get_fleet_status, get_battery_level,
)
from tools.control import stop_robot, emergency_stop     # noqa: F401
from tools.obstacles import check_obstacles              # noqa: F401
from tools.map_viz import get_map_with_robots            # noqa: F401
from tools.coordination import (                          # noqa: F401
    assign_tasks, dispatch_tasks, get_plan, replan,
    set_robot_priority, configure_fleet,
)
from tools.advanced import (                              # noqa: F401
    predict_collisions, add_task_to_queue, get_queue, clear_queue,
    start_auto_dispatch, stop_auto_dispatch,
    start_dashboard, stop_dashboard, assign_tasks_optimal,
)
from tools.natural_language import (                      # noqa: F401
    list_locations, add_location, remove_location,
    go_to_location, send_nearest_to,
)
from tools.chat import robot_chat                         # noqa: F401


def _log(msg: str) -> None:
    # stderr keeps stdio-transport JSON-RPC clean.
    print(f"[robo_fleet] {msg}", file=sys.stderr)


def main() -> None:
    parser = argparse.ArgumentParser(description="Robo_fleet MCP server")
    parser.add_argument(
        "--transport",
        choices=["stdio", "http", "sse"],
        default=os.environ.get("ROBOFLEET_TRANSPORT", "stdio"),
        help="Which MCP transport to serve",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("ROBOFLEET_HOST", "0.0.0.0"),
        help="Bind host for http/sse (default 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("ROBOFLEET_PORT", "8766")),
        help="Bind port for http/sse (default 8766)",
    )
    args = parser.parse_args()

    if args.transport == "stdio":
        _log("starting on stdio")
        mcp.run(transport="stdio")
        return

    # http / sse: configure the FastMCP settings before launching.
    mcp.settings.host = args.host
    mcp.settings.port = args.port

    # Loosen the built-in DNS-rebinding filter so LAN clients can reach the
    # server. The default only whitelists 127.0.0.1/localhost/[::1], which
    # breaks the moment a peer on another IP tries to connect.
    security = mcp.settings.transport_security
    security.enable_dns_rebinding_protection = False
    security.allowed_hosts = ["*"]
    security.allowed_origins = ["*"]

    # streamable-http -> POST /mcp, SSE -> GET /sse
    if args.transport == "http":
        _log(f"starting streamable-http on {args.host}:{args.port}{mcp.settings.streamable_http_path}")
        mcp.run(transport="streamable-http")
    else:
        _log(f"starting sse on {args.host}:{args.port}{mcp.settings.sse_path}")
        mcp.run(transport="sse")


if __name__ == "__main__":
    main()
