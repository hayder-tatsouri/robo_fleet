#!/usr/bin/env python3
"""Launch the robo_fleet dashboard against the live PGuard sim."""
import os
import sys
import time

sys.path.insert(0, "/root/robo_fleet/mcp_server")

from coordination.fleet_state import FleetStateManager
from coordination.dashboard_server import DashboardServer

# The MCP server import chain requires this env for chat_agent even if not used
os.environ.setdefault("ANTHROPIC_API_KEY", "")

def main():
    print("=" * 60)
    print("  Robo_Fleet Dashboard -> PGuard Sim")
    print("=" * 60)

    manager = FleetStateManager()
    manager.start(robot_ids=["pguard"])
    print("FleetStateManager started with robots: ['pguard']")
    time.sleep(2)

    dash = DashboardServer(manager, host="0.0.0.0", port=8080, update_rate=5)
    result = dash.start()
    print(f"Dashboard: {result}")

    print()
    print("Dashboard is streaming.")
    print("  Live state:  ws://localhost:8080")
    print("  Rosbridge:   ws://localhost:9090")
    print()
    print("Open dashboard HTML on the host at:")
    print("  http://localhost:8080/dashboard.html  (if you serve the HTML)")
    print("or copy dashboard.html locally and it will connect over WS.")
    print()

    print("Ctrl-C to quit.")
    try:
        while True:
            for rid in ["pguard"]:
                pos = manager.get_position(rid)
                st = manager.get_state(rid)
                batt = st.get("battery_percentage", "?") if st else "?"
                print(f"  [{rid}] pos=({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f})  batt={batt}")
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        dash.stop()
        manager.stop()

if __name__ == "__main__":
    main()
