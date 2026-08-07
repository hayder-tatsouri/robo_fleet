#!/usr/bin/env python3
"""
Start the Robo_Fleet Live Dashboard.

Usage:
  python start_dashboard.py --rosbridge 192.168.0.8
  python start_dashboard.py --rosbridge localhost

Then open dashboard/live_dashboard.html in your browser.
"""

import sys
import time
import argparse
import webbrowser
import os

sys.path.insert(0, 'mcp_server')

from coordination.fleet_state import FleetStateManager
from coordination.dashboard_server import DashboardServer
from coordination.chat_agent import FleetChatAgent

parser = argparse.ArgumentParser(description="Start Robo_Fleet Live Dashboard")
parser.add_argument("--rosbridge", default="192.168.0.8", help="rosbridge host")
parser.add_argument("--port", type=int, default=9090, help="rosbridge port")
parser.add_argument("--dashboard-port", type=int, default=8080, help="dashboard WS port")
parser.add_argument("--robots", nargs="+", default=["pearlguard1", "pearlguard2"], help="Robot IDs")
parser.add_argument("--open", action="store_true", help="Auto-open browser")
parser.add_argument("--provider", default="anthropic", choices=["anthropic", "bedrock"], help="LLM provider")
parser.add_argument("--api-key", default=None, help="Anthropic API key (or set ANTHROPIC_API_KEY env var)")
parser.add_argument("--model", default=None, help="Model override")
args = parser.parse_args()

print(f"""
╔══════════════════════════════════════════════════════╗
║       🤖 Robo_Fleet - Live Dashboard                 ║
╠══════════════════════════════════════════════════════╣
║  Rosbridge: ws://{args.rosbridge}:{args.port:<27}║
║  Dashboard: ws://localhost:{args.dashboard_port:<26}║
║  Robots:    {', '.join(args.robots):<40}║
╠══════════════════════════════════════════════════════╣
║  Open dashboard/live_dashboard.html in browser       ║
║  Click on map to navigate robots!                    ║
║  Press Ctrl+C to stop                                ║
╚══════════════════════════════════════════════════════╝
""")

# Start FleetStateManager
print("  🌐 Connecting to rosbridge...")
manager = FleetStateManager.get_instance()
manager.start(
    robot_ids=args.robots,
    ws_url=f"ws://{args.rosbridge}:{args.port}"
)
time.sleep(2)

online = sum(1 for r in manager.robots.values() if r.is_online)
print(f"  ✅ Fleet connected: {online}/{len(args.robots)} robots online")

dashboard = DashboardServer(manager, port=args.dashboard_port)
dashboard.rosbridge_url = f"ws://{args.rosbridge}:{args.port}"

# Set up chat agent (LLM)
api_key = args.api_key or os.environ.get("ANTHROPIC_API_KEY", "")
try:
    chat = FleetChatAgent(
        fleet_manager=manager,
        provider=args.provider,
        api_key=api_key,
        model=args.model,
        rosbridge_host=args.rosbridge,
        rosbridge_port=args.port,
    )
    dashboard.chat_agent = chat
    print(f"  🧠 Chat agent ready ({args.provider}, model={chat.model})")
except Exception as e:
    print(f"  ⚠️  Chat agent disabled: {e}")
    print(f"     Set ANTHROPIC_API_KEY or use --provider bedrock")

print(f"  📊 Starting dashboard server on port {args.dashboard_port}...")
result = dashboard.start()
print(f"  ✅ Dashboard ready!")

# Open browser
html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard", "live_dashboard.html")
if args.open:
    webbrowser.open(f"file://{html_path}")
    print(f"  🌐 Opened in browser")
else:
    print(f"\n  Open in browser: file://{html_path}")

print(f"\n  Streaming fleet state to ws://localhost:{args.dashboard_port}")
print(f"  Click on the map to send robots to that position!\n")

# Keep running
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n  👋 Shutting down...")
    dashboard.stop()
    manager.stop()
    FleetStateManager._instance = None
