#!/bin/bash
# ═══════════════════════════════════════════════════════
# Robo_Fleet - Setup (macOS + Linux)
# ═══════════════════════════════════════════════════════
# Sets up the Python venv + deps for the MCP server and dashboard.
# The ROS 2 / Gazebo sim itself is provided by the Docker container
# (see README_outdoor_sim.md / SETUP.md) or the native installer
# (scripts/setup_ubuntu_24_04.sh).

set -e

echo "🤖 Robo_Fleet Setup"
echo "═══════════════════════════════════"

cd "$(dirname "$0")"

# Create venv
echo "📦 Creating virtual environment..."
python3 -m venv .venv
source .venv/bin/activate

# Install deps
echo "📥 Installing dependencies..."
pip install --upgrade pip -q
pip install -r requirements.txt -q

echo ""
echo "✅ Python environment ready!"
echo ""
echo "═══════════════════════════════════"
echo ""
echo "  1. Start the ROS 2 stack (Docker or native, see SETUP.md):"
echo ""
echo "     ros2 launch my_pguard_bot full_stack.launch.py"
echo "     ros2 launch my_pguard_bot robofleet.launch.py"
echo ""
echo "  2. Then run the dashboard + MCP layer:"
echo ""
echo "     source .venv/bin/activate"
echo "     python start_dashboard.py --rosbridge localhost --robots pearlguard1 pearlguard2 --open"
echo ""
echo "  3. Run the unit tests (no sim required):"
echo ""
echo "     source .venv/bin/activate"
echo "     pytest tests/"
echo ""
echo "═══════════════════════════════════"
