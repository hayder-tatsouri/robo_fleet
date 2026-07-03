#!/bin/bash
# ═══════════════════════════════════════════════════════════
# Robo_Fleet - Setup (macOS + Linux)
# ═══════════════════════════════════════════════════════════
# Just Python 3.10+ needed. No ROS2, Docker, or conda required.

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
echo "✅ Setup complete!"
echo ""
echo "═══════════════════════════════════"
echo ""
echo "  To run everything:"
echo ""
echo "    source .venv/bin/activate"
echo "    python run.py"
echo ""
echo "  Then in another terminal:"
echo ""
echo "    source .venv/bin/activate"
echo "    python sim/test_integration.py"
echo ""
echo "═══════════════════════════════════"
