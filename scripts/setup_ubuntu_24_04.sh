#!/usr/bin/env bash
# ============================================================================
#  setup_ubuntu_24_04.sh
#
#  Installs everything needed to run the PGuard outdoor simulation natively
#  on Ubuntu 24.04 (Noble) - no Docker.
#
#  Idempotent: safe to re-run; each step checks whether it's already done.
#
#  What it installs:
#    - System build tools (curl, git, cmake, gnupg, ...)
#    - ROS 2 Jazzy Desktop + Nav2 + robot_localization + rosbridge + foxglove
#    - Gazebo Harmonic + ros_gz bridge + ros_gz_sim
#    - Python deps for the robo_fleet MCP server + dashboard (PIL, websockets, mcp)
#    - Optional: builds the colcon workspace (unless --no-build)
#
#  Usage:
#      # From this repo root:
#      ./scripts/setup_ubuntu_24_04.sh                    # full setup
#      ./scripts/setup_ubuntu_24_04.sh --no-build         # skip colcon build
#      ./scripts/setup_ubuntu_24_04.sh --no-python        # skip pip install
#      ./scripts/setup_ubuntu_24_04.sh --dry-run          # print what it would do
#      ./scripts/setup_ubuntu_24_04.sh --check            # verify install only
#
#  Requires sudo (for apt).
# ============================================================================

set -euo pipefail

# ─── Args ────────────────────────────────────────────────────────────────────
BUILD=1
INSTALL_PYTHON=1
DRY_RUN=0
CHECK_ONLY=0

for arg in "$@"; do
    case "$arg" in
        --no-build)  BUILD=0 ;;
        --no-python) INSTALL_PYTHON=0 ;;
        --dry-run)   DRY_RUN=1 ;;
        --check)     CHECK_ONLY=1 ;;
        -h|--help)
            sed -n '1,30p' "$0"
            exit 0 ;;
        *)
            echo "unknown arg: $arg" >&2
            exit 1 ;;
    esac
done

# ─── Helpers ─────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'
step()  { echo -e "\n${GREEN}==>${NC} $*"; }
warn()  { echo -e "${YELLOW}!!${NC} $*"; }
die()   { echo -e "${RED}✗${NC} $*" >&2; exit 1; }
run()   {
    echo "  $*"
    if [ "$DRY_RUN" = 0 ]; then eval "$*"; fi
}

# ─── Sanity checks ───────────────────────────────────────────────────────────
step "Sanity checks"

if [ "$(id -u)" -eq 0 ]; then
    warn "Running as root. Prefer running as your normal user; the script will call sudo when it needs to."
fi

if ! command -v lsb_release >/dev/null 2>&1; then
    if [ "$CHECK_ONLY" = 0 ] && [ "$DRY_RUN" = 0 ]; then
        run "sudo apt update && sudo apt install -y lsb-release"
    fi
fi

UBUNTU_CODENAME="$( (lsb_release -cs 2>/dev/null) || echo noble )"
if [ "$UBUNTU_CODENAME" != "noble" ]; then
    warn "Ubuntu codename is '$UBUNTU_CODENAME', not 'noble'. ROS 2 Jazzy officially targets Ubuntu 24.04 (Noble). Continuing anyway."
fi

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$REPO_ROOT"
echo "  repo root: $REPO_ROOT"

# ─── --check short-circuit ───────────────────────────────────────────────────
if [ "$CHECK_ONLY" = 1 ]; then
    step "Verifying install"
    have() { command -v "$1" >/dev/null 2>&1 && echo -e "  ${GREEN}✓${NC} $1  ($($1 --version 2>&1 | head -1))" || echo -e "  ${RED}✗${NC} $1"; }
    have ros2
    have gz
    have colcon
    have rviz2
    have python3
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/jazzy/setup.bash
        for pkg in nav2_bringup robot_localization rosbridge_server foxglove_bridge ros_gz_bridge ros_gz_sim xacro; do
            if ros2 pkg list 2>/dev/null | grep -qx "$pkg"; then
                echo -e "  ${GREEN}✓${NC} ROS pkg: $pkg"
            else
                echo -e "  ${RED}✗${NC} ROS pkg: $pkg (missing)"
            fi
        done
    else
        echo -e "  ${RED}✗${NC} /opt/ros/jazzy/setup.bash not found"
    fi
    if [ -f install/setup.bash ]; then
        echo -e "  ${GREEN}✓${NC} workspace built (install/setup.bash present)"
    else
        echo -e "  ${YELLOW}!${NC} workspace not yet built (run without --check)"
    fi
    exit 0
fi

# ─── Step 1: base system packages ────────────────────────────────────────────
step "1/6  Base system packages"
run "sudo apt update"
run "sudo apt install -y \
    curl gnupg lsb-release ca-certificates \
    software-properties-common \
    build-essential cmake git python3-pip python3-venv \
    imagemagick"

# ─── Step 2: ROS 2 Jazzy apt repo + packages ─────────────────────────────────
step "2/6  ROS 2 Jazzy apt repo"

if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    run "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
fi

if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    run "echo 'deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main' | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null"
fi

run "sudo apt update"
run "sudo apt install -y \
    ros-jazzy-desktop-full \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    ros-jazzy-xacro \
    ros-jazzy-rosbridge-server \
    ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    run "sudo rosdep init || true"
fi
run "rosdep update"

# ─── Step 3: Gazebo Harmonic apt repo + packages ─────────────────────────────
step "3/6  Gazebo Harmonic apt repo"

if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
    run "sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"
fi

if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
    run "echo 'deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $UBUNTU_CODENAME main' | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null"
fi

run "sudo apt update"
run "sudo apt install -y \
    gz-harmonic \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim"

# ─── Step 4: GDAL (for OSM map generation) ───────────────────────────────────
step "4/6  GDAL (for OSM map generation)"
run "sudo apt install -y gdal-bin python3-gdal"

# ─── Step 5: Python deps for robo_fleet ──────────────────────────────────────
if [ "$INSTALL_PYTHON" = 1 ]; then
    step "5/6  Python deps for robo_fleet"
    if [ -f robo_fleet/requirements.txt ]; then
        # Ubuntu 24.04 requires --break-system-packages for pip to touch the system Python.
        # If you prefer a venv, create one first and drop --break-system-packages.
        run "pip3 install --break-system-packages --user -r robo_fleet/requirements.txt"
    else
        warn "robo_fleet/requirements.txt not found - skipping pip install"
    fi
else
    step "5/6  Python deps skipped (--no-python)"
fi

# ─── Step 6: Auto-source ROS 2 + build the workspace ─────────────────────────
step "6/6  Shell setup + colcon build"

if ! grep -q '/opt/ros/jazzy/setup.bash' "$HOME/.bashrc" 2>/dev/null; then
    run "echo 'source /opt/ros/jazzy/setup.bash' >> \"$HOME/.bashrc\""
fi
if ! grep -q "$REPO_ROOT/install/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
    run "echo '[ -f $REPO_ROOT/install/setup.bash ] && source $REPO_ROOT/install/setup.bash' >> \"$HOME/.bashrc\""
fi

if [ "$BUILD" = 1 ]; then
    if [ "$DRY_RUN" = 0 ]; then
        # shellcheck disable=SC1091
        source /opt/ros/jazzy/setup.bash
    fi
    run "colcon build --symlink-install"
    warn "workspace built. Open a NEW shell (or 'source install/setup.bash') before running ros2 launch."
else
    step "colcon build skipped (--no-build). Run manually:"
    echo "  source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
fi

# ─── Done ────────────────────────────────────────────────────────────────────
cat <<EOF

==============================================================================
  Setup complete.

  Quick sanity check (in a NEW shell):
      ros2 doctor --report | head
      gz sim --version
      ros2 pkg list | grep my_pguard_bot

  Full stack with Gazebo GUI + RViz2 + Nav2 panel:
      cd $REPO_ROOT
      source install/setup.bash
      ros2 launch my_pguard_bot full_stack.launch.py use_gui:=true rviz:=true

  Web dashboard + AI chat (headless-friendly, works with or without GUI):
      cd $REPO_ROOT
      python3 -m http.server 8091 --directory robo_fleet/dashboard &
      python3 robo_fleet/start_dashboard.py --robots pguard --dashboard-port 8090
      # then open http://localhost:8091/live_dashboard.html?ws=ws://localhost:8090

  Verify install any time:
      ./scripts/setup_ubuntu_24_04.sh --check

  Documentation: docs/PROJECT.md
==============================================================================
EOF
