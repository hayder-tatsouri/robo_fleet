# ROS 2 Outdoor Simulation Workspace (Docker on Amazon Linux 2)

This machine (Amazon Linux 2, no GPU) is not officially supported for ROS 2 or
Gazebo. Everything runs inside a **Docker container** based on the official
`osrf/ros:jazzy-desktop-full` image, extended with Gazebo Harmonic, Nav2, and
robot_localization.

## What works here
- Full ROS 2 Jazzy CLI, `colcon` builds, launch files, bridges, unit tests.
- Gazebo Harmonic in **headless mode** (`gz sim -s`) — physics + sensors + topics.
- ROS \<-> Gazebo bridging via `ros_gz_bridge`.
- Writing/validating SDF worlds, URDFs, Nav2 configs.

## What does NOT work here
- Gazebo GUI (no display, no GPU). Software rendering (`llvmpipe`) is present
  but far too slow for outdoor worlds with heightmaps/cameras/LiDAR.
- RViz2 GUI, mapviz GUI.
- Anything requiring OpenGL 3.3+ hardware acceleration.

For the full visual experience, use this workspace unchanged on an
Ubuntu 24.04 machine with a GPU.

## Quick start

```bash
# 1. Build the extended image (one-time, ~15 min)
docker build -t outdoor-sim:jazzy .

# 2. Open a shell in the container
./scripts/ros2-shell.sh
# or, once the extended image is built:
docker run --rm -it --network host -v "$PWD":/workspace -w /workspace \
    outdoor-sim:jazzy bash

# Inside the container:
gz sim --version
ros2 pkg list | grep ros_gz
gz sim -s -r worlds/outdoor_empty.sdf   # -s = server only (headless)
```

## Layout
```
worlds/    Gazebo SDF world files
models/    Robot URDF/SDF and mesh assets
launch/    ROS 2 launch files
config/    EKF, Nav2, bridge YAML
maps/      Occupancy grids, aerial imagery
scripts/   Docker helper scripts
src/       ROS 2 packages (colcon workspace)
```
