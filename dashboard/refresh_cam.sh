#!/bin/bash
# Periodically re-capture the Gazebo chase-cam frame so the dashboard shows live PGuard.
set -u
CONTAINER=pguard_sim
OUT_DIR=/home/tastouri/ros2_outdoor_sim/robo_fleet/dashboard
INTERVAL=3   # seconds between captures

echo "camera refresh loop started (every ${INTERVAL}s) -> $OUT_DIR"
while true; do
  # Re-aim chase cam at current pguard pose, then grab frame
  docker exec "$CONTAINER" python3 /tmp/aim_chase_cam.py >/dev/null 2>&1 || true
  sleep 0.4
  docker exec "$CONTAINER" python3 /tmp/grab_gz_frame.py /world_cam/chase /tmp/pguard_chase.png >/dev/null 2>&1 || true
  docker cp "$CONTAINER":/tmp/pguard_chase.png "$OUT_DIR/pguard_chase.png" >/dev/null 2>&1 || true
  docker exec "$CONTAINER" python3 /tmp/grab_gz_frame.py /world_cam/top /tmp/pguard_top.png >/dev/null 2>&1 || true
  docker cp "$CONTAINER":/tmp/pguard_top.png "$OUT_DIR/pguard_top.png" >/dev/null 2>&1 || true
  sleep "$INTERVAL"
done
