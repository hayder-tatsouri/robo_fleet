#!/bin/bash
# Wrapper: launches the robo_fleet MCP server inside pguard_sim via stdio.
# Cursor sees a normal stdio MCP server; docker forwards stdin/stdout transparently.
#
# The server source lives inside this workspace at ./robo_fleet/, which is
# bind-mounted into the container at /workspace/robo_fleet - so edits made
# from the host are picked up on the next Cursor MCP reconnect. No docker cp,
# no image rebuild.
#
# Registered in ~/.cursor/mcp.json as:
#   "pguard-fleet": { "command": "/home/tastouri/ros2_outdoor_sim/mcp_pguard.sh" }

exec docker exec -i pguard_sim bash -c '
  cd /workspace/robo_fleet/mcp_server && \
  PYTHONUNBUFFERED=1 python3 -u index.py
'
