#!/bin/bash
# Start the robo_fleet MCP server over streamable-HTTP inside the pguard_sim
# container. Any MCP client that speaks HTTP transport can then connect to
# http://<this-host>:8766/mcp .
#
# Idempotent: kills any previous server on 8766 first.
#
# Usage:
#   ./serve_http.sh           # binds 0.0.0.0:8766
#   ./serve_http.sh 8767      # custom port
#   HOST=127.0.0.1 ./serve_http.sh   # LAN vs loopback

PORT=${1:-8766}
HOST=${HOST:-0.0.0.0}

# Kill any previous instance.
docker exec pguard_sim pkill -f 'index.py --transport http' 2>/dev/null || true
sleep 1

# Start the new one detached. Using `docker exec -d` (not `nohup &` inside
# `bash -c`) so the child survives after the CLI returns.
docker exec -d pguard_sim bash -c "
    cd /workspace/robo_fleet/mcp_server && \
    exec python3 -u index.py --transport http --host ${HOST} --port ${PORT} \
      > /tmp/mcp_http.log 2>&1
"

# Wait briefly for it to bind.
for i in 1 2 3 4 5; do
    if docker exec pguard_sim bash -c "exec 3<>/dev/tcp/127.0.0.1/${PORT}" 2>/dev/null; then
        break
    fi
    sleep 0.5
done

echo "MCP HTTP server started:"
docker exec pguard_sim grep -E 'starting|Uvicorn running' /tmp/mcp_http.log 2>/dev/null | tail -3
echo
echo "Reachable endpoints:"
echo "  From this host:    http://localhost:${PORT}/mcp"
CONTAINER_IP=$(docker inspect pguard_sim --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}')
LAN_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "  Container IP:      http://${CONTAINER_IP}:${PORT}/mcp"
[ -n "$LAN_IP" ] && echo "  LAN (published):   http://${LAN_IP}:${PORT}/mcp"
