# server.py
from mcp.server.fastmcp import FastMCP

# Crée le serveur MCP
mcp = FastMCP(
    name="robots_mcp",
    json_response=True
)