import os
import sys
from server import mcp
from graph.graph import graph


@mcp.tool()
def robot_chat(query: str) -> str:
    """
    Multi-agent conversational entry point. Handles complex multi-step fleet
    commands (e.g. 'send the nearest robot to the warehouse, then check obstacles').
    Internally routes through a LangGraph supervisor to specialized agents.

    For simple single commands (e.g. 'get battery of tb1'), use the dedicated
    tools instead (get_battery_level, get_robot_position, etc.)
    """
    result = graph.invoke({
        "messages": [{"role": "user", "content": query}],
    })

    response = result.get("response")
    if response:
        return str(response)

    return "I processed your request but there was no specific result to report."
