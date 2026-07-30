"""Creates ReAct agents (LLM + tools) for each domain using LangGraph prebuilt."""

import os
from langchain_core.tools import tool
from langchain_anthropic import ChatAnthropic
from langgraph.prebuilt import create_react_agent

# Import existing MCP tool functions (the source of truth)
from tools.navigation import navigate_to_pose
from tools.waypoints import navigate_waypoints
from tools.monitoring import get_robot_position, get_fleet_status, get_battery_level
from tools.control import stop_robot, emergency_stop
from tools.obstacles import check_obstacles
from tools.advanced import (
    predict_collisions, add_task_to_queue, get_queue, clear_queue,
    start_auto_dispatch, stop_auto_dispatch,
    start_dashboard, stop_dashboard, assign_tasks_optimal,
)
from tools.coordination import (
    assign_tasks, dispatch_tasks, get_plan, replan,
    set_robot_priority, configure_fleet,
)
from tools.natural_language import (
    list_locations, add_location, remove_location, go_to_location, send_nearest_to,
)
from tools.map_viz import get_map_with_robots

# Wrap as langchain @tool so create_react_agent can expose them to the LLM
nav_tools = [tool(navigate_to_pose), tool(navigate_waypoints)]
monitor_tools = [tool(get_robot_position), tool(get_fleet_status), tool(get_battery_level)]
control_tools = [tool(stop_robot), tool(emergency_stop)]
collision_tools = [tool(check_obstacles), tool(predict_collisions)]
planning_tools = [tool(assign_tasks), tool(dispatch_tasks), tool(get_plan), tool(replan),
                  tool(set_robot_priority), tool(configure_fleet), tool(assign_tasks_optimal)]
queue_tools = [tool(add_task_to_queue), tool(get_queue), tool(clear_queue),
               tool(start_auto_dispatch), tool(stop_auto_dispatch)]
dashboard_tools = [tool(start_dashboard), tool(stop_dashboard)]
nl_tools = [tool(list_locations), tool(add_location), tool(remove_location), tool(go_to_location), tool(send_nearest_to)]
map_tools = [tool(get_map_with_robots)]


def _load_skill_prompt(name: str, fallback: str) -> str:
    path = os.path.join(os.path.dirname(__file__), f"{name}.md")
    if os.path.exists(path):
        with open(path) as f:
            return f.read().strip()
    return fallback


def _llm() -> ChatAnthropic:
    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        raise ValueError(
            "ANTHROPIC_API_KEY not set. "
            "Set it in your environment or .env file."
        )
    return ChatAnthropic(
        model="claude-haiku-4-5-20251001",
        api_key=api_key,
        temperature=0.1,
        max_tokens=1024,
    )


# ─── System prompts (loaded from skill .md files) ───

NAVIGATION_PROMPT = _load_skill_prompt("navigation", "")
MONITORING_PROMPT = _load_skill_prompt("monitoring", "")
CONTROL_PROMPT = _load_skill_prompt("control", "")
COLLISION_PROMPT = _load_skill_prompt("collision", "")
PLANNING_PROMPT = _load_skill_prompt("planning", "")
QUEUE_PROMPT = _load_skill_prompt("queue", "")
DASHBOARD_PROMPT = _load_skill_prompt("dashboard", "")
NATURAL_LANG_PROMPT = _load_skill_prompt("natural_lang", "")
MAP_VIZ_PROMPT = _load_skill_prompt("map_viz", "")


# ─── Create all ReAct agents (each has its own LLM + tools + prompt) ───

navigation_agent = create_react_agent(_llm(), tools=nav_tools, prompt=NAVIGATION_PROMPT, name="navigation_agent")
monitoring_agent = create_react_agent(_llm(), tools=monitor_tools, prompt=MONITORING_PROMPT, name="monitoring_agent")
control_agent = create_react_agent(_llm(), tools=control_tools, prompt=CONTROL_PROMPT, name="control_agent")
collision_agent = create_react_agent(_llm(), tools=collision_tools, prompt=COLLISION_PROMPT, name="collision_agent")
planning_agent = create_react_agent(_llm(), tools=planning_tools, prompt=PLANNING_PROMPT, name="planning_agent")
queue_agent = create_react_agent(_llm(), tools=queue_tools, prompt=QUEUE_PROMPT, name="queue_agent")
dashboard_agent = create_react_agent(_llm(), tools=dashboard_tools, prompt=DASHBOARD_PROMPT, name="dashboard_agent")
natural_lang_agent = create_react_agent(_llm(), tools=nl_tools, prompt=NATURAL_LANG_PROMPT, name="natural_lang_agent")
map_viz_agent = create_react_agent(_llm(), tools=map_tools, prompt=MAP_VIZ_PROMPT, name="map_viz_agent")

AGENT_REGISTRY = {
    "navigation_agent": navigation_agent,
    "monitoring_agent": monitoring_agent,
    "control_agent": control_agent,
    "collision_agent": collision_agent,
    "planning_agent": planning_agent,
    "queue_agent": queue_agent,
    "dashboard_agent": dashboard_agent,
    "natural_lang_agent": natural_lang_agent,
    "map_viz_agent": map_viz_agent,
}

ALL_AGENT_NAMES = list(AGENT_REGISTRY.keys())
