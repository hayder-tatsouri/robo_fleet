"""Creates ReAct agents (LLM + tools) for each domain using LangGraph prebuilt."""

import os
from langchain_core.tools import tool
from langchain_anthropic import ChatAnthropic
from langgraph.prebuilt import create_react_agent

# Import existing MCP tool functions (the source of truth)
from tools.navigation import navigate_to_pose
from tools.waypoints import navigate_waypoints
from tools.monitoring import get_robot_position, get_fleet_status, get_battery_level
from tools.obstacles import check_obstacles
from tools.advanced import (
    predict_collisions, start_dashboard, stop_dashboard, assign_tasks_optimal,
)
from tools.coordination import (
    assign_tasks, dispatch_tasks, get_plan, replan,
    set_robot_priority, configure_fleet,
)

# Wrap as langchain @tool so create_react_agent can expose them to the LLM
nav_tools = [tool(navigate_to_pose), tool(navigate_waypoints)]
monitor_tools = [tool(get_robot_position), tool(get_fleet_status), tool(get_battery_level)]
collision_tools = [tool(check_obstacles), tool(predict_collisions)]
planning_tools = [tool(assign_tasks), tool(dispatch_tasks), tool(get_plan), tool(replan),
                  tool(set_robot_priority), tool(configure_fleet), tool(assign_tasks_optimal)]
dashboard_tools = [tool(start_dashboard), tool(stop_dashboard)]


_GLOBAL_INSTRUCTION = """
IMPORTANT: Only act on the most recent user message. Previous commands that were already completed should be ignored. Do not re-execute requests from earlier in the conversation history."""


def _load_skill_prompt(name: str, fallback: str) -> str:
    path = os.path.join(os.path.dirname(__file__), f"{name}.md")
    prompt = ""
    if os.path.exists(path):
        with open(path) as f:
            prompt = f.read().strip()
    else:
        prompt = fallback
    return prompt + _GLOBAL_INSTRUCTION


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
COLLISION_PROMPT = _load_skill_prompt("collision", "")
PLANNING_PROMPT = _load_skill_prompt("planning", "")
DASHBOARD_PROMPT = _load_skill_prompt("dashboard", "")


# ─── Create all ReAct agents (each has its own LLM + tools + prompt) ───

navigation_agent = create_react_agent(_llm(), tools=nav_tools, prompt=NAVIGATION_PROMPT, name="navigation_agent")
monitoring_agent = create_react_agent(_llm(), tools=monitor_tools, prompt=MONITORING_PROMPT, name="monitoring_agent")
collision_agent = create_react_agent(_llm(), tools=collision_tools, prompt=COLLISION_PROMPT, name="collision_agent")
planning_agent = create_react_agent(_llm(), tools=planning_tools, prompt=PLANNING_PROMPT, name="planning_agent")
dashboard_agent = create_react_agent(_llm(), tools=dashboard_tools, prompt=DASHBOARD_PROMPT, name="dashboard_agent")

AGENT_REGISTRY = {
    "navigation_agent": navigation_agent,
    "monitoring_agent": monitoring_agent,
    "collision_agent": collision_agent,
    "planning_agent": planning_agent,
    "dashboard_agent": dashboard_agent,
}

ALL_AGENT_NAMES = list(AGENT_REGISTRY.keys())
