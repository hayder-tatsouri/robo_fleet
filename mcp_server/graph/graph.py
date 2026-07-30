import os
from langgraph.graph import StateGraph, END
from langchain_core.messages import HumanMessage, SystemMessage

from graph.state import AgentState
from agents.react import AGENT_REGISTRY, ALL_AGENT_NAMES, _llm

# Import existing MCP tool functions directly (no .invoke() needed for fast path)
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


# ─── Intent-to-direct-tool mapping (fast path: no LLM) ───

def _pick(d, keys):
    return {k: d[k] for k in keys if k in d}

INTENT_TOOLS = {
    "navigate_to_pose":      lambda s: navigate_to_pose(**_pick(s, ("robot_id", "x", "y", "theta", "frame_id", "timeout"))),
    "navigate_waypoints":    lambda s: navigate_waypoints(**_pick(s, ("robot_id", "waypoints", "frame_id", "timeout_per_waypoint"))),
    "go_to_location":        lambda s: go_to_location(**_pick(s, ("robot_id", "location_name", "timeout"))),
    "get_robot_position":    lambda s: get_robot_position(**_pick(s, ("robot_id", "timeout"))),
    "get_fleet_status":     lambda s: get_fleet_status(**_pick(s, ("robot_ids", "timeout"))),
    "get_battery_level":    lambda s: get_battery_level(**_pick(s, ("robot_id", "timeout"))),
    "stop_robot":            lambda s: stop_robot(**_pick(s, ("robot_id",))),
    "emergency_stop":        lambda s: emergency_stop(**_pick(s, ("robot_ids",))),
    "check_obstacles":       lambda s: check_obstacles(**_pick(s, ("robot_id", "distance_threshold", "timeout"))),
    "predict_collisions":    lambda s: predict_collisions(**_pick(s, ("buffer_distance", "time_horizon"))),
    "assign_tasks":          lambda s: assign_tasks(**_pick(s, ("tasks", "collision_buffer"))),
    "dispatch_tasks":        lambda s: dispatch_tasks(**_pick(s, ("tasks", "collision_buffer", "timeout_per_task"))),
    "get_plan":              lambda s: get_plan(),
    "replan":                lambda s: replan(),
    "set_robot_priority":    lambda s: set_robot_priority(**_pick(s, ("robot_id", "priority"))),
    "configure_fleet":       lambda s: configure_fleet(**_pick(s, ("robot_ids", "groups", "collision_buffer"))),
    "assign_tasks_optimal":  lambda s: assign_tasks_optimal(**_pick(s, ("tasks",))),
    "add_task_to_queue":     lambda s: add_task_to_queue(**_pick(s, ("x", "y", "theta", "priority", "group"))),
    "get_queue":             lambda s: get_queue(),
    "clear_queue":           lambda s: clear_queue(),
    "start_auto_dispatch":   lambda s: start_auto_dispatch(),
    "stop_auto_dispatch":    lambda s: stop_auto_dispatch(),
    "start_dashboard":       lambda s: start_dashboard(**_pick(s, ("port",))),
    "stop_dashboard":        lambda s: stop_dashboard(),
    "list_locations":        lambda s: list_locations(),
    "add_location":          lambda s: add_location(**_pick(s, ("name", "x", "y", "description"))),
    "remove_location":       lambda s: remove_location(**_pick(s, ("name",))),
    "send_nearest_to":       lambda s: send_nearest_to(**_pick(s, ("location_name", "group", "timeout"))),
    "get_map_with_robots":   lambda s: get_map_with_robots(**_pick(s, ("robot_ids", "map_width", "map_height", "timeout"))),
}

INTENT_TO_AGENT = {
    "navigate_to_pose": "navigation_agent",
    "navigate_waypoints": "navigation_agent",
    "go_to_location": "navigation_agent",
    "get_robot_position": "monitoring_agent",
    "get_fleet_status": "monitoring_agent",
    "get_battery_level": "monitoring_agent",
    "stop_robot": "control_agent",
    "emergency_stop": "control_agent",
    "check_obstacles": "collision_agent",
    "predict_collisions": "collision_agent",
    "assign_tasks": "planning_agent",
    "dispatch_tasks": "planning_agent",
    "get_plan": "planning_agent",
    "replan": "planning_agent",
    "assign_tasks_optimal": "planning_agent",
    "set_robot_priority": "planning_agent",
    "configure_fleet": "planning_agent",
    "add_task_to_queue": "queue_agent",
    "get_queue": "queue_agent",
    "clear_queue": "queue_agent",
    "start_auto_dispatch": "queue_agent",
    "stop_auto_dispatch": "queue_agent",
    "start_dashboard": "dashboard_agent",
    "stop_dashboard": "dashboard_agent",
    "list_locations": "natural_lang_agent",
    "add_location": "natural_lang_agent",
    "remove_location": "natural_lang_agent",
    "send_nearest_to": "natural_lang_agent",
    "get_map_with_robots": "map_viz_agent",
}


# ─── Supervisor node ───

def supervisor_node(state: AgentState) -> dict:
    if state.get("response"):
        return {"next": "__end__"}

    intent = state.get("intent")
    if intent and intent in INTENT_TOOLS:
        return {"next": INTENT_TO_AGENT[intent]}

    if state.get("messages"):
        try:
            sys_prompt_path = os.path.join(os.path.dirname(__file__), "..", "agents", "supervisor.md")
            if os.path.exists(sys_prompt_path):
                with open(sys_prompt_path) as f:
                    system_prompt = f.read().strip()
            else:
                system_prompt = "You are a router for a robot fleet system. Respond with the agent name or __end__."

            user_msg = state["messages"][-1]
            if isinstance(user_msg, dict):
                user_text = user_msg.get("content", str(user_msg))
            else:
                user_text = str(user_msg)

            llm = _llm()
            resp = llm.invoke([
                SystemMessage(content=system_prompt),
                HumanMessage(content=user_text),
            ])
            agent = resp.content.strip().lower()
            if agent not in ALL_AGENT_NAMES:
                agent = "__end__"
            return {"next": agent}
        except ValueError as e:
            # Missing API key or configuration error — re-raise so user sees it
            raise
        except Exception:
            return {"next": "navigation_agent"}

    return {"next": "__end__"}


# ─── Agent node wrapper ───

def _make_agent_node(agent_name: str):
    react_agent = AGENT_REGISTRY[agent_name]

    def node_fn(state: AgentState) -> dict:
        intent = state.get("intent")
        if intent and intent in INTENT_TOOLS:
            tool_fn = INTENT_TOOLS[intent]
            return {"response": tool_fn(state)}

        msgs = state.get("messages", [])
        if msgs and isinstance(msgs[-1], dict):
            msgs = [
                HumanMessage(content=m["content"]) if isinstance(m, dict) and m.get("role") == "user"
                else HumanMessage(content=str(m))
                for m in msgs
            ]

        result = react_agent.invoke({"messages": msgs})
        final = result.get("messages", [])
        last = final[-1] if final else None
        response_text = last.content if last and hasattr(last, "content") else str(last) if last else ""
        return {"messages": final, "response": response_text}

    return node_fn


# ─── Conditional edge router ───

def router_condition(state: AgentState) -> str:
    return state.get("next", "__end__")


# ─── Build and compile the graph ───

def build_graph() -> StateGraph:
    builder = StateGraph(AgentState)

    builder.add_node("supervisor", supervisor_node)
    for name in ALL_AGENT_NAMES:
        builder.add_node(name, _make_agent_node(name))

    builder.set_entry_point("supervisor")
    builder.add_conditional_edges("supervisor", router_condition, {
        **{a: a for a in ALL_AGENT_NAMES},
        "__end__": END,
    })

    for agent in ALL_AGENT_NAMES:
        builder.add_edge(agent, "supervisor")

    return builder.compile()


graph = build_graph()
