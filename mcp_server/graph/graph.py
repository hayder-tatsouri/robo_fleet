import json
import os
from langgraph.graph import StateGraph, END
from langchain_core.messages import HumanMessage, SystemMessage, AIMessage

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


MAX_HOPS = 6  # hard cap on supervisor visits per request — safety valve against routing loops


# ─── Intent-to-direct-tool mapping (fast path: no LLM) ───

def _pick(d, keys):
    return {k: d[k] for k in keys if k in d}

INTENT_TOOLS = {
    "navigate_to_pose":      lambda s: navigate_to_pose(**_pick(s, ("robot_id", "x", "y", "theta", "frame_id", "timeout"))),
    "navigate_waypoints":    lambda s: navigate_waypoints(**_pick(s, ("robot_id", "waypoints", "frame_id", "timeout_per_waypoint"))),
    "go_to_location":        lambda s: go_to_location(**_pick(s, ("robot_id", "location_name", "timeout"))),
    "get_robot_position":    lambda s: get_robot_position(**_pick(s, ("robot_id", "timeout"))),
    "get_fleet_status":      lambda s: get_fleet_status(**_pick(s, ("robot_ids", "timeout"))),
    "get_battery_level":     lambda s: get_battery_level(**_pick(s, ("robot_id", "timeout"))),
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


# ─── Prompt loading helpers ───

def _load_prompt() -> str:
    path = os.path.join(os.path.dirname(__file__), "..", "agents", "supervisor.md")
    if os.path.exists(path):
        with open(path) as f:
            return f.read().strip()
    return "You are a router for a robot fleet system."


def _section(prompt: str, mode: str) -> str:
    """Extract a MODE section from the prompt."""
    parts = prompt.split(f"# MODE: {mode}")
    if len(parts) < 2:
        return prompt
    section = parts[1]
    end = section.find("\n# MODE:")
    if end != -1:
        section = section[:end]
    header = prompt.split("# MODE:")[0]
    return (header + "# MODE: " + mode + section).strip()


# ─── Plan parsing / combining helpers ───

def _parse_plan(text: str) -> list:
    """Parse the Plan-mode LLM output into a validated list of agent names."""
    text = text.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.lower().startswith("json"):
            text = text[4:].strip()

    try:
        parsed = json.loads(text)
        if isinstance(parsed, list):
            return [a.strip().lower() for a in parsed if isinstance(a, str) and a.strip().lower() in ALL_AGENT_NAMES]
        if isinstance(parsed, str):
            name = parsed.strip().lower()
            return [name] if name in ALL_AGENT_NAMES else []
    except (json.JSONDecodeError, TypeError):
        pass

    # Fallback: plain comma/newline separated text
    candidates = [c.strip().lower() for c in text.replace("\n", ",").split(",")]
    return [c for c in candidates if c in ALL_AGENT_NAMES]


def _combine_step_results(step_results: list) -> str:
    if not step_results:
        return ""
    parts = []
    for r in step_results:
        resp = r["response"]
        text = resp if isinstance(resp, str) else json.dumps(resp, default=str)
        parts.append(f"[{r['agent']}] {text}")
    return "\n\n".join(parts)


def _rewrite(combined_text: str) -> str:
    if not combined_text:
        return "No result to report."
    try:
        prompt = _load_prompt()
        rewrite_section = _section(prompt, "Rewrite")
        llm = _llm()
        resp = llm.invoke([
            SystemMessage(content=rewrite_section),
            HumanMessage(content=f"Rewrite this combined result into one clear, natural answer for the user:\n\n{combined_text}"),
        ])
        return resp.content.strip()
    except Exception:
        return combined_text


# ─── Supervisor node ───

def supervisor_node(state: AgentState) -> dict:
    plan = state.get("plan")
    hop_count = state.get("hop_count", 0)

    # Safety valve: never let the supervisor loop forever
    if hop_count > MAX_HOPS:
        combined = _combine_step_results(state.get("step_results", []))
        return {
            "next": "__end__",
            "response": combined or "Task stopped: too many steps without completing.",
            "plan": None, "step": 0, "step_results": [],
        }

    # ─── Continuing or finishing an active multi-step plan ───
    if plan:
        step = state.get("step", 0)
        step_results = state.get("step_results", [])
        response_text = state.get("response")

        if response_text is not None:
            # the agent for the current step just finished
            step_results = step_results + [{"agent": plan[step], "response": response_text}]
            step += 1

        if step >= len(plan):
            combined = _combine_step_results(step_results)
            return {
                "next": "__end__",
                "response": _rewrite(combined),
                "plan": None, "step": 0, "step_results": [],
            }

        return {
            "next": plan[step],
            "step": step,
            "step_results": step_results,
            "plan_context": _combine_step_results(step_results),
            "response": None,
            "hop_count": hop_count + 1,
        }

    # ─── Fast path: dashboard-supplied structured intent, no LLM ───
    intent = state.get("intent")
    if intent and intent in INTENT_TOOLS:
        return {"next": INTENT_TO_AGENT[intent]}

    # ─── New free-text message: build a plan ───
    if state.get("messages"):
        try:
            prompt = _load_prompt()
            plan_section = _section(prompt, "Plan")

            user_msg = state["messages"][-1]
            user_text = user_msg.get("content", str(user_msg)) if isinstance(user_msg, dict) else str(user_msg)

            llm = _llm()
            resp = llm.invoke([
                SystemMessage(content=plan_section),
                HumanMessage(content=user_text),
            ])
            agent_list = _parse_plan(resp.content)

            if not agent_list:
                answer = _section(prompt, "Answer")
                direct = llm.invoke([
                    SystemMessage(content=answer),
                    HumanMessage(content=user_text),
                ])
                return {"next": "__end__", "response": direct.content.strip()}

            return {
                "next": agent_list[0],
                "plan": agent_list,
                "step": 0,
                "step_results": [],
                "original_query": user_text,
                "hop_count": 1,
                "response": None,
            }
        except ValueError:
            raise  # missing API key etc. — surface loudly
        except Exception:
            return {"next": "__end__", "response": "Sorry, something went wrong while planning this request."}

    return {"next": "__end__"}


# ─── Agent node wrapper ───

def _make_agent_node(agent_name: str):
    react_agent = AGENT_REGISTRY[agent_name]

    def node_fn(state: AgentState) -> dict:
        intent = state.get("intent")
        if intent and intent in INTENT_TOOLS:
            tool_fn = INTENT_TOOLS[intent]
            return {"response": tool_fn(state)}

        plan = state.get("plan")
        if plan:
            # Multi-step mode: give this agent the original request plus
            # whatever prior steps in the plan have already produced.
            original_query = state.get("original_query", "")
            prior_context = state.get("plan_context", "")
            if prior_context:
                content = (
                    f"Context from earlier steps in this task:\n{prior_context}\n\n"
                    f"Original user request: {original_query}\n\n"
                    f"Complete the next step of this request using the context above. "
                    f"You MUST call a tool to complete it — do not describe what you would do."
                )
            else:
                content = (
                    f"{original_query}\n\n"
                    f"You MUST call a tool to fulfill this request. Do not describe what you would do — execute it."
                )
            msgs = [HumanMessage(content=content)]
        else:
            msgs = state.get("messages", [])
            if msgs:
                formatted = []
                for m in msgs:
                    if isinstance(m, dict):
                        role = m.get("role", "user")
                        content = m.get("content", str(m))
                        formatted.append(AIMessage(content=content) if role == "assistant" else HumanMessage(content=content))
                    else:
                        formatted.append(HumanMessage(content=str(m)))
                msgs = formatted

        result = react_agent.invoke({"messages": msgs})
        final = result.get("messages", [])
        last = final[-1] if final else None
        response_text = ""
        if last:
            if hasattr(last, "content") and last.content:
                response_text = last.content
            elif hasattr(last, "tool_calls") and last.tool_calls:
                response_text = f"Called tool: {last.tool_calls[0]['name']}"
            else:
                response_text = str(last)

        update = {"response": response_text}
        if not plan:
            # Only persist full message history for single-shot mode —
            # plan mode uses step_results/plan_context instead, so each
            # agent gets a clean, scoped view rather than an ever-growing log.
            update["messages"] = final
        return update

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