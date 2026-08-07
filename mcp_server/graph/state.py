from typing import Any, Literal, Optional
from typing_extensions import TypedDict


class StepResult(TypedDict):
    agent: str
    response: Any  # matches `response`'s type below — dict, str, or whatever a tool/agent returns


class AgentState(TypedDict):
    messages: list[dict]
    intent: Optional[str]
    next: Literal[
        "supervisor",
        "navigation_agent",
        "monitoring_agent",
        "control_agent",
        "collision_agent",
        "planning_agent",
        "queue_agent",
        "dashboard_agent",
        "natural_lang_agent",
        "map_viz_agent",
        "__end__",
    ]
    response: Optional[dict]

    # Fast-path parameters (set by direct MCP tools) — unchanged
    robot_id: Optional[str]
    robot_ids: Optional[list[str]]
    x: Optional[float]
    y: Optional[float]
    theta: Optional[float]
    waypoints: Optional[list[dict]]
    location_name: Optional[str]
    timeout: Optional[float]
    frame_id: Optional[str]
    distance_threshold: Optional[float]
    buffer_distance: Optional[float]
    time_horizon: Optional[float]
    tasks: Optional[list[dict]]
    collision_buffer: Optional[float]
    timeout_per_task: Optional[float]
    priority: Optional[int]
    groups: Optional[dict]
    port: Optional[int]
    name: Optional[str]
    description: Optional[str]
    map_width: Optional[float]
    map_height: Optional[float]

    # Multi-step plan fields (new)
    plan: Optional[list[str]]              # ordered agent names to execute
    step: Optional[int]                    # index of the currently-executing step
    step_results: Optional[list[StepResult]]  # accumulated results from completed steps
    plan_context: Optional[str]            # formatted prior results, given to the next agent
    original_query: Optional[str]          # user's original message, kept stable across steps
    hop_count: Optional[int]               # safety counter, caps total supervisor visits