from typing import Any, Literal, Optional
from typing_extensions import TypedDict


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

    # Fast-path parameters (set by direct MCP tools)
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
