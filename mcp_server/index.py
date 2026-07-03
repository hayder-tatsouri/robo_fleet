from server import mcp

# Navigation
from tools.navigation import navigate_to_pose
from tools.waypoints import navigate_waypoints

# Monitoring
from tools.monitoring import get_robot_position, get_fleet_status, get_battery_level

# Control
from tools.control import stop_robot, emergency_stop

# Sensing
from tools.obstacles import check_obstacles

# Visualization
from tools.map_viz import get_map_with_robots

# Coordination (multi-robot)
from tools.coordination import (
    assign_tasks,
    dispatch_tasks,
    get_plan,
    replan,
    set_robot_priority,
    configure_fleet,
)

# Advanced - Collision, Queue, Dashboard, Optimal
from tools.advanced import (
    predict_collisions,
    add_task_to_queue,
    get_queue,
    clear_queue,
    start_auto_dispatch,
    stop_auto_dispatch,
    start_dashboard,
    stop_dashboard,
    assign_tasks_optimal,
)

# Natural Language
from tools.natural_language import (
    list_locations,
    add_location,
    remove_location,
    go_to_location,
    send_nearest_to,
)


def main():
    mcp.run(transport="stdio")

if __name__ == "__main__":
    main()
