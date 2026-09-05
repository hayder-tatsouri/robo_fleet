"""
FleetStateManager - Persistent connection with in-memory state cache.
Subscribes to all robot topics once, maintains real-time fleet state.
MCP tools query this cache (O(1) lookup) instead of subscribe_once per request.
"""

import json
import math
import time
import threading
import websocket


class RobotState:
    """In-memory state for a single robot."""

    __slots__ = [
        "robot_id", "x", "y", "theta", "battery", "voltage",
        "status", "group", "priority", "last_seen",
        "goal_x", "goal_y", "goal_id", "scan_closest",
    ]

    def __init__(self, robot_id, group="default"):
        self.robot_id = robot_id
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.battery = 100.0
        self.voltage = 12.6
        self.status = "idle"  # idle, navigating, waiting, error, offline
        self.group = group
        self.priority = 0  # higher = more important
        self.last_seen = 0.0
        self.goal_x = None
        self.goal_y = None
        self.goal_id = None
        self.scan_closest = None  # closest obstacle distance

    @property
    def position(self):
        return (self.x, self.y)

    @property
    def is_available(self):
        return self.status == "idle" and self.battery > 10.0

    @property
    def is_online(self):
        return (time.time() - self.last_seen) < 5.0

    def distance_to(self, x, y):
        dx = self.x - x
        dy = self.y - y
        return math.sqrt(dx * dx + dy * dy)

    def to_dict(self):
        return {
            "robot_id": self.robot_id,
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "theta": round(self.theta, 4),
            "battery": round(self.battery, 1),
            "voltage": round(self.voltage, 2),
            "status": self.status,
            "group": self.group,
            "priority": self.priority,
            "online": self.is_online,
            "available": self.is_available,
            "goal": {"x": self.goal_x, "y": self.goal_y} if self.goal_x is not None else None,
            "scan_closest": self.scan_closest,
        }


class FleetStateManager:
    """
    Singleton that maintains a persistent WebSocket connection to rosbridge
    and keeps all robot states in-memory for O(1) queries.

    Usage:
        manager = FleetStateManager.get_instance()
        manager.start(robot_ids=["pearlguard1", "pearlguard2"])
        
        # Instant queries (no network round-trip):
        pos = manager.get_position("pearlguard1")
        fleet = manager.get_all_states()
        nearest = manager.get_nearest_available(x=2.0, y=3.0)
    """

    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    @classmethod
    def get_manager(cls, robot_ids=None, groups=None, ws_url="ws://localhost:9090"):
        """Get (or create) the singleton AND ensure the persistent rosbridge connection is running.

        This is the single "connection step" every tool path should go through so that
        the FleetStateManager actually subscribes to the fleet topics. Calling get_instance()
        alone only allocates the object — robot states stay empty and every robot reads as
        "offline" until start() runs.
        """
        manager = cls.get_instance()
        if not manager._running:
            manager.start(robot_ids=robot_ids, groups=groups, ws_url=ws_url)
        return manager

    def __init__(self):
        self.robots = {}  # robot_id -> RobotState
        self.groups = {}  # group_name -> [robot_ids]
        self.ws = None
        self.ws_url = "ws://localhost:9090"
        self._running = False
        self._recv_thread = None
        self._reconnect_thread = None
        self._connect_lock = threading.Lock()

    def _close_ws(self):
        """Close any active rosbridge websocket without leaving a stale socket behind."""
        ws = self.ws
        self.ws = None
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass

    def start(self, robot_ids=None, groups=None, ws_url="ws://localhost:9090"):
        """
        Initialize the manager with robot IDs and start background listener.
        
        Args:
            robot_ids: List of robot namespaces. Default: ["pearlguard1", "pearlguard2"]
            groups: Dict of group_name -> [robot_ids]. Optional.
            ws_url: rosbridge WebSocket URL.
        """
        if self._running:
            return

        if robot_ids is None:
            robot_ids = ["pearlguard1", "pearlguard2"]

        self.ws_url = ws_url

        # Initialize robot states
        for robot_id in robot_ids:
            self.robots[robot_id] = RobotState(robot_id)

        # Set up groups
        if groups:
            for group_name, members in groups.items():
                self.groups[group_name] = members
                for robot_id in members:
                    if robot_id in self.robots:
                        self.robots[robot_id].group = group_name
        else:
            self.groups["default"] = robot_ids

        # Connect and subscribe
        self._running = True
        self._connect_and_subscribe()

        # Start background receiver thread
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        # Start health monitor
        self._reconnect_thread = threading.Thread(target=self._health_monitor, daemon=True)
        self._reconnect_thread.start()

    def stop(self):
        """Stop the manager and close connection."""
        self._running = False
        self._close_ws()

    def _connect_and_subscribe(self):
        """Connect to rosbridge and subscribe to all robot topics."""
        with self._connect_lock:
            self._close_ws()
            try:
                self.ws = websocket.create_connection(self.ws_url, timeout=15)

                # Subscribe to all topics for all robots
                for robot_id in self.robots:
                    topics = [
                        (f"/{robot_id}/odometry/filtered", "nav_msgs/msg/Odometry"),
                        (f"/{robot_id}/scan", "sensor_msgs/msg/LaserScan"),
                    ]
                    for topic, msg_type in topics:
                        try:
                            self.ws.send(json.dumps({
                                "op": "subscribe",
                                "topic": topic,
                                "type": msg_type,
                                "throttle_rate": 2,
                                "queue_length": 1,
                            }))
                        except Exception:
                            self._close_ws()
                            raise

            except Exception as e:
                self._close_ws()
                print(f"  FleetStateManager: connection failed ({e})")

    def _recv_loop(self):
        """Background thread: receive messages and update state cache."""
        while self._running:
            if self.ws is None:
                time.sleep(1)
                continue
            try:
                self.ws.settimeout(2.0)
                raw = self.ws.recv()
                data = json.loads(raw)
                self._process_message(data)
            except websocket.WebSocketTimeoutException:
                continue
            except Exception:
                if self._running:
                    self._close_ws()
                    time.sleep(1)
                    self._connect_and_subscribe()

    def _health_monitor(self):
        """Mark robots as offline if not seen recently."""
        while self._running:
            time.sleep(2)
            now = time.time()
            for robot in self.robots.values():
                if now - robot.last_seen > 5.0 and robot.status != "offline":
                    robot.status = "offline"

    def _process_message(self, data):
        """Update robot state from incoming rosbridge message."""
        if data.get("op") != "publish":
            return

        topic = data.get("topic", "")
        msg = data.get("msg", {})

        # Determine which robot this is for
        for robot_id, robot in self.robots.items():
            if f"/{robot_id}/" in topic:
                robot.last_seen = time.time()

                if "odometry/filtered" in topic:
                    pose = msg.get("pose", {}).get("pose", {})
                    pos = pose.get("position", {})
                    orient = pose.get("orientation", {})
                    robot.x = pos.get("x", 0.0)
                    robot.y = pos.get("y", 0.0)
                    z = orient.get("z", 0.0)
                    w = orient.get("w", 1.0)
                    robot.theta = 2 * math.atan2(z, w)
                    if robot.status == "offline":
                        robot.status = "idle"

                elif "battery_state" in topic:
                    robot.battery = msg.get("percentage", 0) * 100
                    robot.voltage = msg.get("voltage", 0)

                elif "scan" in topic:
                    ranges = msg.get("ranges", [])
                    range_min = msg.get("range_min", 0.12)
                    range_max = msg.get("range_max", 10.0)
                    valid = [r for r in ranges if range_min <= r <= range_max]
                    robot.scan_closest = min(valid) if valid else None

                break

    # ───────────────────────────────────────
    # QUERY METHODS (O(1) or O(N) - no network)
    # ───────────────────────────────────────

    def get_position(self, robot_id):
        """Get robot position (O(1))."""
        robot = self.robots.get(robot_id)
        if robot:
            return {"x": robot.x, "y": robot.y, "theta": robot.theta, "online": robot.is_online}
        return None

    def get_state(self, robot_id):
        """Get full robot state (O(1))."""
        robot = self.robots.get(robot_id)
        return robot.to_dict() if robot else None

    def get_all_states(self):
        """Get all robot states (O(N))."""
        return [r.to_dict() for r in self.robots.values()]

    def get_available_robots(self):
        """Get robots that are idle and have sufficient battery (O(N))."""
        return [r for r in self.robots.values() if r.is_available]

    def get_nearest_available(self, x, y, group=None):
        """Find the nearest available robot to a position (O(N))."""
        candidates = self.get_available_robots()
        if group:
            candidates = [r for r in candidates if r.group == group]
        if not candidates:
            return None
        return min(candidates, key=lambda r: r.distance_to(x, y))

    def get_robots_in_group(self, group_name):
        """Get all robots in a group (O(N))."""
        return [r for r in self.robots.values() if r.group == group_name]

    def set_robot_status(self, robot_id, status, goal_x=None, goal_y=None, goal_id=None):
        """Update robot status (called by task planner)."""
        robot = self.robots.get(robot_id)
        if robot:
            robot.status = status
            robot.goal_x = goal_x
            robot.goal_y = goal_y
            robot.goal_id = goal_id

    def set_priority(self, robot_id, priority):
        """Set robot priority for collision resolution."""
        robot = self.robots.get(robot_id)
        if robot:
            robot.priority = priority
