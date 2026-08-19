import json
import time
import uuid
import threading
import websocket


class RosClient:
    """
    WebSocket client for rosbridge protocol.
    Features: publish, subscribe, action goals, auto-reconnect.
    """

    def __init__(self, host="localhost", port=9090, auto_reconnect=True, max_retries=5):
        self.host = host
        self.port = port
        self.url = f"ws://{host}:{port}"
        self.ws = None
        self.auto_reconnect = auto_reconnect
        self.max_retries = max_retries
        self._connected = False
        self._lock = threading.Lock()

    @property
    def connected(self):
        return self._connected and self.ws is not None

    def connect(self):
        """Connect to rosbridge with retry logic."""
        retries = 0
        while retries < self.max_retries:
            try:
                self.ws = websocket.create_connection(self.url, timeout=15)
                self._connected = True
                # NOTE: Do NOT print to stdout - when this module is loaded by
                # the FastMCP server running over stdio, stray stdout writes
                # corrupt the JSON-RPC wire protocol. Log to stderr instead.
                import sys
                print(f"Connected to rosbridge at {self.url}", file=sys.stderr)
                return True
            except Exception as e:
                retries += 1
                if retries < self.max_retries:
                    wait = min(2 ** retries, 10)
                    print(f"  Connection attempt {retries}/{self.max_retries} failed, retrying in {wait}s...")
                    time.sleep(wait)
                else:
                    self._connected = False
                    raise ConnectionError(f"Failed to connect after {self.max_retries} attempts: {e}")

    def disconnect(self):
        """Close WebSocket connection."""
        self._connected = False
        if self.ws:
            try:
                self.ws.close()
            except:
                pass
            self.ws = None

    def _ensure_connected(self):
        """Reconnect if disconnected and auto_reconnect is enabled."""
        if not self._connected and self.auto_reconnect:
            self.connect()
        elif not self._connected:
            raise ConnectionError("Not connected to rosbridge")

    def _send(self, msg):
        """Send with auto-reconnect on failure."""
        with self._lock:
            try:
                self.ws.send(json.dumps(msg))
            except Exception:
                if self.auto_reconnect:
                    self.connect()
                    self.ws.send(json.dumps(msg))
                else:
                    raise

    # ─────────────────────────────────────────
    # PUBLISH
    # ─────────────────────────────────────────
    def publish(self, topic, msg_type, data):
        """Publish a message to a ROS topic."""
        self._ensure_connected()
        msg = {
            "op": "publish",
            "topic": topic,
            "type": msg_type,
            "msg": data
        }
        self._send(msg)

    # ─────────────────────────────────────────
    # SUBSCRIBE
    # ─────────────────────────────────────────
    def subscribe_once(self, topic, msg_type, timeout=5.0):
        """Subscribe and wait for a single message."""
        self._ensure_connected()
        sub_id = f"sub_{uuid.uuid4().hex[:8]}"

        self._send({
            "op": "subscribe",
            "id": sub_id,
            "topic": topic,
            "type": msg_type
        })

        self.ws.settimeout(timeout)
        try:
            while True:
                response = self.ws.recv()
                data = json.loads(response)
                if data.get("op") == "publish" and data.get("topic") == topic:
                    self._send({
                        "op": "unsubscribe",
                        "id": sub_id,
                        "topic": topic
                    })
                    return data.get("msg")
        except websocket.WebSocketTimeoutException:
            return None

    # ─────────────────────────────────────────
    # ACTION - send goal
    # ─────────────────────────────────────────
    def send_goal(self, action, action_type, goal):
        """Send an action goal (non-blocking)."""
        self._ensure_connected()
        goal_id = f"goal_{uuid.uuid4().hex[:8]}"
        self._send({
            "op": "send_action_goal",
            "id": goal_id,
            "action": action,
            "action_type": action_type,
            "args": goal,
            "feedback": True
        })
        return {"goal_id": goal_id}

    # ─────────────────────────────────────────
    # ACTION - wait for result
    # ─────────────────────────────────────────
    def wait_for_result(self, action, goal_id, timeout=30.0):
        """Wait for action result with timeout."""
        self.ws.settimeout(timeout)
        start = time.time()
        try:
            while time.time() - start < timeout:
                response = self.ws.recv()
                data = json.loads(response)
                if data.get("op") == "action_feedback":
                    feedback = data.get("values", {})
                    distance = feedback.get("distance_remaining", "?")

                if data.get("op") == "action_result" and data.get("id") == goal_id:
                    values = data.get("values", {})
                    status = data.get("status")
                    if status != 4:
                        return {
                            "success": False,
                            "status": status,
                            "error": values.get("error", "unknown error"),
                            "goal_id": goal_id
                        }
                    return {
                        "success": status == 4,
                        "status": status,
                        "goal_id": goal_id
                    }
        except websocket.WebSocketTimeoutException:
            return {
                "success": False,
                "error": f"timeout after {timeout}s",
                "goal_id": goal_id
            }

    # ─────────────────────────────────────────
    # CANCEL
    # ─────────────────────────────────────────
    def cancel_action(self, action, goal_id):
        """Cancel a running action goal."""
        self._ensure_connected()
        self._send({
            "op": "cancel_action_goal",
            "id": goal_id,
            "action": action
        })
        return {"success": True, "goal_id": goal_id}

    # ─────────────────────────────────────────
    # HEALTH CHECK
    # ─────────────────────────────────────────
    def ping(self):
        """Check if rosbridge connection is alive."""
        try:
            self.ws.ping()
            return True
        except:
            self._connected = False
            return False
