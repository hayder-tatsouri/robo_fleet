import json
import time
import uuid
import websocket


class RosClient:
    def __init__(self, host="localhost", port=9090):
        self.url = f"ws://{host}:{port}"
        self.ws = None

    def connect(self):
        self.ws = websocket.create_connection(self.url)
        print(f"Connected to rosbridge at {self.url} ✅")

    def disconnect(self):
        if self.ws:
            self.ws.close()
            self.ws = None

    # ─────────────────────────────────────────
    # PUBLISH — envoie un message sur un topic
    # ─────────────────────────────────────────
    def publish(self, topic, msg_type, data):
        msg = {
            "op": "publish",
            "topic": topic,
            "type": msg_type,
            "msg": data
        }
        self.ws.send(json.dumps(msg))

    # ─────────────────────────────────────────
    # SUBSCRIBE — lit un message d'un topic
    # ─────────────────────────────────────────
    def subscribe_once(self, topic, msg_type, timeout=5.0):
        sub_id = f"sub_{uuid.uuid4().hex[:8]}"
        
        # Souscrit
        self.ws.send(json.dumps({
            "op": "subscribe",
            "id": sub_id,
            "topic": topic,
            "type": msg_type
        }))

        # Attend le premier message
        self.ws.settimeout(timeout)
        try:
            while True:
                response = self.ws.recv()
                data = json.loads(response)
                if data.get("op") == "publish" and data.get("topic") == topic:
                    # Désouscrit
                    self.ws.send(json.dumps({
                        "op": "unsubscribe",
                        "id": sub_id,
                        "topic": topic
                    }))
                    return data.get("msg")
        except websocket.WebSocketTimeoutException:
            return None

    # ─────────────────────────────────────────
    # ACTION — envoie un goal (non‑bloquant)
    # ─────────────────────────────────────────
    def send_goal(self, action, action_type, goal):
        goal_id = f"goal_{uuid.uuid4().hex[:8]}"
        self.ws.send(json.dumps({
            "op": "send_action_goal",
            "id": goal_id,
            "action": action,
            "action_type": action_type,
            "args": goal,
            "feedback": True
        }))
        return {"goal_id": goal_id}

    # ─────────────────────────────────────────
    # ACTION — attend le résultat d'un goal
    # ─────────────────────────────────────────
    def wait_for_result(self, action, goal_id, timeout=30.0):
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
    # CANCEL — annule un goal en cours
    # ─────────────────────────────────────────
    def cancel_action(self, action, goal_id):
        self.ws.send(json.dumps({
            "op": "cancel_action_goal",
            "id": goal_id,
            "action": action
        }))
        return {"success": True, "goal_id": goal_id}