#!/usr/bin/env python3
"""
Test script for RosClient - tests every function independently.
Run: python3 test_ros_client.py
"""

import time
import sys
sys.path.insert(0, '/home/hayder/Desktop/mcp_server')
from ros.ros_client import RosClient

# ─────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────
ROBOT = "tb1"
TARGET_X = 0.75
TARGET_Y = 0.0

# ─────────────────────────────────────────
# TEST 1 - Connexion
# ─────────────────────────────────────────
def test_connection():
    print("\n=== TEST 1 : Connexion ===")
    client = RosClient()
    client.connect()
    print("✅ Connexion OK")
    client.disconnect()
    print("✅ Deconnexion OK")

# ─────────────────────────────────────────
# TEST 2 - Subscribe (lire position)
# ─────────────────────────────────────────
def test_subscribe():
    print("\n=== TEST 2 : Subscribe amcl_pose ===")
    client = RosClient()
    client.connect()

    msg = client.subscribe_once(
        topic=f"/{ROBOT}/amcl_pose",
        msg_type="geometry_msgs/msg/PoseWithCovarianceStamped",
        timeout=5.0
    )

    if msg:
        pos = msg["pose"]["pose"]["position"]
        print(f"✅ Position {ROBOT} : x={pos['x']:.2f}, y={pos['y']:.2f}")
    else:
        print(f"❌ Pas de message recu - Nav2 est lance ?")

    client.disconnect()

# ─────────────────────────────────────────
# TEST 3 - Publish (cmd_vel direct)
# ─────────────────────────────────────────
def test_publish():
    print("\n=== TEST 3 : Publish cmd_vel (avance 2 secondes) ===")
    client = RosClient()
    client.connect()

    print(f"  -> Envoi commande avancer a {ROBOT}...")
    for _ in range(20):
        client.publish(
            topic=f"/{ROBOT}/cmd_vel",
            msg_type="geometry_msgs/msg/TwistStamped",
            data={
                "header": {"frame_id": "base_link"},
                "twist": {
                    "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                }
            }
        )
        time.sleep(0.1)

    # Stop
    client.publish(
        topic=f"/{ROBOT}/cmd_vel",
        msg_type="geometry_msgs/msg/TwistStamped",
        data={
            "header": {"frame_id": "base_link"},
            "twist": {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
        }
    )
    print(f"✅ Robot arrete")
    client.disconnect()

# ─────────────────────────────────────────
# TEST 4 - Send goal navigate_to_pose
# ─────────────────────────────────────────
def test_navigation_complete():
    print("\n=== TEST : Send goal + Wait result ===")
    client = RosClient()
    client.connect()

    goal = {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": TARGET_X, "y": TARGET_Y, "z": 0.0},
                "orientation": {"x": 0.75, "y": 0.75, "z": 0.0, "w": 1.0}
            }
        }
    }

    action = f"/{ROBOT}/navigate_to_pose"
    print(f"  -> Envoi goal: x={TARGET_X}, y={TARGET_Y}")
    resp = client.send_goal(
        action=action,
        action_type="nav2_msgs/action/NavigateToPose",
        goal=goal
    )
    goal_id = resp["goal_id"]
    print(f"  -> goal_id = {goal_id}")

    result = client.wait_for_result(action, goal_id, timeout=30.0)
    print(f"  -> Resultat: {result}")

    client.disconnect()

# ─────────────────────────────────────────
# TEST 5 - Cancel action
# ─────────────────────────────────────────
def test_cancel():
    print("\n=== TEST 5 : Cancel action ===")
    client = RosClient()
    client.connect()

    action = f"/{ROBOT}/navigate_to_pose"
    goal = {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": 2.0, "y": 2.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        }
    }

    resp = client.send_goal(action, "nav2_msgs/action/NavigateToPose", goal)
    goal_id = resp["goal_id"]
    print(f"  -> goal envoye: {goal_id}")

    time.sleep(2)
    cancel_resp = client.cancel_action(action, goal_id)
    print(f"  -> cancel: {cancel_resp}")

    client.disconnect()


if __name__ == "__main__":
    test_connection()
    test_subscribe()
    test_publish()
    test_navigation_complete()
    test_cancel()
