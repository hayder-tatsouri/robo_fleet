#!/usr/bin/env python3
"""
Test script for RosClient — tests every function independently.
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
# TEST 1 — Connexion
# ─────────────────────────────────────────
def test_connection():
    print("\n=== TEST 1 : Connexion ===")
    client = RosClient()
    client.connect()
    print("✅ Connexion OK")
    client.disconnect()
    print("✅ Déconnexion OK")

# ─────────────────────────────────────────
# TEST 2 — Subscribe (lire position)
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
        print(f"❌ Pas de message reçu — Nav2 est lancé ?")

    client.disconnect()

# ─────────────────────────────────────────
# TEST 3 — Publish (cmd_vel direct)
# ─────────────────────────────────────────
def test_publish():
    print("\n=== TEST 3 : Publish cmd_vel (avance 2 secondes) ===")
    client = RosClient()
    client.connect()

    print(f"  → Envoi commande avancer à {ROBOT}...")
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
    print(f"✅ Robot arrêté")
    client.disconnect()

# ─────────────────────────────────────────
# TEST 4 — Send goal navigate_to_pose
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

    # Send goal
    result = client.send_goal(
        action=f"/{ROBOT}/navigate_to_pose",
        action_type="nav2_msgs/action/NavigateToPose",
        goal=goal
    )
    goal_id = result["goal_id"]
    print(f"  → Goal envoyé : {goal_id}")

    # Wait result — MÊME connexion
    result = client.wait_for_result(
        action=f"/{ROBOT}/navigate_to_pose",
        goal_id=goal_id,
        timeout=60.0
    )
    print(f"Résultat : {result}")

    client.disconnect()
# ─────────────────────────────────────────
# TEST 6 — Send goal puis cancel
# ─────────────────────────────────────────
def test_cancel():
    print("\n=== TEST 6 : Send goal puis Cancel ===")
    client = RosClient()
    client.connect()

    goal = {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": 5.0, "y": 5.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        }
    }

    result = client.send_goal(
        action=f"/{ROBOT}/navigate_to_pose",
        action_type="nav2_msgs/action/NavigateToPose",
        goal=goal
    )
    goal_id = result["goal_id"]
    print(f"  → Goal envoyé : {goal_id}")

    print("  → Attente 3 secondes puis cancel...")
    time.sleep(3)

    cancel_result = client.cancel_action(
        action=f"/{ROBOT}/navigate_to_pose",
        goal_id=goal_id
    )
    print(f"✅ Cancel envoyé : {cancel_result}")
    client.disconnect()

# ─────────────────────────────────────────
# MAIN — lance tous les tests
# ─────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 50)
    print("  RosClient Test Suite")
    print("=" * 50)
    print("Assure toi que Gazebo + Nav2 + Rosbridge tournent")
    print("=" * 50)

    try:
        test_connection()
        test_subscribe()
        test_publish()
        test_navigation_complete()
        test_cancel()

        print("\n" + "=" * 50)
        print("✅ Tous les tests terminés")
        print("=" * 50)

    except Exception as e:
        print(f"\n❌ Erreur : {e}")
        import traceback
        traceback.print_exc()