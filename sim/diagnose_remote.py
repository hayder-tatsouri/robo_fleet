#!/usr/bin/env python3
"""
Robo_Fleet - Remote Rosbridge Diagnostic
─────────────────────────────────────────
Connects to a real rosbridge and discovers what topics/services exist.
Prints raw messages to help debug format differences.

Usage:
  python sim/diagnose_remote.py --host 192.168.0.8
"""

import json
import time
import argparse
import sys
sys.path.insert(0, 'mcp_server')

import websocket


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.0.8")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()

    url = f"ws://{args.host}:{args.port}"
    print(f"\n{'=' * 60}")
    print(f"  Robo_Fleet - Remote Rosbridge Diagnostic")
    print(f"  Target: {url}")
    print(f"{'=' * 60}")

    ws = websocket.create_connection(url, timeout=5)
    print(f"\n  ✅ Connected\n")

    # ─── 1. List all topics ───
    print("  --- Discovering Topics ---")
    ws.send(json.dumps({
        "op": "call_service",
        "service": "/rosapi/topics",
        "type": "rosapi/Topics",
    }))

    try:
        ws.settimeout(3)
        resp = ws.recv()
        data = json.loads(resp)
        topics = data.get("values", {}).get("topics", [])
        types = data.get("values", {}).get("types", [])

        if topics:
            print(f"\n  Found {len(topics)} topics:\n")
            for topic, ttype in zip(topics, types):
                # Highlight robot-related topics
                if any(r in topic for r in ["/tb", "/cmd_vel", "/amcl", "/nav", "/battery", "/scan", "/odom"]):
                    print(f"  ★ {topic:<50} {ttype}")
                else:
                    print(f"    {topic:<50} {ttype}")
        else:
            print("  No topics found via /rosapi/topics")
            print("  (rosapi might not be installed - trying manual subscribe)")
    except websocket.WebSocketTimeoutException:
        print("  /rosapi/topics not available - trying manual discovery")
        topics = []

    # ─── 2. Try subscribing to known robot topics ───
    print("\n\n  --- Checking Robot Topics ---")

    test_topics = [
        ("/tb1/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"),
        ("/tb1/odom", "nav_msgs/msg/Odometry"),
        ("/tb1/battery_state", "sensor_msgs/msg/BatteryState"),
        ("/tb1/battery", "sensor_msgs/msg/BatteryState"),
        ("/tb1/scan", "sensor_msgs/msg/LaserScan"),
        ("/tb2/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"),
        ("/tb3/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"),
        # Alternative namespaces
        ("/robot1/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"),
        ("/turtlebot1/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"),
    ]

    for topic, msg_type in test_topics:
        ws.send(json.dumps({
            "op": "subscribe",
            "id": f"diag_{topic}",
            "topic": topic,
            "type": msg_type,
        }))

    print("  Subscribed to test topics. Listening 5 seconds...\n")

    received = {}
    start = time.time()
    while time.time() - start < 5:
        try:
            ws.settimeout(1)
            raw = ws.recv()
            data = json.loads(raw)
            if data.get("op") == "publish":
                topic = data.get("topic", "unknown")
                if topic not in received:
                    received[topic] = data
                    # Print first message from each topic
                    msg = data.get("msg", {})
                    print(f"  ✅ {topic}")
                    # Print abbreviated content
                    msg_str = json.dumps(msg, indent=2)
                    if len(msg_str) > 300:
                        print(f"     {msg_str[:300]}...")
                    else:
                        print(f"     {msg_str}")
                    print()
        except websocket.WebSocketTimeoutException:
            continue

    if not received:
        print("  ⚠️  No messages received on any test topic")
        print("     Robots may use different namespaces")

    # ─── 3. Test navigation action ───
    print("\n  --- Testing Navigation Action ---")
    print("  Sending goal to /tb1/navigate_to_pose...\n")

    ws.send(json.dumps({
        "op": "send_action_goal",
        "id": "diag_goal_001",
        "action": "/tb1/navigate_to_pose",
        "action_type": "nav2_msgs/action/NavigateToPose",
        "args": {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": 0.5, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        },
        "feedback": True,
    }))

    print("  Listening for action responses (10s)...\n")
    action_msgs = []
    start = time.time()
    while time.time() - start < 10:
        try:
            ws.settimeout(1)
            raw = ws.recv()
            data = json.loads(raw)
            op = data.get("op", "")
            if op in ("action_result", "action_feedback", "service_response"):
                action_msgs.append(data)
                print(f"  [{op}] {json.dumps(data, indent=2)[:500]}")
                print()
                if op == "action_result":
                    break
        except websocket.WebSocketTimeoutException:
            continue

    if not action_msgs:
        print("  ⚠️  No action response received")
        print("     Nav2 might not be running or action name differs")

    # ─── Summary ───
    print(f"\n{'=' * 60}")
    print(f"  SUMMARY")
    print(f"{'=' * 60}")
    print(f"  Topics receiving data: {list(received.keys()) or 'NONE'}")
    print(f"  Action responses: {len(action_msgs)}")
    print(f"\n  Use this info to configure Robo_Fleet for your setup.")
    print(f"{'=' * 60}")

    ws.close()


if __name__ == "__main__":
    main()
