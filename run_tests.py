#!/usr/bin/env python3
"""
run_tests.py -- end-to-end smoke test for the PGuard + robo_fleet stack.

Exercises every layer in order and prints PASS/FAIL for each:

  L1  Container + Docker         - pguard_sim up, required ports published
  L2  ROS 2 core                 - Nav2, EKF, adapter nodes; live /pguard topics
  L3  rosbridge WebSocket        - handshake works, can publish/subscribe
  L4  MCP stdio                  - mcp_pguard.sh -> initialize -> tools/list = 29
  L5  MCP streamable-http        - localhost:8766/mcp -> tools/list + call_tool
  L6  chat_agent (MCP client)    - dynamic prompt + tool_use round-trip
  L7  Adapter round-trip         - /pguard/cmd_vel -> /cmd_vel

Exit code = number of failed layers.
"""
from __future__ import annotations

import asyncio
import json
import os
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field


REPO = "/home/tastouri/ros2_outdoor_sim"
CONTAINER = "pguard_sim"
STDIO_WRAPPER = f"{REPO}/mcp_pguard.sh"
HTTP_URL = "http://localhost:8766/mcp"


# ANSI helpers - work fine in most terminals, harmless if piped.
GRN, RED, YLW, DIM, RST = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"


@dataclass
class Result:
    name: str
    ok: bool
    detail: str = ""
    sub: list["Result"] = field(default_factory=list)


def run(cmd: list[str] | str, timeout: float = 30, shell: bool = False) -> tuple[int, str, str]:
    p = subprocess.run(
        cmd if not shell else cmd,
        shell=shell,
        capture_output=True, text=True, timeout=timeout,
    )
    return p.returncode, p.stdout, p.stderr


def in_container(cmd: str, timeout: float = 30) -> tuple[int, str, str]:
    return run(["docker", "exec", CONTAINER, "bash", "-lc", cmd], timeout=timeout)


# --------------------------------------------------------------------
# L1 - Container + Docker
# --------------------------------------------------------------------
def test_l1() -> Result:
    rc, out, _ = run(["docker", "ps", "--filter", f"name={CONTAINER}",
                      "--format", "{{.Status}}\t{{.Ports}}"])
    if rc != 0 or not out.strip():
        return Result("L1 container", False, "pguard_sim not running")
    status, ports = (out.strip().split("\t") + [""])[:2]
    subs = [Result(f"container status: {status}", status.startswith("Up"))]

    def port_published(port: str, ports_str: str) -> bool:
        """Handle both single-port ('0.0.0.0:8765->8765/tcp') and
        range-port ('0.0.0.0:8765-8766->8765-8766/tcp') Docker formats."""
        p = int(port)
        for chunk in ports_str.split(","):
            # Extract the "HOST:MAPPING" prefix, then the mapping side.
            if "->" not in chunk:
                continue
            host_side = chunk.split("->")[0]
            # host_side looks like '0.0.0.0:8765' or '0.0.0.0:8765-8766' or ':::8765-8766'
            mapping = host_side.rsplit(":", 1)[-1]
            if "-" in mapping:
                lo, hi = mapping.split("-")
                if lo.isdigit() and hi.isdigit() and int(lo) <= p <= int(hi):
                    return True
            elif mapping.isdigit() and int(mapping) == p:
                return True
        return False

    for p in ("8765", "8766", "9090"):
        subs.append(Result(f"port {p} published", port_published(p, ports)))

    # Bind mount check.
    rc, out, _ = run(["docker", "inspect", CONTAINER, "--format",
                      "{{range .Mounts}}{{.Source}}:{{.Destination}} {{end}}"])
    subs.append(Result("workspace bind-mount",
                       f"{REPO}:/workspace" in out,
                       out.strip()))

    ok = all(s.ok for s in subs)
    return Result("L1  Container + Docker", ok, sub=subs)


# --------------------------------------------------------------------
# L2 - ROS 2 core
# --------------------------------------------------------------------
def test_l2() -> Result:
    rc, out, err = in_container(
        "source /opt/ros/jazzy/setup.bash && ros2 node list 2>&1",
        timeout=15,
    )
    nodes = set(out.split())
    required = {
        "/bt_navigator", "/controller_server", "/planner_server",
        "/ekf_local", "/ekf_global", "/navsat_transform",
        "/robo_fleet_adapter", "/map_server", "/rosbridge_websocket",
    }
    subs = [Result(f"node {n}", n in nodes) for n in sorted(required)]

    # Live topics under /pguard/*
    rc2, out2, _ = in_container(
        "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>&1 | grep '^/pguard/'",
        timeout=10,
    )
    topics = set(out2.split())
    for t in ("/pguard/amcl_pose", "/pguard/battery_state",
              "/pguard/cmd_vel", "/pguard/scan"):
        subs.append(Result(f"topic {t}", t in topics))

    # /odometry/filtered actually flowing: try `topic hz`, and if that flakes
    # (short window right after a burst of activity), fall back to
    # `topic echo --once` which is cheaper.
    rc3, out3, err3 = in_container(
        "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic hz /odometry/filtered 2>&1 | head -3",
        timeout=10,
    )
    hz_seen = "average rate:" in out3
    if not hz_seen:
        rc3b, out3b, _ = in_container(
            "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo --once /odometry/filtered 2>&1 | head -20",
            timeout=10,
        )
        hz_seen = "pose:" in out3b or "twist:" in out3b
        out3 = out3b
    subs.append(Result("/odometry/filtered is publishing",
                       hz_seen, out3.strip().replace("\n", " | ")[:200]))

    ok = all(s.ok for s in subs)
    return Result("L2  ROS 2 core", ok, sub=subs)


# --------------------------------------------------------------------
# L3 - rosbridge
# --------------------------------------------------------------------
def test_l3() -> Result:
    subs = []
    # TCP reachability on 9090.
    try:
        with socket.create_connection(("localhost", 9090), timeout=2):
            subs.append(Result("tcp connect localhost:9090", True))
    except OSError as e:
        subs.append(Result("tcp connect localhost:9090", False, str(e)))
        return Result("L3  rosbridge", False, sub=subs)

    # Functional round-trip: subscribe to /pguard/amcl_pose (published by the
    # adapter) and confirm at least one message arrives via rosbridge. This
    # exercises the actual code path chat_agent + dashboard rely on and does
    # not depend on rosapi_node being launched.
    ws_script = r'''
import websocket, json, time
ws = websocket.create_connection("ws://localhost:9090", timeout=5)
ws.send(json.dumps({
    "op": "subscribe", "id": "s1",
    "topic": "/pguard/amcl_pose",
    "type": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "throttle_rate": 100,
}))
deadline = time.time() + 5
got = None
while time.time() < deadline:
    try:
        msg = json.loads(ws.recv())
    except Exception:
        continue
    if msg.get("op") == "publish" and msg.get("topic") == "/pguard/amcl_pose":
        got = msg
        break
ws.send(json.dumps({"op": "unsubscribe", "topic": "/pguard/amcl_pose"}))
ws.close()
if got:
    p = got["msg"]["pose"]["pose"]["position"]
    print(f"POSE={p['x']:.4f},{p['y']:.4f}")
else:
    print("POSE=NONE")
'''
    script_path_host = os.path.join(REPO, ".smoke_rosbridge.py")
    with open(script_path_host, "w") as f:
        f.write(ws_script)
    try:
        rc, out, err = in_container(
            "python3 /workspace/.smoke_rosbridge.py", timeout=15)
    finally:
        try: os.remove(script_path_host)
        except OSError: pass

    pose_line = next((l for l in out.splitlines() if l.startswith("POSE=")), "POSE=NONE")
    ok = pose_line != "POSE=NONE"
    subs.append(Result(f"subscribed to /pguard/amcl_pose via ws ({pose_line})",
                       ok, err.strip()[-200:] if not ok else ""))

    ok = all(s.ok for s in subs)
    return Result("L3  rosbridge", ok, sub=subs)


# --------------------------------------------------------------------
# L4 - MCP stdio
# --------------------------------------------------------------------
def test_l4() -> Result:
    subs = []
    if not os.access(STDIO_WRAPPER, os.X_OK):
        return Result("L4  MCP stdio", False,
                      sub=[Result(f"{STDIO_WRAPPER} executable", False)])

    p = subprocess.Popen(
        [STDIO_WRAPPER],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True,
    )
    try:
        def send(msg): p.stdin.write(json.dumps(msg) + "\n"); p.stdin.flush()
        send({"jsonrpc": "2.0", "id": 1, "method": "initialize",
              "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                         "clientInfo": {"name": "smoke", "version": "0.1"}}})
        send({"jsonrpc": "2.0", "method": "notifications/initialized", "params": {}})
        send({"jsonrpc": "2.0", "id": 2, "method": "tools/list", "params": {}})

        tool_count = None
        server_name = None
        deadline = time.time() + 10
        while time.time() < deadline:
            line = p.stdout.readline()
            if not line: break
            try: msg = json.loads(line)
            except: continue
            if msg.get("id") == 1:
                server_name = msg["result"]["serverInfo"]["name"]
            elif msg.get("id") == 2:
                tool_count = len(msg["result"]["tools"])
                break
    finally:
        p.terminate()
        try: p.wait(timeout=3)
        except: p.kill()

    subs.append(Result(f"initialize -> serverInfo.name = {server_name!r}",
                       server_name == "robots_mcp"))
    subs.append(Result(f"tools/list = {tool_count} (expected 29)",
                       tool_count == 29))
    ok = all(s.ok for s in subs)
    return Result("L4  MCP stdio", ok, sub=subs)


# --------------------------------------------------------------------
# L5 - MCP streamable-http
# --------------------------------------------------------------------
def test_l5() -> Result:
    subs = []
    try:
        with socket.create_connection(("localhost", 8766), timeout=2):
            subs.append(Result("tcp connect localhost:8766", True))
    except OSError as e:
        subs.append(Result("tcp connect localhost:8766", False, str(e)))
        return Result("L5  MCP streamable-http", False, sub=subs)

    try:
        from mcp import ClientSession
        from mcp.client.streamable_http import streamablehttp_client
    except ImportError:
        subs.append(Result("mcp python sdk available", False))
        return Result("L5  MCP streamable-http", False, sub=subs)

    async def probe():
        async with streamablehttp_client(HTTP_URL) as (r, w, get_sid):
            async with ClientSession(r, w) as session:
                await session.initialize()
                tools = await session.list_tools()
                res = await session.call_tool(
                    "get_robot_position", {"robot_id": "pguard"})
                payload = res.content[0].text if res.content else ""
                return len(tools.tools), payload, get_sid()

    try:
        n_tools, payload, sid = asyncio.run(asyncio.wait_for(probe(), timeout=15))
        subs.append(Result(f"list_tools = {n_tools} (expected 29)", n_tools == 29))
        parsed = json.loads(payload)
        subs.append(Result(
            f"call_tool get_robot_position -> success={parsed.get('success')}",
            bool(parsed.get("success")),
            f"pose ({parsed.get('x')}, {parsed.get('y')}, {parsed.get('theta')}), sid={sid[:8]}..."
        ))
    except Exception as e:
        subs.append(Result("MCP http round-trip", False, repr(e)))

    ok = all(s.ok for s in subs)
    return Result("L5  MCP streamable-http", ok, sub=subs)


# --------------------------------------------------------------------
# L6 - chat_agent (MCP-client based)
# --------------------------------------------------------------------
def test_l6() -> Result:
    """Exercises chat_agent from INSIDE the container using a fake LLM,
    proving dynamic tool discovery + tool-use round-trip works."""
    script = r'''
import sys, json
sys.path.insert(0, "/workspace/robo_fleet/mcp_server")
import coordination.chat_agent as m

class FakeRobot:
    robot_id = "pguard"; x = 0.0; y = 0.0
class FakeFleet:
    robots = {"pguard": FakeRobot()}

_calls = [0]
class FakeAnth:
    def __init__(self, **_):
        outer_calls = _calls
        class Msg:
            @staticmethod
            def create(**_kwargs):
                outer_calls[0] += 1
                class Block:
                    def __init__(self, **kw): self.__dict__.update(kw)
                class Resp:
                    def __init__(self, content, stop="end_turn"):
                        self.content = content; self.stop_reason = stop
                if outer_calls[0] == 1:
                    return Resp([Block(type="tool_use", id="c1",
                                       name="get_robot_position",
                                       input={"robot_id": "pguard"})],
                                stop="tool_use")
                return Resp([Block(type="text",
                                   text="pguard is at the origin.")])
        self.messages = Msg()

m.HAS_ANTHROPIC = True
m.anthropic = type("x", (), {"Anthropic": FakeAnth})

agent = m.FleetChatAgent(fleet_manager=FakeFleet(), api_key="sk-dummy")
prompt = agent._render_system_prompt()
reply = agent.chat("where is pguard?")
out = {
    "tools_discovered": len(agent.tools),
    "tool_names_first_5": [t["name"] for t in agent.tools[:5]],
    "system_prompt_mentions_pguard": "pguard" in prompt,
    "system_prompt_has_no_tb": "tb1" not in prompt and "TurtleBot" not in prompt,
    "reply": reply,
    "tool_used": agent.last_tool_used,
    "llm_turns": _calls[0],
}
print("__RESULT__" + json.dumps(out))
agent.mcp.stop()
'''
    # Write the script to the bind-mounted workspace so we can execute it
    # inside the container without any shell quoting horrors.
    script_path_host = os.path.join(REPO, ".smoke_chat_agent.py")
    script_path_in_ct = "/workspace/.smoke_chat_agent.py"
    with open(script_path_host, "w") as f:
        f.write(script)
    try:
        rc, out, err = in_container(
            f"ANTHROPIC_API_KEY=sk-dummy python3 {script_path_in_ct}",
            timeout=30,
        )
    finally:
        try: os.remove(script_path_host)
        except OSError: pass

    subs = []
    marker = "__RESULT__"
    idx = out.find(marker)
    if idx < 0:
        subs.append(Result("chat_agent script ran", False,
                           (err[-400:] or out[-400:]).strip()))
        return Result("L6  chat_agent (MCP client)", False, sub=subs)

    data = json.loads(out[idx + len(marker):].splitlines()[0])
    subs.append(Result(
        f"tools discovered via real MCP: {data['tools_discovered']}",
        data["tools_discovered"] == 29))
    subs.append(Result("dynamic prompt names pguard",
                       data["system_prompt_mentions_pguard"]))
    subs.append(Result("prompt free of TurtleBot legacy",
                       data["system_prompt_has_no_tb"]))
    subs.append(Result(
        f"tool_use loop executed (LLM called {data['llm_turns']}x, tool={data['tool_used']})",
        data["llm_turns"] >= 2 and data["tool_used"] == "get_robot_position"))
    subs.append(Result(f"final assistant reply: {data['reply']!r}",
                       "origin" in data["reply"].lower()))
    ok = all(s.ok for s in subs)
    return Result("L6  chat_agent (MCP client)", ok, sub=subs)


# --------------------------------------------------------------------
# L7 - Adapter round-trip: /pguard/cmd_vel -> /cmd_vel
# --------------------------------------------------------------------
def test_l7() -> Result:
    """Publish a TwistStamped on /pguard/cmd_vel and confirm the adapter
    republishes it on /cmd_vel."""
    subs = []
    script = r'''
source /opt/ros/jazzy/setup.bash

# Capture /cmd_vel to a file for 5 seconds in the background.
timeout 5 ros2 topic echo /cmd_vel > /tmp/cmd_vel_echo.txt 2>&1 &
ECHO=$!
sleep 1

# Publish a few times so the echo definitely catches one.
for i in 1 2 3; do
  ros2 topic pub -1 /pguard/cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {frame_id: base_link}, twist: {linear: {x: 0.42}, angular: {z: 0.11}}}" \
    >/dev/null 2>&1
  sleep 0.4
done

wait $ECHO 2>/dev/null || true
echo "--- captured on /cmd_vel ---"
cat /tmp/cmd_vel_echo.txt
'''
    rc, out, err = in_container(script, timeout=20)
    lin = "0.42" in out
    ang = "0.11" in out
    tail = out.strip().replace("\n", " | ")[-200:]
    subs.append(Result("adapter forwarded linear.x = 0.42", lin, tail))
    subs.append(Result("adapter forwarded angular.z = 0.11", ang))
    ok = all(s.ok for s in subs)
    return Result("L7  Adapter round-trip", ok, sub=subs)


# --------------------------------------------------------------------
# Runner
# --------------------------------------------------------------------
def print_result(r: Result, indent: int = 0) -> None:
    pad = "  " * indent
    tag = f"{GRN}PASS{RST}" if r.ok else f"{RED}FAIL{RST}"
    line = f"{pad}[{tag}] {r.name}"
    if r.detail:
        line += f"    {DIM}{r.detail}{RST}"
    print(line)
    for s in r.sub:
        print_result(s, indent + 1)


def main() -> int:
    tests = [test_l1, test_l2, test_l3, test_l4, test_l5, test_l6, test_l7]
    print(f"{YLW}=== PGuard + robo_fleet full-stack smoke test ==={RST}\n")
    results = []
    for t in tests:
        name = t.__doc__.strip().splitlines()[0] if t.__doc__ else t.__name__
        try:
            r = t()
        except Exception as e:
            r = Result(t.__name__, False, f"raised {type(e).__name__}: {e}")
        results.append(r)
        print_result(r)
        print()

    passed = sum(1 for r in results if r.ok)
    failed = len(results) - passed
    print(f"{YLW}=== summary ==={RST}  {GRN}{passed} passed{RST}, "
          f"{RED if failed else DIM}{failed} failed{RST}")
    return failed


if __name__ == "__main__":
    sys.exit(main())
