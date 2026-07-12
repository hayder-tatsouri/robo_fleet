"""
Fleet Chat Agent - LLM-powered chatbot that controls robots via natural language.

This is a REAL MCP client. It:
  1. Spawns the robo_fleet FastMCP server as a subprocess over stdio
     (via the pguard-fleet wrapper script or a direct path).
  2. Discovers the available tools dynamically via MCP `list_tools`.
  3. Forwards those tools to Claude (Anthropic or Bedrock) as function-calling
     tools.
  4. Dispatches every tool_use block back through MCP `call_tool` -
     no hardcoded FLEET_TOOLS list, no hardcoded topic namespaces,
     no chat_agent-side ROS knowledge whatsoever.

Adding a new @mcp.tool() to the server makes it available to the chat
on the next restart, with zero changes here.

Preserves the constructor signature used by start_dashboard.py:
    FleetChatAgent(fleet_manager, provider, api_key, model,
                   rosbridge_host, rosbridge_port)
"""

from __future__ import annotations

import asyncio
import json
import os
import queue
import threading
from contextlib import AsyncExitStack
from typing import Any

try:
    import anthropic
    HAS_ANTHROPIC = True
except ImportError:
    HAS_ANTHROPIC = False

try:
    import boto3
    HAS_BOTO3 = True
except ImportError:
    HAS_BOTO3 = False

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


# System prompt is built dynamically from the live fleet state so it
# describes whichever robots are actually connected (pguard, tb1, etc.).
SYSTEM_PROMPT_TEMPLATE = """You are the Robo_Fleet AI assistant controlling a live robot fleet through the Model Context Protocol (MCP).

Connected robots: {robots}

You have access to a set of MCP tools published by the robo_fleet server. The tool list is enumerated at runtime - always trust the current tool schema over any prior assumption.

Guidance:
- When the user asks for status, position, or battery, use the corresponding read-only tool.
- When the user says "send X to (a, b)" or "go to <location>", call the appropriate navigation tool.
- When the user says "stop" or "emergency", stop only the affected robots unless they say "all".
- Coordinates are in meters in the map frame. The map origin is the fleet datum (for the PGuard outdoor setup this is Enova HQ in Novation City).
- Be concise. After each action, briefly confirm what happened.
"""


class _MCPBridge:
    """
    Runs a persistent asyncio event loop in a background thread that owns
    the MCP ClientSession. Exposes sync `list_tools()` / `call_tool()`
    wrappers so the rest of the code (and the dashboard's threading model)
    doesn't need to be async-aware.
    """

    def __init__(self, server_command: str, server_args: list[str] | None = None,
                 env: dict[str, str] | None = None):
        self._params = StdioServerParameters(
            command=server_command,
            args=server_args or [],
            env=env,
        )
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._session: ClientSession | None = None
        self._exit_stack: AsyncExitStack | None = None
        self._ready = threading.Event()
        self._startup_error: BaseException | None = None
        self._tools_cache: list[dict] | None = None

    def start(self, timeout: float = 15.0) -> None:
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=timeout):
            raise TimeoutError("MCP server did not become ready in time")
        if self._startup_error:
            raise self._startup_error

    def _run_loop(self) -> None:
        try:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(self._session_lifecycle())
        except BaseException as exc:
            self._startup_error = exc
            self._ready.set()

    async def _session_lifecycle(self) -> None:
        # Keep the session open for the process's lifetime.
        async with AsyncExitStack() as stack:
            self._exit_stack = stack
            read, write = await stack.enter_async_context(stdio_client(self._params))
            session = await stack.enter_async_context(ClientSession(read, write))
            await session.initialize()
            self._session = session
            self._ready.set()

            # Block forever - the event loop stays alive until stop().
            self._shutdown = asyncio.Event()
            await self._shutdown.wait()

    def stop(self) -> None:
        if self._loop and self._loop.is_running():
            async def _trigger():
                if hasattr(self, "_shutdown"):
                    self._shutdown.set()
            asyncio.run_coroutine_threadsafe(_trigger(), self._loop)

    def _run(self, coro):
        assert self._loop is not None, "MCP bridge not started"
        fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return fut.result()

    def list_tools(self, use_cache: bool = True) -> list[dict]:
        if use_cache and self._tools_cache is not None:
            return self._tools_cache
        result = self._run(self._session.list_tools())
        tools: list[dict] = []
        for t in result.tools:
            tools.append({
                "name": t.name,
                "description": (t.description or "").strip(),
                "input_schema": t.inputSchema or {"type": "object", "properties": {}},
            })
        self._tools_cache = tools
        return tools

    def call_tool(self, name: str, arguments: dict) -> str:
        """Call the tool and flatten the result to a string for the LLM."""
        result = self._run(self._session.call_tool(name, arguments=arguments))
        chunks: list[str] = []
        for c in result.content:
            text = getattr(c, "text", None)
            if text:
                chunks.append(text)
            else:
                chunks.append(str(c))
        payload = "\n".join(chunks) if chunks else ""
        if getattr(result, "isError", False):
            return json.dumps({"error": payload or "tool call failed"})
        return payload or "(no output)"


class FleetChatAgent:
    """LLM-powered chat agent - MCP client edition."""

    def __init__(
        self,
        fleet_manager,
        provider: str = "anthropic",
        api_key: str | None = None,
        model: str | None = None,
        rosbridge_host: str = "localhost",
        rosbridge_port: int = 9090,
        mcp_server_command: str | None = None,
        mcp_server_args: list[str] | None = None,
    ):
        self.fleet = fleet_manager
        self.provider = provider
        self.messages: list[dict] = []

        # LLM client setup.
        if provider == "anthropic":
            if not HAS_ANTHROPIC:
                raise ImportError("pip install anthropic")
            self.api_key = api_key or os.environ.get("ANTHROPIC_API_KEY", "")
            if not self.api_key:
                raise ValueError("ANTHROPIC_API_KEY not set")
            self.model = model or "claude-sonnet-4-20250514"
            self.client = anthropic.Anthropic(api_key=self.api_key)
        elif provider == "bedrock":
            if not HAS_BOTO3:
                raise ImportError("pip install boto3")
            self.model = model or "anthropic.claude-sonnet-4-20250514-v1:0"
            self.client = boto3.client(
                "bedrock-runtime",
                region_name=os.environ.get("AWS_REGION", "us-east-1"),
            )
        else:
            raise ValueError(f"Unknown provider: {provider!r}. Use 'anthropic' or 'bedrock'.")

        # MCP server: default to the sibling FastMCP server (mcp_server/index.py).
        # Callers can override by passing mcp_server_command explicitly, which
        # is how the pguard-fleet wrapper is wired in.
        if mcp_server_command is None:
            here = os.path.dirname(os.path.abspath(__file__))
            server_dir = os.path.normpath(os.path.join(here, ".."))  # mcp_server/
            mcp_server_command = "python3"
            mcp_server_args = ["-u", os.path.join(server_dir, "index.py")]
            env = os.environ.copy()
            # Ensure the child process can import `server`, `tools.*`, etc.
            env["PYTHONPATH"] = server_dir + os.pathsep + env.get("PYTHONPATH", "")
            # Pass the rosbridge address through so the MCP tools connect to the
            # right host without any chat_agent-side ROS logic.
            env["ROBOFLEET_ROSBRIDGE_HOST"] = rosbridge_host
            env["ROBOFLEET_ROSBRIDGE_PORT"] = str(rosbridge_port)
        else:
            env = os.environ.copy()

        self.mcp = _MCPBridge(mcp_server_command, mcp_server_args, env=env)
        self.mcp.start()
        self.tools = self.mcp.list_tools()
        self.last_tool_used: str | None = None

    def chat(self, user_message: str) -> str:
        """Send a message, run the tool-use loop, return the final assistant text."""
        self.messages.append({"role": "user", "content": user_message})
        system = self._render_system_prompt()

        response = self._call_llm(system)
        self.last_tool_used = None

        for _ in range(8):  # tool loop safety cap
            tool_calls = self._extract_tool_calls(response)
            if not tool_calls:
                break

            for tc in tool_calls:
                self.last_tool_used = tc["name"]

            self.messages.append({"role": "assistant", "content": response["content"]})
            self.messages.append({
                "role": "user",
                "content": [
                    {
                        "type": "tool_result",
                        "tool_use_id": tc["id"],
                        "content": self.mcp.call_tool(tc["name"], tc["input"] or {}),
                    }
                    for tc in tool_calls
                ],
            })

            response = self._call_llm(system)

        text = self._extract_text(response)
        self.messages.append({"role": "assistant", "content": text})
        return text

    def reset(self) -> None:
        self.messages = []
        self.last_tool_used = None

    def _render_system_prompt(self) -> str:
        # Describe the fleet dynamically - no hardcoded tb1/tb2/tb3.
        parts: list[str] = []
        for rid, robot in self.fleet.robots.items():
            x = getattr(robot, "x", 0.0)
            y = getattr(robot, "y", 0.0)
            parts.append(f"{rid} @ ({x:.2f}, {y:.2f})")
        robot_list = ", ".join(parts) if parts else "(none online yet)"
        return SYSTEM_PROMPT_TEMPLATE.format(robots=robot_list)

    def _call_llm(self, system: str) -> dict:
        if self.provider == "anthropic":
            resp = self.client.messages.create(
                model=self.model,
                max_tokens=1024,
                system=system,
                tools=self.tools,
                messages=self.messages,
            )
            return {"content": resp.content, "stop_reason": resp.stop_reason}

        body = {
            "anthropic_version": "bedrock-2023-05-31",
            "max_tokens": 1024,
            "system": [{"type": "text", "text": system}],
            "tools": self.tools,
            "messages": self.messages,
        }
        raw = self.client.invoke_model(
            modelId=self.model,
            body=json.dumps(body),
            contentType="application/json",
            accept="application/json",
        )
        result = json.loads(raw["body"].read())
        return {"content": result.get("content", []), "stop_reason": result.get("stop_reason")}

    @staticmethod
    def _extract_tool_calls(response: dict) -> list[dict]:
        calls: list[dict] = []
        content = response.get("content", [])
        if isinstance(content, str):
            return calls
        for block in content:
            btype = getattr(block, "type", None) or (isinstance(block, dict) and block.get("type"))
            if btype != "tool_use":
                continue
            if hasattr(block, "id"):
                calls.append({"id": block.id, "name": block.name, "input": block.input})
            else:
                calls.append({"id": block["id"], "name": block["name"], "input": block["input"]})
        return calls

    @staticmethod
    def _extract_text(response: dict) -> str:
        content = response.get("content", [])
        if isinstance(content, str):
            return content
        texts: list[str] = []
        for block in content:
            btype = getattr(block, "type", None) or (isinstance(block, dict) and block.get("type"))
            if btype != "text":
                continue
            texts.append(getattr(block, "text", None) or block["text"])
        return " ".join(texts) if texts else "Done."
