from __future__ import annotations

import asyncio
import contextvars
import uuid
import warnings
from contextlib import asynccontextmanager
from typing import AsyncGenerator, TypeVar

import rclpy
from rclpy.experimental import AsyncNode

_CURRENT_SESSION: contextvars.ContextVar[AsyncNode | None] = contextvars.ContextVar(
    "afor_ros2_exp_current_session",
    default=None,
)


def current_session() -> AsyncNode:
    """Retreives the current session (ros' AsyncNode)."""
    session = _CURRENT_SESSION.get()
    if session is None:
        raise RuntimeError("No active experimental ROS session")
    return session


@asynccontextmanager
async def async_context(
    node: None | str | AsyncNode = None,
    *,
    auto_run: bool = True,
) -> AsyncGenerator[AsyncNode, None]:
    """Async context manager inside which the default afor.ros2_exp session
    (AsyncNode) is set.

    Passing nothing or a string will create the node and initialize/shutdown
    rclpy. Passing a AsyncNode will (potentially) run/cancel it with this
    context, and set it as default.

    Args:
        node: Name of the node the create, or the existing node to use as default.
        auto_run: if the node should be run and destroyed with this context.

    Yields:
        The node
    """
    owns_rclpy = not rclpy.ok()
    if owns_rclpy:
        rclpy.init()

    try:
        if node is None:
            node = AsyncNode(f"afor_{uuid.uuid4()}".replace("-", "_"))
        elif isinstance(node, str):
            node = AsyncNode(node)
        elif not isinstance(node, AsyncNode):
            raise TypeError("node must be an AsyncNode, node name, or None")

        token = _CURRENT_SESSION.set(node)
        try:
            if auto_run:
                async with node:
                    yield node
            else:
                yield node
        finally:
            _CURRENT_SESSION.reset(token)
    finally:
        if owns_rclpy and rclpy.ok():
            rclpy.shutdown()


def auto_session(session: AsyncNode | None = None) -> AsyncNode:
    """Uses the provided session or get the current default or creates a session.

    Resolution order:
    - explicit ``session`` argument
    - current lexical session context
    - creates a lexical session context
    """
    if session is not None:
        return session
    try:
        return current_session()
    except RuntimeError:
        warnings.warn(
            "An `afor.ros2_exp.session` was never declared. A global one is now instanciated using asyncio.create_task. Prefere entering a context using `async with async_context(node=...)`"
        )
        owns_rclpy = not rclpy.ok()
        if owns_rclpy:
            rclpy.init()
        node = AsyncNode(f"afor_{uuid.uuid4()}".replace("-", "_"))
        asyncio.create_task(node.run(), name="global afor.ros2_exp.session")
        return node
