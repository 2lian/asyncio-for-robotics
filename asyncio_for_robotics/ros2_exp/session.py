from __future__ import annotations

import contextvars
import uuid
from contextlib import asynccontextmanager
from typing import AsyncGenerator, TypeVar

import rclpy
from rclpy.experimental import AsyncNode

_MISSING = object()
_T = TypeVar("_T")
_CURRENT_SESSION: contextvars.ContextVar[AsyncNode | None] = contextvars.ContextVar(
    "afor_ros2_exp_current_session",
    default=None,
)


def current_session(default: _T = _MISSING) -> AsyncNode | _T:
    session = _CURRENT_SESSION.get()
    if session is None:
        if default is _MISSING:
            raise RuntimeError("No active experimental ROS session")
        return default
    return session


def auto_session(session: AsyncNode | None = None) -> AsyncNode:
    if session is not None:
        return session
    return current_session()


@asynccontextmanager
async def async_context(
    node: None | str | AsyncNode = None,
    *,
    auto_run: bool = True,
) -> AsyncGenerator[AsyncNode, None]:
    current = current_session(None)
    if current is not None:
        yield current
        return

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
