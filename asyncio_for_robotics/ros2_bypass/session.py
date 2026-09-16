from __future__ import annotations

import asyncio
import contextvars
import uuid
import warnings
from contextlib import asynccontextmanager, contextmanager, suppress
from typing import AsyncGenerator, Generator

import rclpy
from rclpy.node import Node

from .asyncio_executor_jazzy import BatchAsyncioExecutor


class Session:
    """A normal ROS node and the asyncio executor which runs it."""

    def __init__(
        self,
        node: Node,
        executor: BatchAsyncioExecutor | None = None,
    ) -> None:
        self.node = node
        self.executor = executor or BatchAsyncioExecutor(context=node.context)
        self.executor.add_node(node)

    @contextmanager
    def lock(self) -> Generator[Node, None, None]:
        """Yield the node without locking.

        ``BatchAsyncioExecutor`` safely accepts node operations from the
        asyncio thread, so this method only provides compatibility with the
        regular ROS session API.
        """
        yield self.node


_CURRENT_SESSION: contextvars.ContextVar[Session | None] = contextvars.ContextVar(
    "afor_ros2_bypass_current_session",
    default=None,
)
_FALLBACK_TASKS: set[asyncio.Task[None]] = set()


def current_session() -> Session:
    """Return the session bound to the current lexical context."""
    session = _CURRENT_SESSION.get()
    if session is None:
        raise RuntimeError("No active ROS 2 bypass session")
    return session


def _make_node(node: None | str | Node) -> Node:
    if node is None:
        return Node(f"afor_{uuid.uuid4()}".replace("-", "_"))
    if isinstance(node, str):
        return Node(node)
    if isinstance(node, Node):
        return node
    raise TypeError("node must be a Node, node name, or None")


def _make_session(node: None | str | Node | Session) -> Session:
    if isinstance(node, Session):
        return node
    return Session(_make_node(node))


def _start_executor(session: Session) -> asyncio.Task[None]:
    return asyncio.create_task(
        session.executor.start(),
        name=f"afor.ros2_bypass[{session.node.get_name()}]",
    )


async def _cancel_executor(task: asyncio.Task[None]) -> None:
    task.cancel()
    with suppress(asyncio.CancelledError):
        await task


@asynccontextmanager
async def async_context(
    node: None | str | Node | Session = None,
    *,
    auto_run: bool = True,
) -> AsyncGenerator[Session, None]:
    """Bind a ROS node/executor session and optionally run its executor.

    Passing no value, a node name, or a normal ROS node creates a
    :class:`Session`. An existing session can also be bound directly. If rclpy
    is not already initialized, the context initializes it and shuts it down
    on exit. With ``auto_run=True``, the executor is run and both executor and
    node are stopped and destroyed on exit.

    Args:
        node: An existing session or node, a name for a new node, or ``None``
            for a generated name.
        auto_run: Run the executor and destroy the node with this context.

    Yields:
        The active session, containing its node and executor.
    """
    owns_rclpy = not rclpy.ok()
    if owns_rclpy:
        rclpy.init()

    try:
        active_session = _make_session(node)
        token = _CURRENT_SESSION.set(active_session)
        executor_task: asyncio.Task[None] | None = None
        try:
            if auto_run:
                executor_task = _start_executor(active_session)
                # Give start() a chance to enter its cancellation-safe lifecycle
                # before the context body can immediately exit.
                await asyncio.sleep(0)
                if executor_task.done():
                    executor_task.result()
            yield active_session
        finally:
            try:
                if auto_run:
                    try:
                        if executor_task is not None:
                            await _cancel_executor(executor_task)
                    finally:
                        active_session.node.destroy_node()
            finally:
                _CURRENT_SESSION.reset(token)
    finally:
        if owns_rclpy and rclpy.ok():
            rclpy.shutdown()


def auto_session(session: Session | None = None) -> Session:
    """Return an explicit or lexical session, creating a warned fallback.

    The fallback is intended for compatibility. Prefer ``async_context()`` so
    that the executor, node, and rclpy lifecycle are closed deterministically.
    """
    if session is not None:
        if not isinstance(session, Session):
            raise TypeError("session must be a Session or None")
        return session

    try:
        return current_session()
    except RuntimeError:
        warnings.warn(
            "An `afor.ros2_bypass` session was never declared. A fallback "
            "node and executor are now running. Prefer entering a context "
            "using `async with async_context(node=...)`.",
            stacklevel=2,
        )

    # Fail before allocating ROS resources when called outside asyncio.
    asyncio.get_running_loop()
    if not rclpy.ok():
        rclpy.init()

    session = _make_session(None)
    _CURRENT_SESSION.set(session)
    executor_task = _start_executor(session)
    _FALLBACK_TASKS.add(executor_task)
    executor_task.add_done_callback(_FALLBACK_TASKS.discard)
    return session
