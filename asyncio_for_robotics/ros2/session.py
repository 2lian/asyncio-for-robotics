import contextvars
import warnings
from contextlib import contextmanager
from typing import Any, Generator, Optional, TypeVar

from rclpy.node import Node

from .session_types import BaseSession, SynchronousSession, ThreadedSession

_CURRENT_SESSION: contextvars.ContextVar[BaseSession | None] = contextvars.ContextVar(
    "afor_ros2_current_session",
    default=None,
)

def current_session() -> BaseSession:
    """Return the current lexical ROS session.

    Args:
        default:
            Value returned when no lexical session context is active.
            If omitted, a RuntimeError is raised instead.
    """
    session = _CURRENT_SESSION.get()
    if session is None:
        raise RuntimeError("No active ROS session context")
    return session


@contextmanager
def auto_context(
    node: None | str | Node | BaseSession = None,
) -> Generator[BaseSession, Any, Any]:
    """Async context manager inside which the default afor.ros2 session (Node) is set.

    Passing nothing or a string will create the node and initialize/shutdown rclpy.
    If a session is passed, it will be started but not closed when exiting the context.

    Args:
        node: Name of the node the create, or an existing node to use in a new
            ThreadedSession, or an existing afor.ros2.session

    Yields:
        The node
    """
    if isinstance(node, BaseSession):
        session = node
        close_on_exit = False
    else:
        session = ThreadedSession(node=node)
        close_on_exit = True
    session.start()
    token = _CURRENT_SESSION.set(session)
    try:
        yield session
    finally:
        _CURRENT_SESSION.reset(token)
        if close_on_exit:
            session.close()


@contextmanager
def session_context(
    session: BaseSession,
    close_on_exit: bool = True,
) -> Generator[BaseSession, Any, Any]:
    """Deprecated compatibility wrapper for binding an explicit session."""
    warnings.warn(
        "session_context() is deprecated; use auto_context(session) instead.",
        DeprecationWarning,
        stacklevel=3,
    )
    try:
        with auto_context(session) as active_session:
            yield active_session
    finally:
        if close_on_exit:
            session.close()


def auto_session(session: Optional[BaseSession] = None) -> BaseSession:
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
            "An `afor.ros2.session` was never declared. A global one is now instanciated. Prefere entering a context using `with auto_context(node=...)`"
        )
        session = ThreadedSession()
        session.start()
        _CURRENT_SESSION.set(session)
        return session
