import contextvars
from contextlib import contextmanager
from typing import Any, Generator, Optional, TypeVar

from rclpy.node import Node

from .session_types import BaseSession, SynchronousSession, ThreadedSession

_MISSING = object()
_T = TypeVar("_T")
_CURRENT_SESSION: contextvars.ContextVar[BaseSession | None] = contextvars.ContextVar(
    "afor_ros2_current_session",
    default=None,
)


#: global share session (singleton)
GLOBAL_SESSION: Optional[BaseSession] = None


def current_session(default: _T = _MISSING) -> BaseSession | _T:
    """Return the current lexical ROS session.

    Args:
        default:
            Value returned when no lexical session context is active.
            If omitted, a RuntimeError is raised instead.
    """
    session = _CURRENT_SESSION.get()
    if session is None:
        if default is _MISSING:
            raise RuntimeError("No active ROS session context")
        return default
    return session


@contextmanager
def session_context(
    session: BaseSession, close_on_exit: bool = True
) -> Generator[BaseSession, Any, Any]:
    """Bind a ROS session lexically for this block and optionally close it on exit.

    Inside this block, ``auto_session()`` resolves to this session unless an
    explicit session is passed directly.
    """
    session.start()
    token = _CURRENT_SESSION.set(session)
    try:
        yield session
    finally:
        _CURRENT_SESSION.reset(token)
        if close_on_exit:
            session.close()


@contextmanager
def auto_context(
    node: None | str | Node = None,
) -> Generator[BaseSession, Any, Any]:
    """Bind `auto_session()` lexically for this block.

    This is a convenience helper for the normal ROS path.

    If a lexical ROS session is already active, this reuses it and does not
    close it on exit. Otherwise, it creates the default ROS session for this
    block and closes it when leaving.

    Args:
        node:
            Optional node object or node name used when creating the default
            session.
    """
    cur = current_session(None)
    if cur is not None:
        yield cur
        return
    session = ThreadedSession(node=node)
    with session_context(session) as active_session:
        yield active_session


def auto_session(session: Optional[BaseSession] = None) -> BaseSession:
    """Return a ROS session using explicit, lexical, then global resolution.

    Resolution order:
    - explicit ``session`` argument
    - current lexical session context
    - global singleton fallback

    If no explicit or lexical session exists, a global shared
    ``ThreadedSession`` with a ``SingleThreadedExecutor`` is created on first
    use and returned.
    """
    global GLOBAL_SESSION
    if session is not None:
        return session
    session = current_session(default=None)
    if session is not None:
        return session
    if GLOBAL_SESSION is None or GLOBAL_SESSION._closed:
        ses = ThreadedSession(node=None)
        GLOBAL_SESSION = ses
    else:
        ses = GLOBAL_SESSION
    ses.start()
    return ses
