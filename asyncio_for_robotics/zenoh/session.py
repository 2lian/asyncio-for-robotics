import contextvars
import logging
from contextlib import contextmanager
from os import environ
from typing import Any, Generator, Optional, TypeVar
from warnings import warn

import zenoh

GLOBAL_SESSION: Optional[zenoh.Session] = None
_MISSING = object()
_CURRENT_SESSION: contextvars.ContextVar[zenoh.Session | None] = contextvars.ContextVar(
    "afor_zenoh_current_session",
    default=None,
)

logger = logging.getLogger(__name__)


_T = TypeVar("_T")


def current_session(default: _T = _MISSING) -> _T | zenoh.Session:
    """Return the current lexical Zenoh session.

    Args:
        default:
            Value returned when no lexical session context is active.
            If omitted, a RuntimeError is raised instead.
    """
    session = _CURRENT_SESSION.get()
    if session is None:
        if default is _MISSING:
            raise RuntimeError("No active Zenoh session context")
        return default
    return session


@contextmanager
def session_context(
    session: zenoh.Session, close_on_exit: bool = True
) -> Generator[zenoh.Session, Any, Any]:
    """Bind a Zenoh session lexically for this block and close it on exit.

    Inside this block, ``auto_session()`` resolves to this session unless an
    explicit session is passed directly.
    """
    token = _CURRENT_SESSION.set(session)
    try:
        yield session
    finally:
        _CURRENT_SESSION.reset(token)
        if close_on_exit:
            session.close()


def _open_default_session() -> zenoh.Session:
    if "ZENOH_SESSION_CONFIG_URI" in environ:
        config = zenoh.Config.from_file(environ["ZENOH_SESSION_CONFIG_URI"])
    else:
        warn(
            "'ZENOH_SESSION_CONFIG_URI' environment variable is not set. Using default session provided by zenoh"
        )
        config = zenoh.Config()
    try:
        return zenoh.open(config)
    except zenoh.ZError as e:
        e.add_note("Did you forget to start the zenoh router?")
        raise e


@contextmanager
def auto_context() -> Generator[zenoh.Session, Any, Any]:
    """Bind `auto_session()` lexically for this block.

    This is a convenience helper over ``with session_context(auto_session())``.

    If a lexical Zenoh session is already active, this reuses it and does not
    close it on exit. Otherwise, it resolves the normal auto session and closes
    it when leaving the block.
    """
    cur = current_session(None)
    if cur is not None:
        yield cur
        return
    with session_context(auto_session()) as session:
        yield session


def auto_session(session: Optional[zenoh.Session] = None) -> zenoh.Session:
    """Resolve a Zenoh session, creating a global one as a last resort.

    Resolution order:
        1. Explicit *session* argument (pass-through).
        2. Current ``session_context`` (lexical).
        3. ``GLOBAL_SESSION`` module-level singleton.
        4. Auto-create ``GLOBAL_SESSION`` from ``$ZENOH_SESSION_CONFIG_URI``
           or the Zenoh default config.
    """
    global GLOBAL_SESSION
    if session is not None:
        return session
    session = current_session(default=None)
    if session is not None:
        return session
    if GLOBAL_SESSION is not None and not GLOBAL_SESSION.is_closed():
        return GLOBAL_SESSION

    logger.info("Global zenoh session: Starting")
    ses = _open_default_session()
    GLOBAL_SESSION = ses
    logger.info("Global zenoh session: Running")
    return ses
