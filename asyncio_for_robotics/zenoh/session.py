import contextvars
from contextlib import contextmanager
from os import environ
from typing import Any, Generator
from warnings import warn

import zenoh

_CURRENT_SESSION: contextvars.ContextVar[zenoh.Session | None] = contextvars.ContextVar(
    "afor_zenoh_current_session",
    default=None,
)

def current_session() -> zenoh.Session:
    """Return the current lexical Zenoh session."""
    session = _CURRENT_SESSION.get()
    if session is None:
        raise RuntimeError("No active Zenoh session context")
    return session


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
def auto_context(
    session: zenoh.Session | None = None,
) -> Generator[zenoh.Session, Any, Any]:
    """Bind the default Zenoh session for this block.

    Passing no session opens one from ``$ZENOH_SESSION_CONFIG_URI`` or Zenoh's
    default config and closes it on exit. An explicit session is bound but
    remains owned by the caller.
    """
    close_on_exit = session is None
    if session is None:
        session = _open_default_session()

    token = _CURRENT_SESSION.set(session)
    try:
        yield session
    finally:
        _CURRENT_SESSION.reset(token)
        if close_on_exit:
            session.close()


@contextmanager
def session_context(
    session: zenoh.Session,
    close_on_exit: bool = True,
) -> Generator[zenoh.Session, Any, Any]:
    """Deprecated compatibility wrapper for binding an explicit session."""
    warn(
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


def auto_session(session: zenoh.Session | None = None) -> zenoh.Session:
    """Use an explicit or current session, creating a warned fallback.

    Resolution order:
        1. Explicit ``session`` argument.
        2. Current lexical session context.
        3. Create a context-local fallback session.
    """
    if session is not None:
        return session
    try:
        return current_session()
    except RuntimeError:
        warn(
            "An `afor.zenoh` session was never declared. A fallback one is now "
            "instantiated. Prefer entering a context using `with auto_context(...)`"
        )
        session = _open_default_session()
        _CURRENT_SESSION.set(session)
        return session
