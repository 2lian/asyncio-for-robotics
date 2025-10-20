import logging
from os import environ
from typing import Optional
from warnings import warn

import zenoh

_maybe_session: Optional[zenoh.Session] = None

logger = logging.getLogger(__name__)


def set_auto_session(session: Optional[zenoh.Session] = None) -> None:
    """Set the global shared session instance.

    If called with a session, it replaces the current global session.
    If called with None, the global session is unset (but not close!).

    Args:
        session: zenoh session to set as default
    """
    global _maybe_session
    _maybe_session = session

def auto_session(session: Optional[zenoh.Session] = None) -> zenoh.Session:
    """Returns the passed session.
    If None, returns a singleton session shared with every other None call of
    this function."""
    global _maybe_session
    if _maybe_session is not None:
        return _maybe_session

    if "ZENOH_SESSION_CONFIG_URI" in environ:
        ZENOH_CONFIG = zenoh.Config.from_file(environ["ZENOH_SESSION_CONFIG_URI"])
    else:
        warn(f"'ZENOH_SESSION_CONFIG_URI' environment variable is not set. Using default session provided by zenoh")
        ZENOH_CONFIG = zenoh.Config()
    logger.info("Global zenoh session: Starting")
    ses = zenoh.open(ZENOH_CONFIG)
    set_auto_session(ses)
    logger.info("Global zenoh session: Running")
    return ses
