import logging
from os import environ
from typing import Optional

import zenoh

#: Zenoh config to be used by default (user can either overide by changing the environment variable or by overiding this variable)
ZENOH_CONFIG = zenoh.Config.from_file(environ["ZENOH_SESSION_CONFIG_URI"])
_maybe_session: Optional[zenoh.Session] = None

logger = logging.getLogger(__name__)


def auto_session(session: Optional[zenoh.Session] = None) -> zenoh.Session:
    """Returns the passed session.
    If None, returns a singleton session shared with every other None call of
    this function."""
    global _maybe_session
    if _maybe_session is None:
        logger.info("Global zenoh session: Starting")
        ses = zenoh.open(ZENOH_CONFIG)
        _maybe_session = ses
        logger.info("Global zenoh session: Running")
        return ses
    else:
        return _maybe_session
