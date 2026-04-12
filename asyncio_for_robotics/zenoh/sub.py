import logging
import warnings
from contextlib import suppress
from typing import Callable, Dict, Generic, Optional, Type, TypeVar, Union

import zenoh

from ..core.scope import Scope
from ..core.sub import BaseSub, _AUTO_SCOPE
from .session import auto_session

logger = logging.getLogger(__name__)


class Sub(BaseSub[zenoh.Sample]):
    def __init__(
        self,
        key_expr: Union[zenoh.KeyExpr, str],
        session: Optional[zenoh.Session] = None,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ) -> None:
        """
        asyncio_for_robotics implementation of a Zenoh subscriber.

        Refere to the base class (BaseSub) for details.

        When created inside ``afor.Scope()``, leaving that scope automatically
        undeclares the underlying Zenoh subscriber.

        Args:
            key_expr:
            session:
        """
        self.session = self._resolve_session(session)
        self.sub = self._resolve_sub(key_expr)
        self._name = f"zenoh-{self.sub.key_expr}"
        super().__init__(scope=scope)

    @property
    def name(self) -> str:
        return self._name

    def _resolve_session(self, session: Optional[zenoh.Session]) -> zenoh.Session:
        """Called at __init__ to get the Session.

        Usefull to overide in a child class and change the Node behavior.
        """
        return auto_session(session)

    def _resolve_sub(self, key_expr: Union[zenoh.KeyExpr, str]):
        """Called at __init__ to create the subscriber.

        Usefull to overide in a child class and change the Subscription behavior.
        """
        return self.session.declare_subscriber(
            key_expr=key_expr, handler=self.input_data
        )

    def callback_for_sub(self, sample: zenoh.Sample):
        """DEPRECATED"""
        warnings.warn(
            "afor.zenoh.sub.Sub.callback_for_sub is deprecated. self.input_data"
            "can be used directly."
        )
        self.input_data(sample)

    def close(self):
        """Undeclares the subscriber on the zenoh session"""
        if not self._closed.is_set():
            if not self.session.is_closed():
                self.sub.undeclare()
            else:
                logger.debug("Zenoh session already closed for %s", self.name)
        super().close()
