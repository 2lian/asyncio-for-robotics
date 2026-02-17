import logging
from contextlib import suppress
from typing import Callable, Dict, Generic, Optional, Type, TypeVar, Union

import zenoh

from ..core.sub import BaseSub
from .session import auto_session

logger = logging.getLogger(__name__)


class Sub(BaseSub[zenoh.Sample]):
    def __init__(
        self,
        key_expr: Union[zenoh.KeyExpr, str],
        session: Optional[zenoh.Session] = None,
    ) -> None:
        """
        asyncio_for_robotics implementation of a Zenoh subscriber.

        Args:
            key_expr:
            session:
        """
        self.session = self._resolve_session(session)
        self.sub = self._resolve_sub(key_expr)
        self._name = f"zenoh-{self.sub.key_expr}"
        super().__init__()

    @property
    def name(self) -> str:
        return self._name

    def _resolve_session(self, session: Optional[zenoh.Session]) -> zenoh.Session:
        return auto_session(session)

    def _resolve_sub(self, key_expr: Union[zenoh.KeyExpr, str]):
        return self.session.declare_subscriber(
            key_expr=key_expr, handler=self.callback_for_sub
        )

    def callback_for_sub(self, sample: zenoh.Sample):
        """Zenoh callback happening on the Zenoh thread"""
        if self._closed.is_set():
            return
        try:
            healty = self.input_data(sample)
            if not healty:
                self.close()
        except Exception as e:
            logger.error(e)

    def close(self):
        """Undeclares the subscriber on the zenoh session"""
        if not self._closed.is_set():
            if not self.session.is_closed():
                self.sub.undeclare()
            else:
                logger.debug("Zenoh session already closed for %s", self.name)
        super().close()
