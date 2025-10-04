from contextlib import suppress
import logging
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
        buff_size: int = 10,
    ) -> None:
        """
        Simple implementation of a Zenoh subscriber.

        Args:
            key_expr:
            session:
            buff_size:
        """
        self.session = self._resolve_session(session)
        self.sub = self._resolve_sub(key_expr)
        super().__init__(buff_size)

    @property
    def name(self) -> str:
        return f"zenoh-{self.sub.key_expr}"

    def _resolve_session(self, session: Optional[zenoh.Session]) -> zenoh.Session:
        return auto_session(session)

    def _resolve_sub(self, key_expr: Union[zenoh.KeyExpr, str]):
        return self.session.declare_subscriber(
            key_expr=key_expr, handler=self.callback_for_sub
        )

    def callback_for_sub(self, sample: zenoh.Sample):
        try:
            healty = self.input_data(sample)
            if not healty:
                self.sub.undeclare()
        except Exception as e:
            logger.error(e)

    def close(self):
        logger.debug("Closing ", self.name)
        if not self.session.is_closed():
            self.sub.undeclare()
        else:
            logger.debug("Zenoh session already closed for ", self.name)
