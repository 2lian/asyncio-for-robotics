import logging
from typing import Callable, Dict, Generic, Optional, Type, TypeVar

import zenoh

from .core.sub import BaseSub

logger = logging.getLogger(__name__)


class RawSub(BaseSub[zenoh.Sample]):
    def __init__(
        self,
        key_expr: str,
        session: Optional[zenoh.Session] = None,
        buff_size: int = 10,
    ) -> None:
        self.session = self._resolve_session(session)
        self.sub = self._resolve_sub(key_expr)
        super().__init__(buff_size)

    @property
    def name(self) -> str:
        return f"raw-{self.sub.key_expr}"

    def _resolve_session(self, session: Optional[zenoh.Session]) -> zenoh.Session:
        return auto_session(session)

    def _resolve_sub(self, key_expr: str):
        return self.session.declare_subscriber(
            key_expr=key_expr, handler=self._unsafe_input_callback
        )

    def _unsafe_input_callback(self, sample: zenoh.Sample):
        try:
            healty = self.input_data(sample)
            if not healty:
                self.sub.undeclare()
        except Exception as e:
            logger.error(e)

    def close(self):
        self.sub.undeclare()
