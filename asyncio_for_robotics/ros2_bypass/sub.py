"""Async subscriber backed by :class:`BatchAsyncioExecutor`."""

from typing import TypeVar

from rclpy.qos import QoSProfile

from ..core.scope import AUTO_SCOPE, Scope
from ..ros2.sub import Sub as RosSub
from ..ros2.utils import QOS_DEFAULT
from .session import Session, auto_session

_MsgType = TypeVar("_MsgType")


class Sub(RosSub[_MsgType]):
    """ROS subscriber whose callbacks run directly on the asyncio loop."""

    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: QoSProfile = QOS_DEFAULT,
        session: Session | None = None,
        *,
        scope: Scope | None = AUTO_SCOPE,
    ) -> None:
        super().__init__(
            msg_type=msg_type,
            topic=topic,
            qos_profile=qos_profile,
            session=session,
            scope=scope,
        )

    def _resolve_session(self, session: Session | None) -> Session:
        """Resolve the explicit or current asyncio-executor session."""
        return auto_session(session)

    def callback_for_sub(self, msg: _MsgType) -> None:
        """Deliver a ROS message without another cross-thread loop wakeup."""
        self._input_data_guarded(msg)
