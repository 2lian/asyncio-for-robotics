import logging

from typing import Callable, Dict, Generic, Optional, Type, TypeVar, Union

from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription

from ..core.sub import BaseSub
from .session import BaseSession, auto_session
from .utils import QOS_DEFAULT, TopicInfo, _MsgType

try:
    from typing import Self  # Python 3.11+
except ImportError:
    from typing_extensions import Self  # Python 3.10

logger = logging.getLogger(__name__)


class Sub(BaseSub[_MsgType]):
    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: QoSProfile = QOS_DEFAULT,
        session: Optional[BaseSession] = None,
    ) -> None:
        """
        Simple implementation of a Zenoh subscriber.

        Args:
            key_expr:
            session:
            buff_size:
        """
        self.session: BaseSession = self._resolve_session(session)
        self.topic_info = TopicInfo(topic=topic, msg_type=msg_type, qos=qos_profile)
        self.sub = self._resolve_sub(self.topic_info)
        super().__init__()

    @classmethod
    def from_info(
        cls: type[Self],
        topic_info: TopicInfo,
        session: Optional[BaseSession] = None,
        buff_size: int = 10,
    ) -> Self:
        return cls(**topic_info.as_kwarg(), session=session, buff_size=buff_size)

    @property
    def name(self) -> str:
        try:
            return f"ROS2-{self.sub.topic_name}"
        except:
            return f"ROS2-{self.topic_info.topic}"

    def _resolve_session(self, session: Optional[BaseSession]) -> BaseSession:
        return auto_session(session)

    def _resolve_sub(self, topic_info: TopicInfo) -> Subscription:
        with self.session.lock() as node:
            return node.create_subscription(
                **topic_info.as_kwarg(),
                callback=self.callback_for_sub,
            )

    def callback_for_sub(self, sample: _MsgType):
        try:
            healty = self.input_data(sample)
            if not healty:
                self.session._node.destroy_subscription(self.sub)
        except Exception as e:
            logger.error(e)

    def close(self):
        with self.session.lock() as node:
            if not node.executor.context.ok():
                return
            node.destroy_subscription(self.sub)
