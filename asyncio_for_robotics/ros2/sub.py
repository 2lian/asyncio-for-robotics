import logging
from typing import Callable, Dict, Generic, Optional, Type, TypeVar, Union

from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription

from ..core.sub import BaseSub
from .session import RosSession, auto_session
from .utils import QOS_DEFAULT, TopicInfo, _MsgType

logger = logging.getLogger(__name__)


class Sub(BaseSub[_MsgType]):
    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: QoSProfile = QOS_DEFAULT,
        session: Optional[RosSession] = None,
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
        self.topic_info = TopicInfo(topic=topic, msg_type=msg_type, qos=qos_profile)
        self.sub = self._resolve_sub(self.topic_info)
        super().__init__(buff_size)

    @property
    def name(self) -> str:
        return f"ROS2-{self.sub.topic_name}"

    def _resolve_session(self, session: Optional[RosSession]) -> RosSession:
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
                with self.session.lock() as node:
                    node.destroy_subscription(self.sub)
        except Exception as e:
            logger.error(e)

    def close(self):
        with self.session.lock() as node:
            node.destroy_subscription(self.sub)
