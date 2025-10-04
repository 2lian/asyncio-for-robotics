import logging
from typing import Callable, Dict, Generic, Optional, Type, TypeVar, Union

from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription

from ..core.sub import BaseSub
from .session import RosSession, TopicInfo, _MsgType, auto_session

logger = logging.getLogger(__name__)


class Sub(BaseSub[_MsgType]):
    def __init__(
        self,
        topic: str,
        msg_type: _MsgType,
        qos: QoSProfile,
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
        self.topic_info = TopicInfo(topic, msg_type, qos)
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
                topic_info.msg_type,
                topic_info.name,
                self.callback_for_sub,
                topic_info.qos,
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
