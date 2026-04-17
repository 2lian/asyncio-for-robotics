import logging
import warnings
from typing import Callable, Dict, Generic, Optional, Type, TypeVar, Union

from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription

from ..core.scope import AUTO_SCOPE, Scope
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
        *,
        scope: Scope | None = AUTO_SCOPE,
    ) -> None:
        """
        Implementation of a asyncio ROS2 subscriber.

        Refere to the base class (BaseSub) for details.

        When created inside ``afor.Scope()``, leaving that scope automatically
        destroys the underlying ROS 2 subscription.

        Args:
            msg_type: The type of ROS messages the subscription will subscribe to.
            topic: The name of the topic the subscription will subscribe to.
            qos: A QoSProfile to apply to the subscription.
            session: The ROS 2 node to use. If not provided, will use
                auto_session to create/get one.
        """
        self.session: BaseSession = self._resolve_session(session)
        self.topic_info = TopicInfo(topic=topic, msg_type=msg_type, qos=qos_profile)
        self.sub = self._resolve_sub(self.topic_info)
        super().__init__(scope=scope)

    @classmethod
    def from_info(
        cls,
        topic_info: TopicInfo,
        session: Optional[BaseSession] = None,
    ) -> Self:
        """Creates the sub from a topic info directly.

        TypeHinting doesn't work with that so... I don't like it
        """
        return cls(**topic_info.as_kwarg(), session=session)

    @property
    def name(self) -> str:
        try:
            return f"ROS2-{self.sub.topic_name}"
        except:
            return f"ROS2-{self.topic_info.topic}"

    def _resolve_session(self, session: Optional[BaseSession]) -> BaseSession:
        """Called at __init__ to get the Node.

        Usefull to overide in a child class and change the Node behavior.
        """
        return auto_session(session)

    def _resolve_sub(self, topic_info: TopicInfo) -> Subscription:
        """Called at __init__ to create the subscriber.

        Usefull to overide in a child class and change the Subscription behavior.
        """
        with self.session.lock() as node:
            return node.create_subscription(
                **topic_info.as_kwarg(),
                callback=self.callback_for_sub,
            )

    def callback_for_sub(self, msg: _MsgType):
        """Callback of the ROS 2 Subscriber.

        This might be executed in another thread.

        Args:
            sample: incoming message
        """
        # warnings.warn(
        #     "afor.zenoh.sub.Sub.callback_for_sub is deprecated. self.input_data"
        #     "can be used directly."
        # )
        self.input_data(msg)

    def close(self):
        super().close()
        with self.session.lock() as node:
            if not node.executor.context.ok():
                return
            node.destroy_subscription(self.sub)
