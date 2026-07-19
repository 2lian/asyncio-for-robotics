from __future__ import annotations

from typing import Optional, TypeVar

from rclpy.experimental import AsyncNode
from rclpy.experimental.async_subscription import AsyncSubscription
from rclpy.qos import QoSProfile, qos_profile_system_default

from ..core.scope import AUTO_SCOPE, Scope
from ..core.sub import BaseSub
from ..ros2.utils import TopicInfo
from .session import auto_session

_MsgType = TypeVar("_MsgType")


class Sub(BaseSub[_MsgType]):
    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: QoSProfile = qos_profile_system_default,
        session: Optional[AsyncNode] = None,
        *,
        scope: Scope | None = AUTO_SCOPE,
    ) -> None:
        self.session: AsyncNode = auto_session(session)
        self.topic_info: TopicInfo = TopicInfo(
            topic=topic, msg_type=msg_type, qos=qos_profile
        )
        self.sub: AsyncSubscription[_MsgType] = self._resolve_sub(self.topic_info)
        super().__init__(scope=scope)

    def _resolve_session(self, session: Optional[AsyncNode]) -> AsyncNode:
        """Called at __init__ to get the Node.

        Usefull to overide in a child class and change the Node behavior.
        """
        return auto_session(session)

    def _resolve_sub(self, topic_info: TopicInfo) -> AsyncSubscription[_MsgType]:
        """Called at __init__ to create the subscriber.

        Usefull to overide in a child class and change the Subscription behavior.
        """
        return self.session.create_subscription(
            **topic_info.as_kwarg(),
            callback=self._input_data_guarded,
        )

    @property
    def name(self) -> str:
        return f"ROS2-EXP-{self.topic_info.topic}"

    def close(self) -> None:
        if not self._closed.is_set():
            self.sub.destroy()
        super().close()
