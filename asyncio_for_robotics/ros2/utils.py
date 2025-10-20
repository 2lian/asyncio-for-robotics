from dataclasses import dataclass, field
from typing import Any, Dict, Final, Generic, NamedTuple, Tuple, TypeVar

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_system_default,
)

#: Default qos
QOS_DEFAULT: Final = qos_profile_system_default

#: "always available" qos
QOS_TRANSIENT: Final = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


_MsgType = TypeVar("_MsgType")


@dataclass(frozen=True, slots=True)
class TopicInfo(Generic[_MsgType]):
    """Precisely describes a ROS2 topic

    Attributes:
        name:
        msg_type:
        qos:
    """

    topic: str
    msg_type: _MsgType
    qos: QoSProfile = field(default_factory=lambda *_, **__: QOS_DEFAULT)

    def as_arg(self) -> Tuple[_MsgType, str, QoSProfile]:
        return (self.msg_type, self.topic, self.qos)

    def as_kwarg(self) -> Dict[str, Any]:
        return {"msg_type": self.msg_type, "topic": self.topic, "qos_profile": self.qos}
