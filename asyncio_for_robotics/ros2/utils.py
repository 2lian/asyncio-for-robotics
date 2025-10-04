from typing import Any, Dict, Final, Generic, NamedTuple, TypeVar

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


class _TopicRosarg(NamedTuple, Generic[_MsgType]):
    msg_type: _MsgType
    topic: str
    qos_profile: QoSProfile = QOS_DEFAULT


class TopicInfo(NamedTuple, Generic[_MsgType]):
    """Precisely describes a ROS2 topic

    Attributes:
        msg_type:
        name:
        qos:
    """

    topic: str
    msg_type: _MsgType
    qos: QoSProfile = QOS_DEFAULT

    def as_arg(self) -> _TopicRosarg:
        return _TopicRosarg(self.msg_type, self.topic, self.qos)

    def as_kwarg(self) -> Dict[str, Any]:
        return _TopicRosarg(self.msg_type, self.topic, self.qos)._asdict()
