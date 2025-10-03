import logging
from abc import ABC, abstractmethod
from typing import (
    Any,
    AsyncGenerator,
    Awaitable,
    Callable,
    Deque,
    Final,
    Generic,
    List,
    Optional,
    Set,
    TypeVar,
)

logger = logging.getLogger(__name__)

_MsgType = TypeVar("_MsgType")

class BasePub(Generic[_MsgType], ABC):
    def __init__(self):
        """Standalone ros2 publisher working with python asyncio.

        Args:
            node: A ROS2 BackgroundNode of this async module that is running.
            topic: Topic to publish onto
        """
        ...

    @abstractmethod
    def pub(self, data: _MsgType):
        """Publishes a message"""
        ...
