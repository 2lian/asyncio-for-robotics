import logging
from typing import Any, AsyncGenerator, Callable, Generator, Optional

import pytest

pytest.importorskip("rclpy")
import rclpy
from base_tests import (
    test_freshness,
    test_listen_one_by_one,
    test_listen_too_fast,
    test_reliable_extremely_fast,
    test_reliable_one_by_one,
    test_reliable_too_fast,
    test_wait_for_value,
    test_wait_new,
    test_wait_next,
)
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import asyncio_for_robotics.ros2 as aros
from asyncio_for_robotics.core import BaseSub
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.ros2.session import ThreadedSession, set_auto_session

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module")
def session() -> Generator[aros.BaseSession, Any, Any]:
    logger.info("Starting rclpy and session")
    rclpy.init()
    set_auto_session(ThreadedSession())
    ses = aros.auto_session()
    yield ses
    logger.info("closing rclpy and session")
    ses.close()
    rclpy.shutdown()


topic = aros.TopicInfo(
    "test/something",
    String,
    QoSProfile(
        depth=10000,
    ),
)
TOPIC = topic

class SubProcessed(BaseSub[str]):
    def __init__(
        self,
        msg_type: type[String],
        topic: str,
        qos_profile: QoSProfile = aros.QOS_DEFAULT,
        session: Optional[aros.BaseSession] = None,
    ) -> None:
        super().__init__()
        self.ros_sub = aros.Sub(msg_type, topic, qos_profile, session)
        self.ros_sub.asap_callback.append(lambda x: self._input_data_asyncio(x.data))

    def close(self):
        self.ros_sub.close()


@pytest.fixture(scope="module")
def pub(session: aros.BaseSession) -> Generator[Callable[[str], None], Any, Any]:
    with session.lock() as node:
        publisher = node.create_publisher(*TOPIC.as_arg())

    def write_in_proc(input: str) -> None:
        publisher.publish(String(data=input))

    yield write_in_proc
    with session.lock() as node:
        node.destroy_publisher(publisher)


@pytest.fixture
async def sub(session) -> AsyncGenerator[BaseSub[str], Any]:
    s: BaseSub[str] = SubProcessed(*TOPIC.as_arg())
    yield s
    s.close()
