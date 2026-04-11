import pytest

from asyncio_for_robotics.core.sub import ConverterSub

pytest.importorskip("rclpy")

import logging
from typing import Any, AsyncGenerator, Callable, Generator, Optional

import rclpy
from base_tests import (
    test_freshness,
    test_listen_one_by_one,
    test_listen_too_fast,
    test_loop_cancellation,
    test_reliable_extremely_fast,
    test_reliable_one_by_one,
    test_reliable_too_fast,
    test_wait_cancellation,
    test_wait_for_value,
    test_wait_new,
    test_wait_next,
)
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import asyncio_for_robotics.ros2 as afor
from asyncio_for_robotics.core import BaseSub
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.ros2.session import ThreadedSession

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module")
def session() -> Generator[afor.BaseSession, Any, Any]:
    logger.info("Starting rclpy and session")
    with afor.session_context(ThreadedSession()) as ses:
        yield ses
    logger.info("closing rclpy and session")


topic = afor.TopicInfo(
    "test/something",
    String,
    QoSProfile(
        depth=10000,
    ),
)
TOPIC = topic


@pytest.fixture(scope="module")
def pub(session: afor.BaseSession) -> Generator[Callable[[str], None], Any, Any]:
    with session.lock() as node:
        publisher = node.create_publisher(*TOPIC.as_arg())

    def write_in_proc(input: str) -> None:
        publisher.publish(String(data=input))

    yield write_in_proc
    with session.lock() as node:
        node.destroy_publisher(publisher)


@pytest.fixture
async def sub(session) -> AsyncGenerator[BaseSub[str], Any]:
    inner_sub = afor.Sub(*TOPIC.as_arg())
    s: BaseSub[str] = ConverterSub(inner_sub, lambda msg: msg.data)
    yield s
    s.close()
