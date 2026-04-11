import pytest

pytest.importorskip("rclpy")
pytest.importorskip("yaml")

import logging
from typing import Any, AsyncGenerator, Callable, Generator

import rclpy
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from test_ros2_pubsub_thr import (
    TOPIC,
    sub,
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

import asyncio_for_robotics.ros2 as afor
from asyncio_for_robotics.ros2.session import SynchronousSession

logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="function", autouse=True)
async def session() -> AsyncGenerator[afor.BaseSession, Any]:
    logger.info("Starting session")
    with afor.session_context(SynchronousSession()) as ses:
        yield ses
    logger.info("closing session")


@pytest.fixture(scope="function")
def pub(session: afor.BaseSession) -> Generator[Callable[[str], None], Any, Any]:
    with session.lock() as node:
        publisher = node.create_publisher(*TOPIC.as_arg())

    def write_in_proc(input: str) -> None:
        publisher.publish(String(data=input))

    yield write_in_proc
    with session.lock() as node:
        node.destroy_publisher(publisher)
