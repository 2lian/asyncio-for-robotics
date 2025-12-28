import logging
from typing import Any, AsyncGenerator, Callable, Generator

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("yaml")
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
    test_reliable_extremely_fast,
    test_reliable_one_by_one,
    test_reliable_too_fast,
    test_wait_for_value,
    test_wait_new,
    test_wait_next,
)

import asyncio_for_robotics.ros2 as aros
from asyncio_for_robotics.ros2.session import SynchronousSession, set_auto_session

logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module", autouse=True)
def rclpy_init():
    logger.info("Starting rclpy")
    rclpy.init()
    yield
    logger.info("closing rclpy ")
    rclpy.shutdown()


@pytest.fixture(scope="function", autouse=True)
async def session(rclpy_init) -> AsyncGenerator[aros.BaseSession, Any]:
    logger.info("Starting session")
    set_auto_session(SynchronousSession())
    ses = aros.auto_session()
    yield ses
    logger.info("closing session")
    ses.close()


@pytest.fixture(scope="function")
def pub(session: aros.BaseSession) -> Generator[Callable[[str], None], Any, Any]:
    with session.lock() as node:
        publisher = node.create_publisher(*TOPIC.as_arg())

    def write_in_proc(input: str) -> None:
        publisher.publish(String(data=input))

    yield write_in_proc
    with session.lock() as node:
        node.destroy_publisher(publisher)
