import logging
from typing import Any, AsyncGenerator, Generator

import pytest
import rclpy
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from test_ros2_pubsub_thr import (
    test_freshness,
    test_listen_one_by_one,
    test_listen_too_fast,
    test_reliable_one_by_one,
    test_reliable_too_fast,
    test_wait_for_value,
    test_wait_new,
    test_wait_next,
)

import asyncio_for_robotics.ros2 as aros
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.ros2.session import SynchronousSession, set_auto_session

setup_logger(debug_path="tests")
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


TOPIC = aros.TopicInfo(
    "test/something",
    String,
    QoSProfile(
        depth=100,
    ),
)


@pytest.fixture
def pub(session: aros.BaseSession) -> Generator[Publisher, Any, Any]:
    with session.lock() as node:
        p: Publisher = node.create_publisher(**TOPIC.as_kwarg())
    yield p
    with session.lock() as node:
        node.destroy_publisher(p)


@pytest.fixture
async def sub(session) -> AsyncGenerator[aros.Sub[String], Any]:
    s: aros.Sub = aros.Sub(**TOPIC.as_kwarg())
    yield s
    s.close()
