import asyncio
import logging
import uuid
from typing import Any, AsyncGenerator, Callable, Generator, Optional

import pytest

from asyncio_for_robotics.core import BaseSub
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.core.sub import ConverterSub

from .base_tests import (
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

pytest.importorskip("rclpy.experimental.async_node")

import rclpy
from rclpy.experimental.async_node import AsyncNode
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import asyncio_for_robotics.ros2_exp as afor

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module")
def rclpy_init() -> Generator[None, Any, Any]:
    logger.info("Starting rclpy")
    rclpy.init()
    yield
    logger.info("closing rclpy")
    rclpy.shutdown()


@pytest.fixture(scope="function")
async def session(rclpy_init) -> AsyncGenerator[AsyncNode, Any]:
    logger.info("Starting async_node")
    async with afor.async_context() as node:
        yield node
        logger.info("closing async_node")


topic = afor.TopicInfo(
    "test/something",
    String,
    QoSProfile(
        depth=10000,
    ),
)
TOPIC = topic


@pytest.fixture
def pub(session: AsyncNode) -> Generator[Callable[[str], None], Any, Any]:
    with session.create_publisher(*TOPIC.as_arg()) as publisher:

        def write_in_proc(input: str) -> None:
            publisher.publish(String(data=input))

        yield write_in_proc


@pytest.fixture
async def sub(session: AsyncNode) -> AsyncGenerator[BaseSub[str], Any]:
    inner_sub = afor.Sub(*TOPIC.as_arg())
    s: BaseSub[str] = ConverterSub(inner_sub, lambda msg: msg.data)
    yield s
    s.close()
