import logging
from typing import Any, AsyncGenerator, Callable, Generator

import pytest

from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.core.sub import BaseSub

from .test_ros2_serv import (
    test_client_receives_response,
    test_client_wait,
    test_client_wait_fails_properly,
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

pytest.importorskip("rclpy.experimental.async_node")

import rclpy
from rclpy.experimental.async_node import AsyncNode
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool

import asyncio_for_robotics.ros2_exp as afor
from asyncio_for_robotics.ros2_exp.service import Client, Responder, Server

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
    "test/srv",
    SetBool,
    QoSProfile(
        depth=500,
    ),
)


@pytest.fixture
async def server(
    session: AsyncNode,
) -> AsyncGenerator[Server[SetBool.Request, SetBool.Response], Any]:
    server = Server(**topic.as_kwarg())
    yield server
    server.close()


@pytest.fixture
async def client(
    session: AsyncNode,
) -> AsyncGenerator[Client[SetBool.Request, SetBool.Response], Any]:
    client = Client(**topic.as_kwarg())
    yield client
    client.close()


@pytest.fixture
def pub(
    client: Client[SetBool.Request, SetBool.Response],
) -> Generator[Callable[[bool], None], Any, Any]:
    def send_payload(input: bool) -> None:
        req = SetBool.Request(data=input)
        client.call(req)

    yield send_payload


@pytest.fixture
async def sub(
    server: Server[SetBool.Request, SetBool.Response],
) -> AsyncGenerator[BaseSub[bool], Any]:
    sub: BaseSub[bool] = BaseSub()

    def transmit(msg: Responder[SetBool.Request, SetBool.Response]):
        msg.send()
        sub._input_data_asyncio(msg.request.data)

    server.asap_callback.append(transmit)
    yield sub
    server.asap_callback.remove(transmit)
