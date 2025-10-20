import asyncio
import copy
import logging
from typing import Any, AsyncGenerator, Generator

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("yaml")
import rclpy
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool

import asyncio_for_robotics.ros2 as aros
from asyncio_for_robotics import soft_wait_for
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.ros2.service import Client, Server
from asyncio_for_robotics.ros2.session import ThreadedSession, set_auto_session

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module", autouse=True)
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
        depth=100,
    ),
)


@pytest.fixture
async def server(
    session: aros.BaseSession,
) -> AsyncGenerator[Server[SetBool.Request, SetBool.Response], Any]:
    server = Server(SetBool, "test/srv")
    yield server
    server.close()


@pytest.fixture
async def client(
    session: aros.BaseSession,
) -> AsyncGenerator[Client[SetBool.Request, SetBool.Response], Any]:
    client = Client(SetBool, "test/srv")
    yield client
    client.close()


async def test_client_receives_response(
    server: Server[SetBool.Request, SetBool.Response],
    client: Client[SetBool.Request, SetBool.Response],
):
    response_async = client.call(SetBool.Request(data=True))

    responder = await soft_wait_for(server.wait_for_value(), 1)
    assert not isinstance(responder, TimeoutError), f"Server did not receive request"
    assert responder.request.data == True
    responder.response.success = True
    responder.response.message = "hello"
    await responder.send()

    response = await soft_wait_for(response_async, 1)
    assert not isinstance(response, TimeoutError), f"Client did not receive reply"
    assert response.message == "hello"
    assert response.success == True
    return
