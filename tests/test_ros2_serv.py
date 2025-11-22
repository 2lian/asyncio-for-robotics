import asyncio
import copy
import logging
import sys
from typing import Any, AsyncGenerator, Callable, Generator

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("yaml")
import rclpy
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool

import asyncio_for_robotics as afor
import asyncio_for_robotics.ros2 as aros
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.core.sub import BaseSub
from asyncio_for_robotics.ros2.service import Client, Responder, Server
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
    "test/srv",
    SetBool,
    QoSProfile(
        depth=500,
    ),
)


@pytest.fixture
async def server(
    session: aros.BaseSession,
) -> AsyncGenerator[Server[SetBool.Request, SetBool.Response], Any]:
    server = Server(**topic.as_kwarg())
    yield server
    server.close()


@pytest.fixture
async def client(
    session: aros.BaseSession,
) -> AsyncGenerator[Client[SetBool.Request, SetBool.Response], Any]:
    client = Client(**topic.as_kwarg())
    yield client
    client.close()


async def test_client_wait_fails_properly(
    client: Client[SetBool.Request, SetBool.Response],
):
    res = await afor.soft_wait_for(client.wait_for_service(), 1)
    assert isinstance(res, TimeoutError), f"No server is avaible, should timeout"


async def test_client_wait(
    server: Server[SetBool.Request, SetBool.Response],
    client: Client[SetBool.Request, SetBool.Response],
):
    res = await afor.soft_wait_for(client.wait_for_service(), 1)
    assert not isinstance(res, TimeoutError), f"Server is avaible, should not timeout"


async def test_client_receives_response(
    server: Server[SetBool.Request, SetBool.Response],
    client: Client[SetBool.Request, SetBool.Response],
):
    response_async = client.call(SetBool.Request(data=True))

    responder = await afor.soft_wait_for(server.wait_for_value(), 1)
    assert not isinstance(responder, TimeoutError), f"Server did not receive request"
    assert responder.request.data == True
    responder.response.success = True
    responder.response.message = "hello"
    responder.send()

    response = await afor.soft_wait_for(response_async, 1)
    assert not isinstance(response, TimeoutError), f"Client did not receive reply"
    assert response.message == "hello"
    assert response.success == True
    return


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
    s: BaseSub[bool] = BaseSub()

    def transmit(msg: Responder[SetBool.Request, SetBool.Response]):
        nonlocal s
        msg.send()
        s._input_data_asyncio(msg.request.data)

    server.asap_callback.append(transmit)
    yield s
    server.asap_callback.remove(transmit)


def binary(n: int):
    return n % 2 == 0


async def test_wait_for_value(pub: Callable[[bool], None], sub: BaseSub[bool]):
    logger.info("entered test")
    payload = True
    logger.info("publishing")
    pub(payload)
    logger.info("awaiting data")
    sample = await afor.soft_wait_for(sub.wait_for_value(), 1)
    assert not isinstance(sample, TimeoutError), f"Did not receive response in time"
    logger.info("got data")
    logger.info(sample)
    assert sample == payload
    logger.info("passed")


async def test_wait_new(pub: Callable[[bool], None], sub: BaseSub[bool]):
    payload = True
    pub(payload)
    sample = await sub.wait_for_value()
    assert not isinstance(sample, TimeoutError), f"Should get a message"

    wait_task = sub.wait_for_new()
    new_sample = await afor.soft_wait_for(wait_task, 0.1)
    assert isinstance(new_sample, TimeoutError), f"Should not get a message"

    wait_task = sub.wait_for_new()
    pub(payload)
    new_sample = await afor.soft_wait_for(wait_task, 0.1)
    assert not isinstance(new_sample, TimeoutError), f"Should get the message"
    assert new_sample == payload


async def test_wait_next(pub: Callable[[bool], None], sub: BaseSub[bool]):
    first_payload = True
    pub(first_payload)
    sample = await sub.wait_for_value()
    assert not isinstance(sample, TimeoutError), f"Should get a message"

    wait_task = sub.wait_for_next()
    new_sample = await afor.soft_wait_for(wait_task, 0.1)
    assert isinstance(new_sample, TimeoutError), f"Should not get a message"

    wait_task = sub.wait_for_next()
    pub(first_payload)
    for other_payload in range(10):
        await asyncio.sleep(0.001)
        pub(other_payload % 2 == 0)

    new_sample = await afor.soft_wait_for(wait_task, 0.1)
    assert not isinstance(new_sample, TimeoutError), f"Should get the message"
    assert new_sample == first_payload


async def test_listen_one_by_one(pub: Callable[[bool], None], sub: BaseSub[bool]):
    last_payload = True
    pub(last_payload)
    sample_count = 0
    put_count = 1
    max_iter = 20
    async for sample in sub.listen():
        sample_count += 1
        assert sample == last_payload
        if sample_count >= max_iter:
            break
        last_payload = binary(sample_count)
        pub(last_payload)
        put_count += 1

    assert put_count == sample_count == max_iter


async def test_listen_too_fast(pub: Callable[[bool], None], sub: BaseSub[bool]):
    if sys.platform.startswith("win"):
        sleep_time = 0.1
    else:
        sleep_time = 0.001
    last_payload = True
    pub(last_payload)
    sample_count = 0
    put_count = 2
    max_iter = 10
    await asyncio.sleep(0.01)
    async for sample in sub.listen():
        sample_count += 1
        assert sample == last_payload, f"failed on {sample_count=}"
        if sample_count >= max_iter:
            break
        last_payload = binary(sample_count)
        pub(last_payload)
        put_count += 1
        await asyncio.sleep(0.001)
        last_payload = binary(sample_count)
        pub(last_payload)
        put_count += 1
        await asyncio.sleep(sleep_time)

    assert sample_count == max_iter


async def test_reliable_one_by_one(pub: Callable[[bool], None], sub: BaseSub[bool]):
    last_payload = True
    pub(last_payload)
    sample_count = 0
    put_count = 1
    max_iter = 20
    async for sample in sub.listen_reliable():
        sample_count += 1
        assert sample == last_payload
        if sample_count >= max_iter:
            break
        last_payload = binary(sample_count)
        pub(last_payload)
        put_count += 1

    assert put_count == sample_count == max_iter


async def test_reliable_too_fast(pub: Callable[[bool], None], sub: BaseSub[bool]):
    data = [binary(n) for n in range(30)]
    put_queue: list[bool] = data.copy()
    put_queue.reverse()
    received_buf = []
    listener = sub.listen_reliable(fresh=True, queue_size=len(data) * 2)
    await asyncio.sleep(0.001)
    pub(put_queue.pop())
    async with afor.soft_timeout(2):
        async for sample in listener:
            payload = int(sample)
            received_buf.append(payload)
            if len(received_buf) >= len(data):
                break
            if put_queue != []:
                pub(put_queue.pop())
                await asyncio.sleep(0.001)
            if put_queue != []:
                pub(put_queue.pop())
                await asyncio.sleep(0.001)

    assert data == received_buf


@pytest.mark.xfail
async def test_reliable_extremely_fast(pub: Callable[[bool], None], sub: BaseSub[bool]):
    data = [binary(n) for n in range(30)]
    put_queue: list[bool] = [b for b in data]
    put_queue.reverse()
    received_buf = []
    listener = sub.listen_reliable(fresh=True, queue_size=len(data) * 2)
    pub(put_queue.pop())
    async with afor.soft_timeout(2):
        async for sample in listener:
            payload = int(sample)
            received_buf.append(payload)
            if len(received_buf) >= len(data):
                break
            if put_queue != []:
                pub(put_queue.pop())
            if put_queue != []:
                pub(put_queue.pop())

    assert set(data) == set(received_buf)


async def test_freshness(pub: Callable[[bool], None], sub: BaseSub[bool]):
    payload = True
    new = sub.wait_for_new()
    pub(payload)
    await new
    sample = await afor.soft_wait_for(anext(sub.listen(fresh=False)), 0.1)
    assert not isinstance(sample, TimeoutError), f"Should get the message"
    assert sample == payload

    new = sub.wait_for_new()
    pub(payload)
    await new
    sample = await afor.soft_wait_for(anext(sub.listen_reliable(fresh=False)), 0.1)
    assert not isinstance(sample, TimeoutError), f"Should get the message"
    assert sample == payload
    await sub.wait_for_value()

    new = sub.wait_for_new()
    pub(payload)
    await new
    sample = await afor.soft_wait_for(anext(sub.listen(fresh=True)), 0.1)
    assert isinstance(sample, TimeoutError), f"Should NOT get the message. got {sample}"

    new = sub.wait_for_new()
    pub(payload)
    await new
    sample = await afor.soft_wait_for(anext(sub.listen_reliable(fresh=True)), 0.1)
    assert isinstance(sample, TimeoutError), f"Should NOT get the message"
