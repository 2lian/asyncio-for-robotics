import asyncio
import copy
import logging
from typing import Any, AsyncGenerator, Generator

import pytest
import rclpy
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from asyncio_for_robotics.core._logger import setup_logger
import asyncio_for_robotics.ros2 as aros
from asyncio_for_robotics import soft_wait_for

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="session", autouse=True)
def session() -> Generator[aros.RosSession, Any, Any]:
    logger.info("Starting rclpy and session")
    rclpy.init()
    ses = aros.auto_session()
    yield ses
    logger.info("closing rclpy and session")
    ses.close()
    rclpy.shutdown()


topic = aros.TopicInfo("test/something", String, QoSProfile(
    depth=100,
    ))


@pytest.fixture
def pub(session: aros.RosSession) -> Generator[Publisher, Any, Any]:
    with session.lock() as node:
        p: Publisher = node.create_publisher(**topic.as_kwarg())
    yield p


@pytest.fixture
async def sub(session) -> AsyncGenerator[aros.Sub[String], Any]:
    s: aros.Sub = aros.Sub(**topic.as_kwarg())
    yield s
    s.close()


async def test_wait_for_value(pub: Publisher, sub: aros.Sub[String]):
    logger.info("entered test")
    payload = "hello"
    logger.info("publishing")
    pub.publish(String(data=payload))
    logger.info("awaiting data")
    sample = await soft_wait_for(sub.wait_for_value(), 1)
    assert not isinstance(sample, TimeoutError), f"Did not receive response in time"
    assert sample.data == payload


async def test_wait_new(pub: Publisher, sub: aros.Sub[String]):
    payload = "hello"
    pub.publish(String(data=payload))
    sample = await sub.wait_for_value()
    assert not isinstance(sample, TimeoutError), f"Should get a message"

    wait_task = sub.wait_for_new()
    new_sample = await soft_wait_for(wait_task, 0.1)
    assert isinstance(new_sample, TimeoutError), f"Should not get a message"

    wait_task = sub.wait_for_new()
    pub.publish(String(data=payload))
    new_sample = await soft_wait_for(wait_task, 0.1)
    assert not isinstance(new_sample, TimeoutError), f"Should get the message"
    assert new_sample.data == payload


async def test_wait_next(pub: Publisher, sub: aros.Sub[String]):
    first_payload = "hello"
    pub.publish(String(data=first_payload))
    sample = await sub.wait_for_value()
    assert not isinstance(sample, TimeoutError), f"Should get a message"

    wait_task = sub.wait_for_next()
    new_sample = await soft_wait_for(wait_task, 0.1)
    assert isinstance(new_sample, TimeoutError), f"Should not get a message"

    wait_task = sub.wait_for_next()
    pub.publish(String(data=first_payload))
    for other_payload in range(10):
        await asyncio.sleep(0.001)
        pub.publish(String(data=str(other_payload)))

    new_sample = await soft_wait_for(wait_task, 0.1)
    assert not isinstance(new_sample, TimeoutError), f"Should get the message"
    assert new_sample.data == first_payload


async def test_listen_one_by_one(pub: Publisher, sub: aros.Sub[String]):
    last_payload = "hello"
    pub.publish(String(data=last_payload))
    sample_count = 0
    put_count = 1
    max_iter = 20
    async for sample in sub.listen():
        sample_count += 1
        assert sample.data == last_payload
        if sample_count >= max_iter:
            break
        last_payload = f"hello{sample_count}"
        pub.publish(String(data=last_payload))
        put_count += 1

    assert put_count == sample_count == max_iter


async def test_listen_too_fast(pub: Publisher, sub: aros.Sub[String]):
    last_payload = "hello"
    pub.publish(String(data=last_payload))
    pub.publish(String(data=last_payload))
    sample_count = 0
    put_count = 2
    max_iter = 20
    await asyncio.sleep(0.01)
    async for sample in sub.listen():
        sample_count += 1
        assert sample.data == last_payload
        if sample_count >= max_iter:
            break
        last_payload = f"hello{sample_count}"
        pub.publish(String(data=last_payload))
        put_count += 1
        last_payload = f"hello{sample_count}"
        pub.publish(String(data=last_payload))
        put_count += 1
        await asyncio.sleep(0.01)

    assert put_count / 2 == sample_count == max_iter


async def test_reliable_one_by_one(pub: Publisher, sub: aros.Sub[String]):
    last_payload = "hello"
    pub.publish(String(data=last_payload))
    sample_count = 0
    put_count = 1
    max_iter = 20
    async for sample in sub.listen_reliable():
        sample_count += 1
        assert sample.data == last_payload
        if sample_count >= max_iter:
            break
        last_payload = f"hello{sample_count}"
        pub.publish(String(data=last_payload))
        put_count += 1

    assert put_count == sample_count == max_iter


async def test_reliable_too_fast(pub: Publisher, sub: aros.Sub[String]):
    data = list(range(20))
    put_queue = [str(v) for v in data]
    received_buf = []
    listener = sub.listen_reliable(fresh=True)
    pub.publish(String(data=put_queue.pop()))
    await asyncio.sleep(0.1)
    pub.publish(String(data=put_queue.pop()))
    async for sample in listener:
        payload = int(sample.data)
        received_buf.append(payload)
        if len(received_buf) >= len(data):
            break
        if put_queue != []:
            pub.publish(String(data=put_queue.pop()))
        if put_queue != []:
            pub.publish(String(data=put_queue.pop()))

    received_buf.reverse()
    assert data == received_buf


async def test_freshness(pub: Publisher, sub: aros.Sub[String]):
    payload = "hello"
    new = sub.wait_for_new()
    pub.publish(String(data=payload))
    await new
    sample = await soft_wait_for(anext(sub.listen(fresh=False)), 0.1)
    assert not isinstance(sample, TimeoutError), f"Should get the message"
    assert sample.data == payload

    new = sub.wait_for_new()
    pub.publish(String(data=payload))
    await new
    sample = await soft_wait_for(anext(sub.listen_reliable(fresh=False)), 0.1)
    assert not isinstance(sample, TimeoutError), f"Should get the message"
    assert sample.data == payload
    await sub.wait_for_value()

    new = sub.wait_for_new()
    pub.publish(String(data=payload))
    await new
    sample = await soft_wait_for(anext(sub.listen(fresh=True)), 0.1)
    assert isinstance(sample, TimeoutError), f"Should NOT get the message"

    new = sub.wait_for_new()
    pub.publish(String(data=payload))
    await new
    sample = await soft_wait_for(anext(sub.listen_reliable(fresh=True)), 0.1)
    assert isinstance(sample, TimeoutError), f"Should NOT get the message"
