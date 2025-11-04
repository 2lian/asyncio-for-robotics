import logging

import pytest

pytest.importorskip("zenoh")

import asyncio
from contextlib import suppress
from typing import Any, AsyncGenerator, Generator

import zenoh

from asyncio_for_robotics.zenoh import Sub, auto_session, soft_timeout, soft_wait_for

logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module", autouse=True)
def session() -> Generator[zenoh.Session, Any, Any]:
    ses = auto_session()
    yield ses
    if not auto_session().is_closed():
        with suppress(zenoh.ZError):
            ses.close()


@pytest.fixture
def pub(session) -> Generator[zenoh.Publisher, Any, Any]:
    pub_topic = "test/something"
    logger.debug("Creating PUB-%s", pub_topic)
    p: zenoh.Publisher = auto_session().declare_publisher(pub_topic)
    yield p
    if not auto_session().is_closed():
        logger.debug("closing PUB-%s", pub_topic)
        p.undeclare()


@pytest.fixture
async def sub(session) -> AsyncGenerator[Sub, Any]:
    s: Sub = Sub("test/**")
    yield s
    s.close()


async def test_wait_for_value(pub: zenoh.Publisher, sub: Sub):
    payload = b"hello"
    pub.put(payload)
    sample = await soft_wait_for(sub.wait_for_value(), 1)
    assert not isinstance(sample, TimeoutError), f"Did not receive response in time"
    assert sample.payload.to_bytes() == payload
    assert str(sample.key_expr) == str(pub.key_expr)


async def test_wait_new(pub: zenoh.Publisher, sub: Sub):
    payload = b"hello"
    pub.put(payload)
    sample = await sub.wait_for_value()
    assert not isinstance(sample, TimeoutError), f"Should get a message"

    wait_task = sub.wait_for_new()
    new_sample = await soft_wait_for(wait_task, 0.1)
    assert isinstance(new_sample, TimeoutError), f"Should not get a message"

    wait_task = sub.wait_for_new()
    pub.put(payload)
    new_sample = await soft_wait_for(wait_task, 0.1)
    assert not isinstance(new_sample, TimeoutError), f"Should get the message"
    assert new_sample.payload.to_bytes() == payload


async def test_wait_next(pub: zenoh.Publisher, sub: Sub):
    first_payload = b"hello"
    pub.put(first_payload)
    sample = await sub.wait_for_value()
    assert not isinstance(sample, TimeoutError), f"Should get a message"

    wait_task = sub.wait_for_next()
    new_sample = await soft_wait_for(wait_task, 0.1)
    assert isinstance(new_sample, TimeoutError), f"Should not get a message"

    wait_task = sub.wait_for_next()
    pub.put(first_payload)
    for other_payload in range(10):
        await asyncio.sleep(0.001)
        pub.put(str(other_payload))

    new_sample = await soft_wait_for(wait_task, 0.1)
    assert not isinstance(new_sample, TimeoutError), f"Should get the message"
    assert new_sample.payload.to_bytes() == first_payload


async def test_listen_one_by_one(pub: zenoh.Publisher, sub: Sub):
    last_payload = "hello"
    pub.put(last_payload)
    sample_count = 0
    put_count = 1
    max_iter = 20
    async for sample in sub.listen():
        sample_count += 1
        assert sample.payload.to_string() == last_payload
        if sample_count >= max_iter:
            break
        last_payload = f"hello{sample_count}"
        pub.put(last_payload)
        put_count += 1

    assert put_count == sample_count == max_iter


async def test_listen_too_fast(pub: zenoh.Publisher, sub: Sub):
    last_payload = "hello"
    pub.put(last_payload)
    pub.put(last_payload)
    sample_count = 0
    put_count = 2
    max_iter = 20
    await asyncio.sleep(0.001)
    async for sample in sub.listen():
        sample_count += 1
        assert sample.payload.to_string() == last_payload
        if sample_count >= max_iter:
            break
        last_payload = f"hello{sample_count}"
        pub.put(last_payload)
        put_count += 1
        await asyncio.sleep(0.001)
        last_payload = f"hello{sample_count}"
        pub.put(last_payload)
        put_count += 1
        await asyncio.sleep(0.001)

    assert put_count / 2 == sample_count == max_iter


async def test_reliable_one_by_one(pub: zenoh.Publisher, sub: Sub):
    last_payload = "hello"
    pub.put(last_payload)
    sample_count = 0
    put_count = 1
    max_iter = 20
    async for sample in sub.listen_reliable():
        sample_count += 1
        assert sample.payload.to_string() == last_payload
        if sample_count >= max_iter:
            break
        last_payload = f"hello{sample_count}"
        pub.put(last_payload)
        put_count += 1

    assert put_count == sample_count == max_iter


async def test_reliable_too_fast(pub: zenoh.Publisher, sub: Sub):
    data = list(range(200))
    put_queue = [str(v) for v in data]
    received_buf = []
    listener = sub.listen_reliable(fresh=True, queue_size=len(data))
    await asyncio.sleep(0.1)
    pub.put(put_queue.pop())
    await asyncio.sleep(0.001)
    pub.put(put_queue.pop())
    async with soft_timeout(2):
        async for sample in listener:
            payload = int(sample.payload.to_string())
            received_buf.append(payload)
            if len(received_buf) >= len(data):
                break
            if put_queue != []:
                pub.put(put_queue.pop())
                await asyncio.sleep(0.001)
            if put_queue != []:
                pub.put(put_queue.pop())
                await asyncio.sleep(0.001)

    received_buf.reverse()
    assert data == received_buf

async def test_reliable_extremely_fast(pub: zenoh.Publisher, sub: Sub):
    data = list(range(200))
    put_queue = [str(v) for v in data]
    received_buf = []
    listener = sub.listen_reliable(fresh=True, queue_size=len(data))
    pub.put(put_queue.pop())
    pub.put(put_queue.pop())
    async with soft_timeout(2):
        async for sample in listener:
            payload = int(sample.payload.to_string())
            received_buf.append(payload)
            if len(received_buf) >= len(data):
                break
            if put_queue != []:
                pub.put(put_queue.pop())
            if put_queue != []:
                pub.put(put_queue.pop())

    assert set(data) == set(received_buf)


async def test_freshness(pub: zenoh.Publisher, sub: Sub):
    payload = "hello"

    new = sub.wait_for_new()
    pub.put(payload)
    await new
    sample = await soft_wait_for(anext(sub.listen(fresh=False)), 0.1)
    assert not isinstance(sample, TimeoutError), f"Should get the message"
    assert sample.payload.to_string() == payload

    new = sub.wait_for_new()
    pub.put(payload)
    await new
    sample = await soft_wait_for(anext(sub.listen_reliable(fresh=False)), 0.1)
    assert not isinstance(sample, TimeoutError), f"Should get the message"
    assert sample.payload.to_string() == payload
    await sub.wait_for_value()

    new = sub.wait_for_new()
    pub.put(payload)
    await new
    sample = await soft_wait_for(anext(sub.listen(fresh=True)), 0.1)
    assert isinstance(sample, TimeoutError), f"Should NOT get the message"

    new = sub.wait_for_new()
    pub.put(payload)
    await new
    sample = await soft_wait_for(anext(sub.listen_reliable(fresh=True)), 0.1)
    assert isinstance(sample, TimeoutError), f"Should NOT get the message"
