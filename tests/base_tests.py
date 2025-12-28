import asyncio
import logging
from typing import Any, AsyncGenerator, Callable, Generator

import pytest

import asyncio_for_robotics.core as afor
from asyncio_for_robotics.core._logger import setup_logger

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")

async def test_wait_for_value(pub: Callable[[str], None], sub: afor.BaseSub[str]):
    logger.info("entered test")
    payload = "hello"
    logger.info("publishing")
    pub(payload)
    logger.info("awaiting data")
    sample = await afor.soft_wait_for(sub.wait_for_value(), 1)
    assert not isinstance(sample, TimeoutError), f"Did not receive response in time"
    logger.info("got data")
    logger.info(sample)
    assert sample == payload
    logger.info("passed")


async def test_wait_new(pub: Callable[[str], None], sub: afor.BaseSub[str]):
    payload = "hello"
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


async def test_wait_next(pub: Callable[[str], None], sub: afor.BaseSub[str]):
    first_payload = "hello"
    pub(first_payload)
    sample = await sub.wait_for_value()
    assert not isinstance(sample, TimeoutError), f"Should get a message"

    wait_task = sub.wait_for_next()
    new_sample = await afor.soft_wait_for(wait_task, 0.1)
    assert isinstance(new_sample, TimeoutError), f"Should not get a message"

    wait_task = sub.wait_for_next()
    pub(first_payload)
    for other_payload in range(10):
        await asyncio.sleep(0.005)
        pub(str(other_payload))

    new_sample = await afor.soft_wait_for(wait_task, 0.1)
    assert not isinstance(new_sample, TimeoutError), f"Should get the message"
    assert new_sample == first_payload


async def test_listen_one_by_one(pub: Callable[[str], None], sub: afor.BaseSub[str]):
    last_payload = "test"
    pub(last_payload)
    sample_count = 0
    put_count = 1
    max_iter = 20
    async for sample in sub.listen():
        sample_count += 1
        assert sample == last_payload
        if sample_count >= max_iter:
            break
        last_payload = f"test#{sample_count}"
        pub(last_payload)
        put_count += 1

    assert put_count == sample_count == max_iter


async def test_listen_too_fast(pub: Callable[[str], None], sub: afor.BaseSub[str]):
    delay = 0.005
    last_payload = "hello"
    pub(last_payload)
    pub(last_payload)
    sample_count = 0
    put_count = 2
    max_iter = 20
    await asyncio.sleep(delay)
    async for sample in sub.listen():
        sample_count += 1
        assert sample == last_payload
        if sample_count >= max_iter:
            break
        last_payload = f"hello{sample_count}"
        pub(last_payload)
        put_count += 1
        await asyncio.sleep(delay)
        last_payload = f"hello{sample_count}"
        pub(last_payload)
        put_count += 1
        await asyncio.sleep(delay)

    assert put_count / 2 == sample_count == max_iter


async def test_reliable_one_by_one(pub: Callable[[str], None], sub: afor.BaseSub[str]):
    last_payload = "hello"
    pub(last_payload)
    sample_count = 0
    put_count = 1
    max_iter = 20
    async for sample in sub.listen_reliable():
        sample_count += 1
        assert sample == last_payload
        if sample_count >= max_iter:
            break
        last_payload = f"hello{sample_count}"
        pub(last_payload)
        put_count += 1

    assert put_count == sample_count == max_iter


async def test_reliable_too_fast(pub: Callable[[str], None], sub: afor.BaseSub[str]):
    delay = 0.005
    data = list(range(30))
    put_queue = [str(v) for v in data]
    put_queue.reverse()
    received_buf = []
    listener = sub.listen_reliable(fresh=True, queue_size=len(data) * 2)
    await asyncio.sleep(delay)
    pub(put_queue.pop())
    await asyncio.sleep(delay)
    async with afor.soft_timeout(2):
        async for sample in listener:
            payload = int(sample)
            received_buf.append(payload)
            if len(received_buf) >= len(data):
                break
            if put_queue != []:
                pub(put_queue.pop())
                await asyncio.sleep(delay)
            if put_queue != []:
                pub(put_queue.pop())
                await asyncio.sleep(delay)

    assert data == received_buf


@pytest.mark.xfail(strict=False, reason="flaky depending on platform, middleware ...")
async def test_reliable_extremely_fast(pub: Callable[[str], None], sub: afor.BaseSub[str]):
    data = list(range(30))
    put_queue = [str(v) for v in data]
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


async def test_freshness(pub: Callable[[str], None], sub: afor.BaseSub[str]):
    payload = "hello"
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

