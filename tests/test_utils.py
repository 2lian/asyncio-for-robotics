import asyncio
from time import time

from pytest import approx
import pytest
import asyncio_for_robotics as afor

async def test_soft_wait_for_coro():
    res = await afor.soft_wait_for(asyncio.sleep(1), 0.001)
    assert isinstance(res, TimeoutError), "Should timeout"

async def test_soft_wait_for_fut():
    fut = asyncio.create_task(asyncio.sleep(1))
    res = await afor.soft_wait_for(fut, 0.001)
    assert isinstance(res, TimeoutError), "Should timeout"

async def test_soft_timeout():
    fut = asyncio.create_task(asyncio.sleep(1))
    res = 'oh nyo'
    async with afor.soft_timeout(0.001):
        await asyncio.sleep(1)
        res = 'did not timeout'
    assert res == 'oh nyo', "Should timeout"

async def test_soft_timeout_return_not_timeout():
    async with afor.soft_timeout(0.001) as timeouted:
        await asyncio.sleep(1)
    assert timeouted() == True, "yielded function should return True when timeout occurs"

async def test_soft_timeout_return_timeout():
    async with afor.soft_timeout(1) as timeouted:
        await asyncio.sleep(0.001)
    assert timeouted() == False, "yielded function should return False when timeout doe not occur"

async def test_rate():
    async with afor.soft_timeout(1) as timeouted:
        count = 0
        start = time()
        async for call_time in afor.Rate(100).listen_reliable():
            count += 1
            if count >= 10:
                break
        end = time()
        dt = end - start
        assert dt == approx(0.1, abs=0.1)

    if timeouted() == True:
        pytest.fail(f"test took too long")
