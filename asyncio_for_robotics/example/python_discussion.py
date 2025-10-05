"""
Example discussion module for asyncio-based subscribers.

This file is meant to demonstrate the different ways to consume data
from an `asyncio_for_robotics` subscriber, including:

- `wait_for_value`: get the latest value (can return the same value repeatedly).
- `wait_for_new`: wait for the most recent fresh message.
- `wait_for_next`: deterministically return the next message after the call.
- `listen`: async generator that streams new data but may skip intermediate messages.
- `listen_reliable`: async generator that queues all messages without skipping.

It also shows how to use timeouts to control asynchronous loops.
"""
import asyncio
from textwrap import dedent, fill
from typing import Any, Callable, Coroutine

from asyncio_for_robotics.core.sub import BaseSub, _MsgType
from asyncio_for_robotics.core.utils import soft_timeout


def brint(string: str):
    """print but creates a single paragraph of 70 columns"""
    print(fill(dedent(string)))


async def discuss(
    afor_sub: BaseSub[_MsgType],
    get_str_func: Callable[[_MsgType], str],
):

    sub = afor_sub
    print()
    brint(f"\nAwaiting value with `wait_for_value`")
    value = await sub.wait_for_value()
    brint(f"Received: {get_str_func(value)}")

    print()
    brint(
        f"""\nAwaiting value again with `wait_for_value` will return the same
        value as before."""
    )
    value = await sub.wait_for_value()
    brint(f"Received: {get_str_func(value)}")

    print()
    brint(
        f"""\nAwaiting NEW data with `wait_for_new`, will really wait for fresh
        data and return the most recent message."""
    )
    value = await sub.wait_for_new()
    brint(f"Received: {get_str_func(value)}")
    brint(f"`wait_for_value` is now: {get_str_func(await sub.wait_for_value())}")

    print()
    brint(
        f"""\n`wait_for_next` will also get fresh data. However will return a
        deterministic next-after-call value (first message after instantiation
                                             of the awaitable)"""
    )
    brint("Now, calling `wait_for_new` and `wait_for_next` simultaneously")
    new = sub.wait_for_new()
    next = sub.wait_for_next()
    brint(f"Value at time of call is: {get_str_func(await sub.wait_for_value())}")
    brint(f"Sleeping for 3 messages")
    for _ in range(3):
        await sub.wait_for_new()
    brint(f"wait_for_new: {get_str_func(await new)}")
    brint(f"wait_for_next: {get_str_func(await next)}")
    brint(
        f"""`wait_for_next` got exactly the next message after its call,
        whereas `wait_for_new` just displays the most recent value"""
    )

    print()
    brint(
        f"""\nAsync generators allow to continuously listen to data, simply in
        an async for loop. This can replace callbacks and handlers in
        ros/zenoh."""
    )
    count = 0
    async with soft_timeout(1):
        async for sample in sub.listen():
            brint(f"Processing: {get_str_func(sample)}")
            count += 1

    print()
    brint(
        f"""\nAs before with new/next, the `listen` method can miss messages.
        It only processes the latest message."""
    )
    count = 0
    async for sample in sub.listen():
        brint(f"Processing: {get_str_func(sample)}")
        brint(f"Sleeping for 0.5s")
        await asyncio.sleep(0.5)
        count += 1
        if count > 4:
            break

    print()
    brint(
        f"""\nOn the other hand, the `listen_reliable` method does not miss. It
        will properly build a queue of messages to process"""
    )
    count = 0
    async for sample in sub.listen_reliable(queue_size=40):
        brint(f"Processing: {get_str_func(sample)}")
        brint(f"Sleeping for 0.5s")
        await asyncio.sleep(0.5)
        count += 1
        if count > 4:
            break

    return
