import asyncio

from asyncio_for_robotics.zenoh.session import auto_session
from asyncio_for_robotics.zenoh.sub import Sub


async def talking_loop():
    pub = auto_session().declare_publisher("example/discussion")
    print(f"Publishing onto {pub.key_expr}")
    try:
        count = 0
        while 1:
            pub.put(f"[Hello world! timestamp: {count/10:.1f}s]")
            count += 1
            await asyncio.sleep(0.1)
    finally:
        # pass
        pub.undeclare()


async def main():
    print(
        f"Calling `auto_session()` will start and return a global zenoh session. asyncio_for_robotics will automatically use `auto_session` unless you pass you own zenoh session."
    )
    background_talker_task = asyncio.create_task(talking_loop())
    await asyncio.sleep(0.001)

    sub = Sub("example/**")
    print(f"Creating subscriber ({sub.sub.key_expr})")
    print(f"\nAwaiting value with `wait_for_value`")
    value = await sub.wait_for_value()
    print(f"Received: {value.payload.to_string()}")

    print(
        f"\nAwaiting value again with `wait_for_value` will return the same value as before."
    )
    value = await sub.wait_for_value()
    print(f"Received: {value.payload.to_string()}")

    print(
        f"\nAwaiting NEW data with `wait_for_new`, will really wait for fresh data and return the most recent message."
    )
    value = await sub.wait_for_new()
    print(f"Received: {value.payload.to_string()}")
    print(
        f"`wait_for_value` is now: {(await sub.wait_for_value()).payload.to_string()}"
    )

    print(
        f"\n`wait_for_next` will also get fresh data. However will return a deterministic next-after-call value (first message after instantiation of the awaitable)"
    )
    print("Now, calling `wait_for_new` and `wait_for_next` simultaneously")
    new = sub.wait_for_new()
    next = sub.wait_for_next()
    print(
        f"Value at time of call is: {(await sub.wait_for_value()).payload.to_string()}"
    )
    print(f"Sleeping for 3 messages")
    for k in range(3):
        await sub.wait_for_new()
    print(f"wait_for_new: {(await new).payload.to_string()}")
    print(f"wait_for_next: {(await next).payload.to_string()}")
    print(
        f"`wait_for_next` got exactly the next message after its call, whereas `wait_for_new` just displays the most recent value"
    )

    print(
        f"\nAsync generators allow to continuously listen to data, simply in an async for loop. This can replace callbacks and handlers in ros/zenoh."
    )
    count = 0
    async for sample in sub.listen():
        print(f"Processing: {sample.payload.to_string()}")
        count += 1
        if count > 10:
            break

    print(
        f"\nAs before with new/next, the `listen` method can miss messages. It only processes the latest message."
    )
    count = 0
    async for sample in sub.listen():
        print(f"Processing: {sample.payload.to_string()}")
        print(f"Sleeping for 0.5s")
        await asyncio.sleep(0.5)
        count += 1
        if count > 4:
            break

    print(
        f"\nOn the other hand, the `listen_reliable` method does not miss. It will properly build a queue of messages to process"
    )
    count = 0
    async for sample in sub.listen_reliable(queue_size=40):
        print(f"Processing: {sample.payload.to_string()}")
        print(f"Sleeping for 0.5s")
        await asyncio.sleep(0.5)
        count += 1
        if count > 4:
            break

    background_talker_task.cancel()
    return


if __name__ == "__main__":
    asyncio.run(main())
    print(
        f"\nTo finish and let python exit, the zenoh session needs to be closed with `auto_session().close()`"
    )
    auto_session().close()
