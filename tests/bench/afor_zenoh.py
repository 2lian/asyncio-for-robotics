import asyncio
import logging
import sys

import pyfiglet
import uvloop
import zenoh

from asyncio_for_robotics import soft_timeout
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.zenoh.session import auto_session
from asyncio_for_robotics.zenoh.sub import Sub

setup_logger("./")
logger = logging.getLogger("asyncio_for_robotics")

TOPIC_HELLO = "bench/hello"
TOPIC_WORLD = "bench/world"


async def increase_from_to(sub: Sub, pub: zenoh.Publisher):
    async for msg in sub.listen_reliable():
        i = int(msg.payload.to_bytes())
        pub.put(str(i + 1))
        if i % 500 == 0:
            print(f"message count: {i:_}")


async def main(hello: bool, world: bool):
    pub_hello = auto_session().declare_publisher(TOPIC_HELLO)
    pub_world = auto_session().declare_publisher(TOPIC_WORLD)

    sub_hello = Sub(TOPIC_HELLO)
    sub_world = Sub(TOPIC_WORLD)

    tasks = []
    if hello:
        tasks.append(asyncio.create_task(increase_from_to(sub_hello, pub_world)))
    if world:
        tasks.append(asyncio.create_task(increase_from_to(sub_world, pub_hello)))

    pub_hello.put(str(0))
    await asyncio.sleep(0.1)
    await sub_world.wait_for_value()
    await sub_hello.wait_for_new()

    test_timeout = 5

    async with soft_timeout(test_timeout):
        await asyncio.wait(tasks)

    score = (
        max(
            int((await sub_world.wait_for_value()).payload.to_bytes()),
            int((await sub_hello.wait_for_value()).payload.to_bytes()),
        )
        / test_timeout
    )
    print(f"\n{score=:_} Hz")
    print(pyfiglet.figlet_format(f"score :\n{score:_.0f} Hz".replace("_", ", ")))


if __name__ == "__main__":
    if "--world" in sys.argv:
        use_world = True
    else:
        use_world = False
    if "--hello" in sys.argv:
        use_hello = True
    else:
        use_hello = False
    if "--hello" not in sys.argv and "--world" not in sys.argv:
        use_hello = True
        use_world = True
    uvloop.run(main(use_hello, use_world))
    auto_session().close()
