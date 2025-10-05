import asyncio
import logging
import sys

import pyfiglet
import rclpy
import uvloop
from rclpy.publisher import Publisher
from std_msgs.msg import Int64

from asyncio_for_robotics import soft_timeout
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.ros2 import *

setup_logger("./")
logger = logging.getLogger("asyncio_for_robotics")

TOPIC_HELLO = TopicInfo(msg_type=Int64, topic="bench/hello")
TOPIC_WORLD = TopicInfo(msg_type=Int64, topic="bench/world")


async def increase_from_to(sub: Sub[Int64], pub: Publisher):
    async for msg in sub.listen_reliable():
        pub.publish(
            Int64(
                data=msg.data + 1,
            )
        )
        if msg.data % 500 == 0:
            print(f"message count: {msg.data:_}")


async def main(hello: bool, world: bool):
    with auto_session().lock() as node:
        pub_hello = node.create_publisher(
            TOPIC_HELLO.msg_type, TOPIC_HELLO.topic, TOPIC_HELLO.qos
        )
        pub_world = node.create_publisher(
            TOPIC_WORLD.msg_type, TOPIC_WORLD.topic, TOPIC_WORLD.qos
        )

    sub_hello = Sub(TOPIC_HELLO.msg_type, TOPIC_HELLO.topic, TOPIC_HELLO.qos)
    sub_world = Sub(TOPIC_WORLD.msg_type, TOPIC_WORLD.topic, TOPIC_WORLD.qos)

    tasks = []
    if hello:
        tasks.append(asyncio.create_task(increase_from_to(sub_hello, pub_world)))
    if world:
        tasks.append(asyncio.create_task(increase_from_to(sub_world, pub_hello)))

    pub_hello.publish(Int64(data=0))
    await asyncio.sleep(0.1)
    await sub_world.wait_for_value()
    await sub_hello.wait_for_new()

    test_timeout = 5

    async with soft_timeout(test_timeout):
        await asyncio.wait(tasks)
    print("test done")

    score = (
        max(
            (await sub_world.wait_for_value()).data,
            (await sub_hello.wait_for_value()).data,
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
    rclpy.init()
    uvloop.run(main(use_hello, use_world))
    auto_session().close()
    rclpy.shutdown()
