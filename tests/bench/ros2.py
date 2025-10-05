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

TOPIC_HELLO = TopicInfo(msg_type=Int64, topic="bench/hello_ros")
TOPIC_WORLD = TopicInfo(msg_type=Int64, topic="bench/world_ros")


def increase_from_to(msg: Int64, pub: Publisher):
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
        with auto_session().lock() as node:
            node.create_subscription(
                TOPIC_HELLO.msg_type,
                TOPIC_HELLO.topic,
                lambda msg: increase_from_to(msg, pub_world),
                TOPIC_HELLO.qos,
            )
    if world:
        with auto_session().lock() as node:
            node.create_subscription(
                TOPIC_WORLD.msg_type,
                TOPIC_WORLD.topic,
                lambda msg: increase_from_to(msg, pub_hello),
                TOPIC_WORLD.qos,
            )

    pub_hello.publish(Int64(data=0))
    await asyncio.sleep(0.1)
    await sub_world.wait_for_value()
    await sub_hello.wait_for_new()

    test_timeout = 5

    await asyncio.sleep(test_timeout)
    print("test done")
    with auto_session().lock() as node:
        await asyncio.sleep(1)

        score = (
            max(
                (await sub_world.wait_for_value()).data,
                (await sub_hello.wait_for_value()).data,
            )
            / test_timeout
        )
        print(f"\n{score=:_} Hz")
        print(pyfiglet.figlet_format(f"score :\n{score:_.0f} Hz".replace("_", ", ")))
    auto_session().close()


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
    rclpy.shutdown()
