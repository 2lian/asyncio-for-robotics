"""Implements a publisher and subscriber in the same node.

run with `python3 -m asyncio_for_robotics.example.ros2_pubsub`"""
import asyncio
from contextlib import suppress

import rclpy
from std_msgs.msg import String

from asyncio_for_robotics.core.utils import Rate
import asyncio_for_robotics.ros2 as afor

TOPIC = afor.TopicInfo(msg_type=String, topic="topic")


async def hello_world_publisher():
    # create the publisher safely
    with afor.auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)

    i = 0
    async for t in Rate(frequency=2).listen_reliable(): # non stable timer
        data = f"Hello world: {i}"
        i += 1
        print(f"Publishing: {data}")
        pub.publish(String(data=data))  # sends data (lock is not necessary)


async def hello_world_subscriber():
    # creates sub on the given topic
    sub = afor.Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    # async for loop itterating every messages
    async for message in sub.listen_reliable():
        print(f"Received: {message.data}")


async def hello_world_pubsub():
    sub_task = asyncio.create_task(hello_world_subscriber())
    pub_task = asyncio.create_task(hello_world_publisher())
    await asyncio.wait([pub_task, sub_task])


if __name__ == "__main__":
    rclpy.init()
    try:
        # suppress, just so we don't flood the terminal on exit
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(hello_world_pubsub())  # starts asyncio executor
    finally:
        # cleanup. `finally` statment always executes
        afor.auto_session().close()
        rclpy.shutdown()
