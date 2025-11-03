"""Implements a simple ros2 talker, publishing in a non-blocking asyncio loop"""

import asyncio
from contextlib import suppress

import rclpy
from std_msgs.msg import String

from asyncio_for_robotics.core.utils import Rate
from asyncio_for_robotics.ros2 import TopicInfo, auto_session

TOPIC = TopicInfo(msg_type=String, topic="example/talker")


async def main():
    # create the publisher safely
    with auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)

    count = 0
    async for t in Rate(frequency=2).listen_reliable():  # stable timer
        data = f"[Hello world! timestamp: {count/10:.1f}s]"
        count += 1
        print(f"Sending: {data}")
        pub.publish(String(data=data))  # sends data (lock is not necessary)
        await asyncio.sleep(0.5)  # non-blocking sleep


if __name__ == "__main__":
    rclpy.init()
    try:
        # suppress, just so we don't flood the terminal on exit
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(main())  # starts asyncio executor
    finally:
        # cleanup. `finally` statment always executes
        auto_session().close()
        rclpy.shutdown()
