"""Implements a simple ros2 listener on `msg_type=String, topic="example/talker"`"""
import asyncio
from contextlib import suppress

import rclpy
from std_msgs.msg import String

from asyncio_for_robotics.ros2 import Sub, TopicInfo, auto_session

TOPIC = TopicInfo(msg_type=String, topic="example/talker")


async def main():
    # creates sub on the given topic
    sub = Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    # async for loop itterating every messages
    async for message in sub.listen_reliable():
        print(f"Received: {message.data}")


if __name__ == "__main__":
    rclpy.init()
    try:
        # suppress, just so we don't flood the terminal on exit
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(main()) # starts asyncio executor
    finally:
        # cleanup. `finally` statment always executes
        auto_session().close()
        rclpy.shutdown()
