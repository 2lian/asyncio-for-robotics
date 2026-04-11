"""Implements a simple ros2 listener on `msg_type=String, topic="example/talker"`"""
import asyncio
from contextlib import suppress

from std_msgs.msg import String

import asyncio_for_robotics.ros2 as afor

TOPIC = afor.TopicInfo(msg_type=String, topic="example/talker")


@afor.scoped
async def main():
    # creates sub on the given topic
    sub = afor.Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    # async for loop itterating every messages
    async for message in sub.listen_reliable():
        print(f"Received: {message.data}")


if __name__ == "__main__":
    with afor.auto_context():
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(main())
