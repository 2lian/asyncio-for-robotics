"""Implements a simple ros2 talker, publishing in a non-blocking asyncio loop"""

import asyncio
from contextlib import suppress

from std_msgs.msg import String

from asyncio_for_robotics.core.utils import Rate
import asyncio_for_robotics.ros2 as afor

TOPIC = afor.TopicInfo(msg_type=String, topic="example/talker")


@afor.scoped
async def main():
    # create the publisher safely
    with afor.auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)

    count = 0
    last_t = None
    async for t in Rate(frequency=2).listen_reliable():  # stable timer
        if last_t is None:
            last_t = t
        data = f"[Hello World! timestamp: {(t-last_t)/1e9}s]"
        count += 1
        print(f"Sending: {data}")
        pub.publish(String(data=data))  # sends data (lock is not necessary)
        await asyncio.sleep(0.5)  # non-blocking sleep


if __name__ == "__main__":
    with afor.auto_context():
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(main())
