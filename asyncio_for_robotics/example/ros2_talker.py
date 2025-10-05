"""Implements a simple ros2 talker, publishing in a non-blocking asyncio loop"""
import asyncio

import rclpy
from std_msgs.msg import String

from asyncio_for_robotics.ros2.session import auto_session
from asyncio_for_robotics.ros2.utils import TopicInfo

TOPIC = TopicInfo(msg_type=String, topic="example/talker")


async def main():
    # create the publisher safely
    with auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)

    count = 0
    while 1:  # This loop is not very precise and can drift
        data = f"[Hello world! timestamp: {count/10:.1f}s]"
        count += 1
        print(f"Sending: {data}")
        pub.publish(String(data=data)) # sends data (lock is not necessary)
        await asyncio.sleep(0.1) # non-blocking sleep


if __name__ == "__main__":
    rclpy.init()
    asyncio.run(main())
    auto_session().close()
    rclpy.shutdown()
