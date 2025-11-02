"""
Example ROS 2 publishers using asyncio integration.

This example demonstrates how to:
- Safely create and destroy publishers using `auto_session().lock()`.
- Publish messages asynchronously at regular intervals with non-blocking
  `asyncio.sleep`.
- Run multiple publishers concurrently and cancel them dynamically.
- Ensure cleanup of publishers when tasks are cancelled or errors occur,
  which is especially useful for testing scenarios with short-lived
  publishers and subscribers.

The script alternates between running:
  - Publisher #1 alone.
  - Publisher #1 and Publisher #2 together.
  - Only Publisher #2.

Run this script using `python3 -m asyncio_for_robotics.example.ros2_double_talker`
Run this script side by side with `python3 -m asyncio_for_robotics.example.ros2_double_listener`
"""
import asyncio

import rclpy
from std_msgs.msg import String

from asyncio_for_robotics.ros2.session import auto_session
from asyncio_for_robotics.ros2.utils import TopicInfo

TOPIC1 = TopicInfo(msg_type=String, topic="example/talker1")
TOPIC2 = TopicInfo(msg_type=String, topic="example/talker2")

async def pub1_loop():
    # create the publisher safely
    with auto_session().lock() as node:
        pub = node.create_publisher(TOPIC1.msg_type, TOPIC1.topic, TOPIC1.qos)
        node.get_clock(...)
        print("Pub #1 safely created")

    try:
        count = 0
        while 1:  # This loop is not very precise and can drift
            data = f"[Hello world! timestamp: {count}s]"
            count += 1
            print(f"Pub #1 sending: {data}")
            pub.publish(String(data=data)) # sends data (lock is not necessary)
            await asyncio.sleep(1) # non-blocking sleep
    finally:
        # if error or anything occurs, publisher is cleaned up
        # This is not necessary in most application. 
        # It is mainly usefull for tests where many short lived pub/sub are created
        with auto_session().lock() as node:
            node.destroy_publisher(pub)
            print("Pub #1 safely cleaned up")

async def pub2_loop():
    """Same as pub1_loop"""
    with auto_session().lock() as node:
        pub = node.create_publisher(TOPIC2.msg_type, TOPIC2.topic, TOPIC2.qos)
        print("Pub #2 safely created")

    try:
        count = 0
        while 1:
            data = f"[Hello world! timestamp: {count}s]"
            count += 1
            print(f"Pub #2 sending: {data}")
            pub.publish(String(data=data))
            await asyncio.sleep(1)
    finally:
        with auto_session().lock() as node:
            node.destroy_publisher(pub)
            print("Pub #2 safely cleaned up")


async def main():
    while 1:
        print()
        print("Running pub #1 for 5s")
        task1 = asyncio.create_task(pub1_loop())
        await asyncio.sleep(5)
        print()
        print("Running pub #1 and #2 for 5s")
        task2 = asyncio.create_task(pub2_loop())
        await asyncio.sleep(5)
        print()
        print("stopping pub #1, keep running pub #2 for 5s")
        task1.cancel()
        await asyncio.sleep(5)
        print()
        print("stopping pub #2")
        task2.cancel()
        await asyncio.sleep(2)


if __name__ == "__main__":
    rclpy.init()
    asyncio.run(main())
    auto_session().close()
    rclpy.shutdown()

