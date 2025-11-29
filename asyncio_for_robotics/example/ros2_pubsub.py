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
    # creates a standard ROS 2 publisher safely
    with afor.auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    start_time = None
    
    # Async for loop, itterating for every tick of a 2Hz timer
    async for t_ns in Rate(frequency=2).listen_reliable():  
        if start_time is None:
            start_time = t_ns
        payload = f"[Hello World! timestamp: {(t_ns-start_time)/1e9}s]"
        print(f"Publishing: {payload}")
        pub.publish(String(data=payload))  # sends payload (lock not necessary)


async def hello_world_subscriber():
    # creates an afor subscriber for a ROS 2 topic
    sub = afor.Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    
    # async for loop itterating for every messages received by the sub
    async for message in sub.listen_reliable():
        print(f"Received: {message.data}")


async def hello_world_pubsub():
    # starts both tasks.
    # Notice how easy it is to compose behaviors.
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
