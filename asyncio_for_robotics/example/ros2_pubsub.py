"""Implements a publisher and subscriber in the same node.

run with `python3 -m asyncio_for_robotics.example.ros2_pubsub`"""

import asyncio
from contextlib import suppress

from std_msgs.msg import String

import asyncio_for_robotics.ros2 as afor
from asyncio_for_robotics.core.utils import Rate

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


@afor.scoped
async def hello_world_pubsub():
    # starts both tasks.
    # Notice how easy it is to compose behaviors.
    tg = afor.Scope.current().task_group
    assert tg is not None
    tg.create_task(hello_world_subscriber())
    tg.create_task(hello_world_publisher())
    await asyncio.Future()


if __name__ == "__main__":
    # this will init rclpy and create us a "session" (executor + node).
    # both will be shutdown when the scope exits.
    # inside this context afor defaults to this session for calls requiring a session.
    with afor.auto_context():
        # suppress, just so we don't flood the terminal on exit
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(hello_world_pubsub())  # starts asyncio executor
