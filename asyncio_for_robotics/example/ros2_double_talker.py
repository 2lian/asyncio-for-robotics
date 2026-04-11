"""
Example ROS 2 publishers using asyncio integration.

This example demonstrates how to:
- Safely create publishers and let the current `afor.Scope` destroy them
  automatically on exit.
- Publish messages asynchronously at regular intervals with non-blocking
  `asyncio.sleep`.
- Run multiple publishers concurrently and cancel them dynamically.
- Show how scope ownership can manage non-afor ROS objects too.

The script alternates between running:
  - Publisher #1 alone.
  - Publisher #1 and Publisher #2 together.
  - Only Publisher #2.

Run this script using `python3 -m asyncio_for_robotics.example.ros2_double_talker`
Run this script side by side with `python3 -m asyncio_for_robotics.example.ros2_double_listener`
"""

import asyncio
from contextlib import suppress

from std_msgs.msg import String

import asyncio_for_robotics.ros2 as afor

TOPIC1 = afor.TopicInfo(msg_type=String, topic="example/talker1")
TOPIC2 = afor.TopicInfo(msg_type=String, topic="example/talker2")


def make_scoped_publisher(topic: afor.TopicInfo[type[String]], name: str):
    """Create a ROS publisher and register its teardown on the current scope.

    This is a good pattern when afor owns the lifetime of your task.
    """
    scope = afor.Scope.current()
    assert scope.exit_stack is not None

    with afor.auto_session().lock() as node:
        pub = node.create_publisher(topic.msg_type, topic.topic, topic.qos)
        print(f"{name} created")

    def cleanup() -> None:
        with afor.auto_session().lock() as node:
            node.destroy_publisher(pub)
            print(f"{name} cleaned up")

    scope.exit_stack.callback(cleanup)
    return pub


@afor.scoped
async def pub1_loop():
    # because of @afor.scope and make_scoped_publisher, our publisher will be
    # automatically destroyed once this function returns 
    pub = make_scoped_publisher(TOPIC1, "Pub #1")
    count = 0
    while 1:  # This loop is not very precise and can drift
        data = f"[Hello world! timestamp: {count}s]"
        count += 1
        print(f"Pub #1 sending: {data}")
        pub.publish(String(data=data))  # sends data (lock is not necessary)
        await asyncio.sleep(1)  # non-blocking sleep


@afor.scoped
async def pub2_loop():
    """Same as pub1_loop"""
    pub = make_scoped_publisher(TOPIC2, "Pub #2")
    count = 0
    while 1:
        data = f"[Hello world! timestamp: {count}s]"
        count += 1
        print(f"Pub #2 sending: {data}")
        pub.publish(String(data=data))
        await asyncio.sleep(1)


@afor.scoped
async def main():
    # this is the asyncio.TaskGroup attached to our current afor.Scope
    # using @afor.scoped , this Scope applies to all code in this function
    tg = afor.Scope.current().task_group
    assert tg is not None
    while 1:
        print()
        print("Running pub #1 for 5s")
        task1 = tg.create_task(pub1_loop())
        await asyncio.sleep(5)
        print()
        print("Running pub #1 and #2 for 5s")
        task2 = tg.create_task(pub2_loop())
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
    with afor.auto_context():
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(main())
