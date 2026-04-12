"""
Verbose example showing how to integrate ROS 2 with asyncio_for_robotics.

This script demonstrates:
- Creating a ROS 2 session with `auto_context()` and a publisher with
  `auto_session()`.
- Safely declaring publishers using the session lock to manipulate the node
  running in a background thread.
- Publishing data in the background while running subscriber examples.
- Calling `python_discussion.discuss` to illustrate asyncio-based subscription
  methods (`wait_for_value`, `wait_for_new`, `wait_for_next`, `listen`,
  and `listen_reliable`). See `python_discussion.py` for detailed behavior.
- Letting the lexical session context cleanly shut everything down on exit.
"""
import asyncio
from contextlib import suppress

from rclpy.qos import QoSProfile
from std_msgs.msg import String

import asyncio_for_robotics.ros2 as afor

from .python_discussion import brint, discuss

TOPIC = afor.TopicInfo(
    msg_type=String,
    topic="example/discussion",
    qos=QoSProfile(
        depth=100,
    ),
)


async def talking_loop():
    print()
    brint(
        f"""asyncio_for_robotics does not provide ros2 publishers, it is out
          of scope. You remain in full control of publishing. You can get the
          ros node to declare a publisher by using `auto_session()`"""
    )
    brint(
        f"""
        Manipulating the node spinning in the background thread is slightly
        different than usual. You need to enter the session.lock() context to
        safely add pub, sub, services ... to the node. This lock will block the
        node for as long as the context is active = as long as your code is
        indented in the `with session.lock() as node:` codeblock.
          """
    )
    print(
f"""Here is how the publisher of this demo is created:

with auto_session().lock() as node:
    pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
"""
            )
    with afor.auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    print(f"ROS 2 started publishing onto {pub.topic_name}")
    try:
        count = 0
        while 1:
            pub.publish(String(data=f"[Hello world! timestamp: {count/10:.1f}s]"))
            count += 1
            await asyncio.sleep(0.1)
    finally:
        with afor.auto_session().lock() as node:
            node.destroy_publisher(pub)


def get_str_from_msg(msg: String):
    return msg.data


@afor.scoped
async def main():
    tg = afor.Scope.current().task_group
    assert tg is not None
    background_talker_task = tg.create_task(talking_loop())
    try:
        await asyncio.sleep(0.001)
        print("\n#####################")
        brint(
            f"""
            From now on, all objects are initialized and the code is the same
            between ros2 and zenoh!
            """
        )
        print("#####################\n")
        sub = afor.Sub(**TOPIC.as_kwarg())
        await discuss(sub, get_str_from_msg)
    finally:
        background_talker_task.cancel()


if __name__ == "__main__":
    print()
    brint(
        f"""`afor.auto_context()` creates a ROS session for this block.
        The session owns the executor, the node, and ROS init/shutdown when
        needed.
        """
    )
    brint(
        f""" 
        The session is made of a node and executor running in a background
        thread. You can create your own session with your node and executor,
        or let asyncio_for_robotics handle it with `ros2.auto_context()`.
        """
    )
    with afor.auto_context():
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(main())
