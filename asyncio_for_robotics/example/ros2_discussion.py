"""
Verbose example showing how to integrate ROS 2 with asyncio_for_robotics.

This script demonstrates:
- Initializing ROS 2 nodes and creating a publisher with `auto_session()`.
- Safely declaring publishers using the session lock to manipulate the node
  running in a background thread.
- Publishing data in the background while running subscriber examples.
- Calling `python_discussion.discuss` to illustrate asyncio-based subscription
  methods (`wait_for_value`, `wait_for_new`, `wait_for_next`, `listen`,
  and `listen_reliable`). See `python_discussion.py` for detailed behavior.
- Properly shutting down both the ROS 2 session and `rclpy` to allow clean
  program exit.
"""
import asyncio
import rclpy

from rclpy.qos import QoSProfile
from std_msgs.msg import String

from asyncio_for_robotics.ros2.session import auto_session
from asyncio_for_robotics.ros2.sub import Sub
from asyncio_for_robotics.ros2.utils import TopicInfo

from .python_discussion import brint, discuss

TOPIC = TopicInfo(
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
    with auto_session().lock() as node:
        pub = node.create_publisher(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    print(f"Zenoh started publishing onto {pub.topic_name}")
    try:
        count = 0
        while 1:
            pub.publish(String(data=f"[Hello world! timestamp: {count/10:.1f}s]"))
            count += 1
            await asyncio.sleep(0.1)
    finally:
        with auto_session().lock() as node:
            node.destroy_publisher(pub)


def get_str_from_msg(msg: String):
    return msg.data


async def main():
    background_talker_task = asyncio.create_task(talking_loop())
    await asyncio.sleep(0.001)
    print("\n#####################")
    brint(
        f"""
        From now on, all objects are initialized and the code is the same
        between ros2 and zenoh!
        """
    )
    print("#####################\n")
    sub = Sub(**TOPIC.as_kwarg())
    await discuss(sub, get_str_from_msg)
    background_talker_task.cancel()


if __name__ == "__main__":
    print()
    brint(
        f""" ROS requires `rclpy.init` to be called once per python process
        (then shutdown). Unlike the session, asyncio_for_robotics will
        not call `rclpy.init` automatically.
        """
    )
    brint(
        f""" 
        The concept of session is however also present in
        asyncio_for_robotics.ros2! The session is made of a node and executor
        running in a background thread. You can create your own session
        with your node and your executor, or let asyncio_for_robotics automatically
        handle it with `ros2.auto_session()`. `ros2.auto_session()` will be called automatically every time a session is not provided as argument.
        """
    )
    rclpy.init()
    asyncio.run(main())
    print()
    brint(
        f"""
    To finish and let python exit, the session and rclpy needs to be closed. We can
    retrieve and close the one automatically created with `auto_session().close()`
    """
    )
    auto_session().close()
    rclpy.shutdown()
