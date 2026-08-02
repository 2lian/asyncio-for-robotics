import asyncio
from contextlib import ExitStack, suppress

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from rclpy.type_support import check_is_valid_msg_type
from std_msgs.msg import String


async def consume(ros_sub: _rclpy.Subscription, queue: asyncio.Queue):
    msg, info = ros_sub.take_message(String, False)
    while msg is not None:
        await queue.put(msg)
        msg, info = ros_sub.take_message(String, False)


async def main():
    with ExitStack() as es:
        rclpy.init()
        es.callback(rclpy.shutdown)
        n = Node("afor_dbg")
        es.callback(n.destroy_node)
        check_is_valid_msg_type(String)

        ros_sub = _rclpy.Subscription(
            n.handle,
            String,
            n.resolve_topic_name("example/talker"),
            qos_profile_default.get_c_qos_profile(),
        )
        es.callback(n.destroy_subscription, ros_sub)
        queue: asyncio.Queue[tuple[String]] = asyncio.Queue()
        loop = asyncio.get_event_loop()

        def react(*_):
            asyncio.run_coroutine_threadsafe(consume(ros_sub, queue), loop)

        ros_sub.set_on_new_message_callback(react)
        es.callback(ros_sub.clear_on_new_message_callback)

        while 1:
            msg = await queue.get()
            print(msg)


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
