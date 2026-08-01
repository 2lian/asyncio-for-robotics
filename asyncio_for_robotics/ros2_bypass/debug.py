import asyncio
from contextlib import ExitStack, suppress
from typing import Any

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_default

try:
    from rclpy.subscription import MessageInfo
except:
    MessageInfo = Any
from rclpy.type_support import check_is_valid_msg_type
from std_msgs.msg import String

from ..core import BaseSub


async def display(sub: BaseSub[tuple[String, MessageInfo]]):
    print("listening start")
    async for msg, info in sub.listen():
        print(f"msg: {msg}\ninfo: {info}")


async def main():
    with ExitStack() as es:
        rclpy.init()
        es.callback(rclpy.shutdown)
        n = Node("afor_dbg")
        es.callback(n.destroy_node)

        handle = n.handle
        check_is_valid_msg_type(String)
        ros_sub = _rclpy.Subscription(
            handle,
            String,
            n.resolve_topic_name("example/talker"),
            qos_profile_default.get_c_qos_profile(),
        )
        es.callback(n.destroy_subscription, ros_sub)

        afor_sub: BaseSub[tuple[String, MessageInfo]] = BaseSub()

        def consume():
            data = ros_sub.take_message(String, False)
            while data is not None:
                afor_sub._input_data_guarded(data)
                data = ros_sub.take_message(String, False)

        loop = asyncio.get_event_loop()

        def react(*_):
            loop.call_soon_threadsafe(consume)

        ros_sub.set_on_new_message_callback(react)

        try:
            await display(afor_sub)
        finally:
            ros_sub.clear_on_new_message_callback()
            ros_sub.destroy_when_not_in_use()


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
