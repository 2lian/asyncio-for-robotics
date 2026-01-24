"""
Example showing how to use asyncio_for_robotics subscriptions with ROS 2
subscription event callbacks (e.g. matched publishers, lost messages).

afor's generic `BaseSub` is used to accumulate a datastream not natively
supported by `afor`. The external ROS object calls `BaseSub.input_data` which
is threadsafe

Run:
    python3 -m asyncio_for_robotics.example.ros2_event_callback

In another terminal:
    ros2 topic pub /example std_msgs/msg/String "data: hey"

Expected output:
    Match Event:
    New publisher
    {'current_count': 1,
     'current_count_change': 1,
     'total_count': 1,
     'total_count_change': 1}

    I heard: hey
    ...
    I heard: hey

    Match Event:
    Undeclared publisher
    {'current_count': 0,
     'current_count_change': -1,
     'total_count': 1,
     'total_count_change': 0}
"""

import asyncio
import inspect
from contextlib import suppress
from pprint import pprint
from typing import Optional

import rclpy
from colorama import Fore
from rclpy.event_handler import (
    IncompatibleTypeInfo,
    QoSLivelinessChangedInfo,
    QoSMessageLostInfo,
    QoSRequestedDeadlineMissedInfo,
    QoSRequestedIncompatibleQoSInfo,
    QoSSubscriptionMatchedInfo,
    SubscriptionEventCallbacks,
)
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription
from std_msgs.msg import String

from asyncio_for_robotics.core.sub import BaseSub, _MsgType
from asyncio_for_robotics.ros2 import (
    QOS_DEFAULT,
    BaseSession,
    Sub,
    TopicInfo,
    auto_session,
)

TOPIC = TopicInfo(
    "example",
    String,
)


class AdvancedSub(Sub[_MsgType]):
    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: QoSProfile = QOS_DEFAULT,
        session: Optional[BaseSession] = None,
        event_callbacks: Optional[SubscriptionEventCallbacks] = None,
    ) -> None:
        """
        Extension of `Sub` that allows passing ROS 2
        `SubscriptionEventCallbacks` when creating the underlying rclpy
        subscription.
        """
        self.event_callbacks = event_callbacks
        super().__init__(msg_type, topic, qos_profile, session)

    def _resolve_sub(self, topic_info: TopicInfo) -> Subscription:
        """
        Create the underlying rclpy Subscription with event callbacks attached.
        """
        with self.session.lock() as node:
            return node.create_subscription(
                **topic_info.as_kwarg(),
                callback=self.callback_for_sub,
                event_callbacks=self.event_callbacks,
            )


async def verbose_match_event(match_sub: BaseSub[QoSSubscriptionMatchedInfo]):
    """Listens to match/unmatch events and print their full contents."""
    async for event in match_sub.listen_reliable():
        event: QoSSubscriptionMatchedInfo
        print("")
        print(f"{Fore.RED}Match Event triggered:{Fore.RESET}")
        if event.current_count_change > 0:
            print(f"{Fore.GREEN}Match publisher{Fore.RESET}")
        if event.current_count_change < 0:
            print(f"{Fore.YELLOW}Unmatch publisher{Fore.RESET}")
        # pprint(dir(event))
        pprint(
            {
                name: value
                for name, value in inspect.getmembers(event)
                if not name.startswith("_")
            }
        )
        print("")


async def what_do_you_hear(sub: Sub[String]):
    """Simply print the payloads of a topic."""
    async for msg in sub.listen_reliable():
        print(f"I heard: {msg.data}")


async def event_example():
    """
    - Creates `BaseSub` instances to receive ROS events in afor.
    - Registers them as ROS 2 subscription event callbacks.
    - Runs a simple String subscription.
    """
    # afor's generic `BaseSub` is used to accumulate a generic datastream. The
    # event callback calls `BaseSub.input_data` which is threadsafe
    lost_sub: BaseSub[QoSMessageLostInfo] = BaseSub()
    match_sub: BaseSub[QoSSubscriptionMatchedInfo] = BaseSub()
    event_cbk = SubscriptionEventCallbacks(
        message_lost=lost_sub.input_data, matched=match_sub.input_data  # type: ignore
    )
    sub = AdvancedSub(
        **TOPIC.as_kwarg(),
        event_callbacks=event_cbk,
    )
    hear_task = asyncio.create_task(what_do_you_hear(sub))
    match_task = asyncio.create_task(verbose_match_event(match_sub))
    print(
        f"""Start/stop publishing using 
        `ros2 topic pub /{sub.topic_info.topic} std_msgs/msg/String "data: hey"`"""
    )
    await asyncio.wait([hear_task, match_task])


if __name__ == "__main__":
    rclpy.init()
    try:
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(event_example())
    finally:
        auto_session().close()
        rclpy.shutdown()
