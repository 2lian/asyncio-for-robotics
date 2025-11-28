"""
Example ROS 2 subscriber using asyncio integration.

This example demonstrates how to:
- Subscribe to multiple ROS 2 topics concurrently with asyncio.
- React immediately to the first new message from any topic.
- Use `soft_timeout` to wait briefly for additional messages while still
  resuming as soon as all subscriptions have received data.
- Distinguish between new and previously received messages.
- Safely retrieve the most recent values even if one topic has never
  produced data.

Run this script using `python3 -m asyncio_for_robotics.example.ros2_double_listener`
Run this script side by side with `python3 -m asyncio_for_robotics.example.ros2_double_talker`
"""

import asyncio
from contextlib import suppress

import rclpy
from colorama import Fore  # colorama is included with ros so not dependency
from std_msgs.msg import String

from asyncio_for_robotics import soft_timeout
from asyncio_for_robotics.ros2 import Sub, TopicInfo, auto_session

TOPIC1 = TopicInfo(msg_type=String, topic="example/talker1")
TOPIC2 = TopicInfo(msg_type=String, topic="example/talker2")


async def main():
    sub1 = Sub(TOPIC1.msg_type, TOPIC1.topic, TOPIC1.qos)
    sub2 = Sub(TOPIC2.msg_type, TOPIC2.topic, TOPIC2.qos)
    while 1:
        task1 = asyncio.create_task(sub1.wait_for_new())
        task2 = asyncio.create_task(sub2.wait_for_new())

        # waits for a message on any of the topics
        await asyncio.wait([task1, task2], return_when=asyncio.FIRST_COMPLETED)

        # this waits for an additional 0.2s after receiving the first message
        # However it continues immediately once all messages are received!
        async with soft_timeout(0.2):
            await asyncio.wait([task1, task2], return_when=asyncio.ALL_COMPLETED)

        # gets the latest data if it exists
        # necessary because one of the topic might get a message, while the
        # other never received anything
        sub1_has_data = sub1.alive.is_set()
        sub2_has_data = sub2.alive.is_set()
        data1 = (await sub1.wait_for_value()).data if sub1_has_data else None
        data2 = (await sub2.wait_for_value()).data if sub2_has_data else None
        # adds fancy colors depending if data is new or old
        freshness1 = (
            f"{Fore.GREEN}new{Fore.RESET}"
            if task1.done()
            else f"{Fore.RED}old{Fore.RESET}"
        )
        freshness2 = (
            f"{Fore.GREEN}new{Fore.RESET}"
            if task2.done()
            else f"{Fore.RED}old{Fore.RESET}"
        )
        print(
            f"""
Latest received messages:
  - Pub #1: ({freshness1}) {data1}
  - Pub #2: ({freshness2}) {data2}
              """
        )


if __name__ == "__main__":
    rclpy.init()
    try:
        # suppress, just so we don't flood the terminal on exit
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(main())  # starts asyncio executor
    finally:
        # cleanup. `finally` statment always executes
        auto_session().close()
        rclpy.shutdown()
