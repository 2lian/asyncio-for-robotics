import asyncio

import rclpy
from std_msgs.msg import String

from asyncio_for_robotics.ros2 import Sub, TopicInfo, auto_session

TOPIC = TopicInfo(msg_type=String, topic="example/talker")


async def main():
    sub = Sub(TOPIC.msg_type, TOPIC.topic, TOPIC.qos)
    async for message in sub.listen_reliable():
        print(f"Received: {message.data}")

if __name__ == "__main__":
    rclpy.init()
    asyncio.run(main())
    auto_session().close()
    rclpy.shutdown()
