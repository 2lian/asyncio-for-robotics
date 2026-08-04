"""Listen to the Jazzy example talker through the asyncio ROS session.

Run with::

    python -m asyncio_for_robotics.ros2_bypass.example
"""

import asyncio
import time
from contextlib import suppress

from std_msgs.msg import String

import asyncio_for_robotics.ros2_bypass as afor


@afor.scoped
async def main() -> None:
    """Print messages received from ``example/talker``."""
    async with afor.async_context("afor_asyncio_executor_example"):
        sub = afor.Sub(String, "example/talker", 10)
        print("Listening on example/talker")
        async for msg in sub.listen_reliable():
            print(f"t={time.time()}  I heard: {msg.data}")


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
