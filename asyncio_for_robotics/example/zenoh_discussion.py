"""
Verbose example showing how to integrate Zenoh with asyncio_for_robotics.

This script demonstrates:
- Initializing Zenoh objects and creating a publisher with `auto_session()`.
- Publishing data in the background while running subscriber examples.
- Call to `python_discussion.discuss` then illustrate different asyncio-based
  subscription methods (`wait_for_value`, `wait_for_new`, `wait_for_next`,
  `listen`, and `listen_reliable`). Refere to `python_discussion.py` for details
- Properly shutting down Zenoh sessions to allow clean program exit.
"""
import asyncio

import zenoh

from asyncio_for_robotics.zenoh.session import auto_session
from asyncio_for_robotics.zenoh.sub import Sub

from .python_discussion import brint, discuss


async def talking_loop():
    brint(
        f"""asyncio_for_robotics does not provide zenoh publishers, it is out
          of scope. You remain in full control of publishing. You can get the
          zenoh session to declare a publisher by using `auto_session()`"""
    )
    pub = auto_session().declare_publisher("example/discussion")
    print(f"Zenoh started publishing onto {pub.key_expr}")
    try:
        count = 0
        while 1:
            pub.put(f"[Hello world! timestamp: {count/10:.1f}s]")
            count += 1
            await asyncio.sleep(0.1)
    finally:
        # pass
        pub.undeclare()


def get_str_from_msg(msg: zenoh.Sample):
    return msg.payload.to_string()


async def main():
    background_talker_task = asyncio.create_task(talking_loop())
    sub = Sub("example/**")
    await discuss(sub, get_str_from_msg)
    background_talker_task.cancel()


if __name__ == "__main__":
    brint(
        f"""
        Zenoh requires a zenoh session runing in the background.
        asyncio_for_robotics will automatically call `auto_session` if a
        zenoh session is not provided. This auto_session is a singleton
        (one per python process).
        """
    )
    asyncio.run(main())
    print()
    brint(
        f"""
    To finish and let python exit, the zenoh session needs to be closed. We can
    retrieve and close the one automatically created with `auto_session().close()`
    """
    )
    auto_session().close()
