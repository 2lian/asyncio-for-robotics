"""
Verbose example showing how to integrate Zenoh with asyncio_for_robotics.

This script demonstrates:
- Initializing Zenoh objects and creating a publisher with `auto_session()`.
- Publishing data in the background while running subscriber examples.
- Call to `python_discussion.discuss` then illustrate different asyncio-based
  subscription methods (`wait_for_value`, `wait_for_new`, `wait_for_next`,
  `listen`, and `listen_reliable`). Refere to `python_discussion.py` for details
- Letting the lexical session context cleanly shut the session down on exit.
"""
import asyncio
from contextlib import suppress

import zenoh

from asyncio_for_robotics.core._logger import setup_logger
import asyncio_for_robotics.zenoh as afor

from .python_discussion import brint, discuss
setup_logger("./")


async def talking_loop():
    brint(
        f"""asyncio_for_robotics does not provide zenoh publishers, it is out
          of scope. You remain in full control of publishing. You can get the
          zenoh session to declare a publisher by using `auto_session()`"""
    )
    pub = afor.auto_session().declare_publisher("example/discussion")
    print(f"Zenoh started publishing onto {pub.key_expr}")
    try:
        count = 0
        while 1:
            pub.put(f"[Hello world! timestamp: {count/10:.1f}s]")
            count += 1
            await asyncio.sleep(0.1)
    finally:
        print(f"Zenoh stopped publishing onto {pub.key_expr}")
        pub.undeclare()


def get_str_from_msg(msg: zenoh.Sample):
    return msg.payload.to_string()


@afor.scoped
async def main():
    tg = afor.Scope.current().task_group
    assert tg is not None
    background_talker_task = tg.create_task(talking_loop())
    try:
        sub = afor.Sub("example/**")
        await discuss(sub, get_str_from_msg)
    finally:
        background_talker_task.cancel()


if __name__ == "__main__":
    brint(
        f"""
        `afor.auto_context()` creates a Zenoh session for this block. Inside
        the block, afor objects use this session by default.
        """
    )
    with afor.auto_context():
        with suppress(KeyboardInterrupt, asyncio.CancelledError, zenoh.ZError):
            asyncio.run(main())
