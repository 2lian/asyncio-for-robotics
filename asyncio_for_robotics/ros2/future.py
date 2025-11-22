import asyncio
import logging
from asyncio import AbstractEventLoop, Future
from typing import Optional

from rclpy.task import Future as RosFuture

logger = logging.getLogger(__name__)


def asyncify_future(
    ros_future: RosFuture,
    event_loop: Optional[AbstractEventLoop] = None,
) -> Future:
    """
    Convert a ROS Future into an asyncio Future.

    The asyncio Future will complete when the ROS Future completes,
    propagating its result, cancellation or exception.

    Args:
        ros_future: ROS Future.
        event_loop: Asyncio event loop to use. If None, uses asyncio.get_event_loop().

    Returns:
        Asyncio Future reflecting the ROS Future.
    """
    ao_future: Future = Future()
    if event_loop == None:
        event_loop = asyncio.get_event_loop()

    def ros_cbk(fut: RosFuture):
        nonlocal ao_future
        if fut.cancelled():
            event_loop.call_soon_threadsafe(ao_future.cancel)
            return
        if fut.done():
            exc = fut.exception()
            if exc is not None:
                event_loop.call_soon_threadsafe(ao_future.set_exception, exc)
            else:
                res = fut.result()  # type: ignore
                event_loop.call_soon_threadsafe(ao_future.set_result, res)

    # lock not necessary, ros seems safe
    ros_future.add_done_callback(ros_cbk)
    return ao_future
