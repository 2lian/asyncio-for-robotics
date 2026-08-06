import asyncio
import logging
from asyncio import AbstractEventLoop, Future
from contextlib import suppress
from typing import Optional

from rclpy.task import Future as RosFuture

logger = logging.getLogger(__name__)


def try_cancel_future(future: Future) -> None:
    """Cancel *future* if it has not already completed."""
    with suppress(asyncio.InvalidStateError):
        future.cancel()


def try_set_future_exception(future: Future, exc: BaseException) -> None:
    """Set *future* exception, ignoring late completions."""
    with suppress(asyncio.InvalidStateError):
        future.set_exception(exc)


def try_set_future_result(future: Future, result: object) -> None:
    """Set *future* result, ignoring late completions."""
    with suppress(asyncio.InvalidStateError):
        future.set_result(result)


def asyncify_future(
    ros_future: RosFuture,
    event_loop: Optional[AbstractEventLoop] = None,
) -> Future:
    """
    Convert a ROS Future into an asyncio Future.

    The asyncio Future will complete when the ROS Future completes,
    propagating its result, cancellation or exception.

    If the asyncio Future is already done (e.g. cancelled by asyncio.wait_for),
    the ROS callback is silently ignored.

    Args:
        ros_future: ROS Future.
        event_loop: Asyncio event loop to use. If None, uses asyncio.get_event_loop().

    Returns:
        Asyncio Future reflecting the ROS Future.
    """
    ao_future: Future = Future()
    if event_loop is None:
        event_loop = asyncio.get_event_loop()

    def ros_cbk(fut: RosFuture) -> None:
        if fut.cancelled():
            event_loop.call_soon_threadsafe(try_cancel_future, ao_future)
            return
        if fut.done():
            exc = fut.exception()
            if exc is not None:
                event_loop.call_soon_threadsafe(
                    try_set_future_exception, ao_future, exc
                )
            else:
                res = fut.result()  # type: ignore
                event_loop.call_soon_threadsafe(try_set_future_result, ao_future, res)

    # lock not necessary, ros seems safe
    ros_future.add_done_callback(ros_cbk)
    return ao_future
