import asyncio
import logging
import time
from contextlib import asynccontextmanager, suppress
from typing import Any, Awaitable, Callable, Coroutine, TypeVar

from asyncio_for_robotics.core.scope import AUTO_SCOPE, Scope
from asyncio_for_robotics.core.sub import BaseSub

try:
    from asyncio import timeout as john_timeout
except ImportError:
    from async_timeout import timeout as john_timeout

logger = logging.getLogger(__name__)

_T = TypeVar("_T")


async def soft_wait_for(coro: Awaitable[_T], timeout: float) -> _T | TimeoutError:
    """Await *coro* with a timeout — returns ``TimeoutError`` instead of raising.

    Useful for try-or-skip patterns where a timeout is an expected outcome,
    not an error::

        result = await soft_wait_for(sub.wait_for_new(), 2.0)
        if isinstance(result, TimeoutError):
            print("no message within 2 s")

    Args:
        coro: Awaitable to run.
        timeout: Maximum wait in seconds.

    Returns:
        The awaitable's result, or a ``TimeoutError`` instance on expiry.
    """

    timer = asyncio.create_task(asyncio.sleep(timeout))
    task = asyncio.ensure_future(coro)

    await asyncio.wait([timer, task], return_when=asyncio.FIRST_COMPLETED)

    if task.done():
        return task.result()
    else:
        task.cancel()
        return TimeoutError("afor.soft_wait_for")


@asynccontextmanager
async def soft_timeout(timeout: float):
    """Async context manager: cancel the block on expiry, no exception outside.

    Inside the block, awaited operations may be cancelled.  After the block,
    execution continues normally regardless of whether the timeout fired::

        async with soft_timeout(1.0):
            await asyncio.sleep(10)   # cancelled after 1 s

        print("continues here either way")

    Args:
        timeout: Maximum time in seconds.
    """
    timed_out = False
    try:
        async with john_timeout(timeout):
            yield lambda: timed_out
    except asyncio.CancelledError:
        timed_out = True
    except asyncio.TimeoutError:
        timed_out = True


class Rate(BaseSub[int]):
    """Drift-free periodic timer, usable like any ``BaseSub``.

    Each tick yields the *scheduled* time in nanoseconds, so you can detect
    jitter by comparing against the actual wall clock.

    Usage patterns:

    - ``async for _ in rate.listen():`` — run at the rate, skip if behind.
    - ``async for _ in rate.listen_reliable():`` — run every tick, queue if behind.
    - ``await rate.wait_for_new()`` — wait for the next tick.

    Example::

        rate = afor.Rate(10)  # 10 Hz
        async for scheduled_ns in rate.listen():
            do_work()
    """

    def __init__(
        self,
        frequency: float,
        time_source: Callable[[], int] = time.time_ns,
        scope: Scope | None = AUTO_SCOPE,
    ) -> None:
        """Create a rate timer.

        Args:
            frequency: Tick frequency in Hz.
            time_source: Callable returning the current time in nanoseconds.
            scope: ``afor.Scope`` to attach to.
        """
        self.period: int= int(1e9 / frequency)
        super().__init__(scope=scope)
        self.time_source: Callable[[], int] = time_source
        self.periodic_task: asyncio.Task=self._initialize_task()

    def _initialize_task(self):
        async def periodic_coro():
            start_time = self.time_source()
            count = 0
            while 1:
                count += 1
                scheduled_time = start_time + count * (self.period)
                dt = scheduled_time - self.time_source()
                await asyncio.sleep(max(0, dt / 1e9))
                self._periodic_cbk(scheduled_time)

        return asyncio.create_task(periodic_coro())

    def _periodic_cbk(self, scheduled_time):
        self.input_data(scheduled_time)

    @property
    def name(self):
        return f"timer_{1e-9/self.period:.3f}Hz"

    def close(self):
        self.periodic_task.cancel()
        super().close()
