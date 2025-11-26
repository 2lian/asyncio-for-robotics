import asyncio
import logging
import time
from contextlib import asynccontextmanager, suppress
from typing import Any, Awaitable, Callable, Coroutine, TypeVar

from asyncio_for_robotics.core.sub import BaseSub

try:
    from asyncio import timeout as john_timeout
except ImportError:
    from async_timeout import timeout as john_timeout

logger = logging.getLogger(__name__)

_T = TypeVar("_T")


async def soft_wait_for(coro: Awaitable[_T], timeout: float) -> _T | TimeoutError:
    """Awaits a coroutine with a timeout,
    if timeout occurs returns TimeoutError but does not raise it.

    Args:
        coro: coroutine to await
        timeout: timeout in seconds

    Returns:
        coroutine result or TimeoutError
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
    """
    Run an async block with a time limit, cancelling it on expiry.

    - Inside the block: awaited operations may be cancelled.
    - Outside the block: no exception propagates.

    Example:
        async with soft_timeout(1):
            await asyncio.sleep(2)  # interrupted after 1 second

        # Execution resumes here without raising CancelledError.

    Args:
        duration: Maximum time in seconds to allow the block to run.
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
    def __init__(
        self, frequency: float, time_source: Callable[[], int] = time.time_ns
    ) -> None:
        """Provides a reliable, no drift rate, usable like an afor subscriber.

        Data returned is the time (in ns) at which the rate was suppose to fire.

        Posibilities:
            - wait_for_value returns the last tic time (ns)
            - wait_for_next and wait_for_new will wait for the next tic.
            - listen() executes every tic. However, if there is a queue, it
              skips and executes the latest.
            - listen_reliable() executes every tic and never misses any (queuing).

        Args:
            frequency: Frequency in Hz
            time_source: Source of time in ns
        """
        self.period: int= int(1e9 / frequency)
        super().__init__()
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
