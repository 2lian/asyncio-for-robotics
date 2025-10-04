import asyncio
from contextlib import asynccontextmanager, suppress
from typing import Awaitable, TypeVar

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
    try:
        return await asyncio.wait_for(coro, timeout=timeout)
    except TimeoutError as e:
        return e


@asynccontextmanager
async def soft_timeout(duration: float):
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
        async with asyncio.timeout(duration):
            yield lambda: timed_out
    except asyncio.CancelledError:
        timed_out = True
