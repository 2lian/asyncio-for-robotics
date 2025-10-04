import asyncio
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
