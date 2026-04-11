import asyncio
import contextvars
import inspect
from contextlib import AsyncExitStack
from functools import wraps
from typing import Any, Optional

from ._compat import BaseExceptionGroup, TaskGroup

_MISSING = object()
_CURRENT_SCOPE: contextvars.ContextVar["Scope | None"] = contextvars.ContextVar(
    "afor_current_scope",
    default=None,
)


class ScopeBreak(Exception):
    """Exit the current afor.Scope immediately.

    Raise this inside the body of the current scope when you want to stop that
    scope now and jump to after the ``async with`` block.
    """


class _ScopeCancelled(Exception):
    """Internal exception used to stop a scope early."""

class Scope:
    """Lexical owner for afor resources and background tasks.

    Public attributes:
        task_group:
            The underlying TaskGroup used by this scope.
            It is available only while the scope is active, meaning inside the
            ``async with`` block.
        exit_stack:
            The underlying AsyncExitStack used by this scope.
            It is available only while the scope is active.
        finished:
            Future resolved once scope teardown is complete.
            It becomes available once the scope has been entered.

    Typical use:
        - normal users only create ``async with afor.Scope():``
        - active afor objects attach to the current scope automatically
        - use ``raise afor.ScopeBreak()`` to exit the current scope now
        - advanced users may directly use ``scope.task_group`` or
          ``scope.exit_stack`` for custom structured-concurrency or cleanup
          needs
    """

    def __init__(self) -> None:
        #: Underlying AsyncExitStack.
        #: It is None before ``__aenter__`` and after ``__aexit__``.
        self.exit_stack: Optional[AsyncExitStack] = None
        #: Underlying TaskGroup.
        #: It is None before ``__aenter__`` and after ``__aexit__``.
        self.task_group: Optional[TaskGroup] = None
        self._finished: Optional[asyncio.Future[None]] = None
        self._updating_finished = False
        self._current_scope_token: Optional[contextvars.Token["Scope | None"]] = None
        self._cancel_called = False

    async def __aenter__(self) -> "Scope":
        """Enter the scope and activate its TaskGroup and AsyncExitStack."""
        self._finished = asyncio.get_running_loop().create_future()
        self._finished.add_done_callback(self._finished_done_callback)
        self._finished.add_done_callback(self._finished_consume_exception_callback)
        self.exit_stack = AsyncExitStack()
        await self.exit_stack.__aenter__()
        self.task_group = await self.exit_stack.enter_async_context(TaskGroup())
        self._current_scope_token = _CURRENT_SCOPE.set(self)
        return self

    async def __aexit__(self, exc_type, exc, tb) -> bool:
        """Leave the scope, running cleanup callbacks and exiting the TaskGroup.

        ``ScopeBreak`` is treated as normal scoped control flow and is
        suppressed after teardown finishes.
        """
        assert self.exit_stack is not None
        assert self._current_scope_token is not None
        _CURRENT_SCOPE.reset(self._current_scope_token)
        try:
            suppressed = await self.exit_stack.__aexit__(exc_type, exc, tb)
            self._set_finished_result()
            return suppressed or exc_type is ScopeBreak
        except BaseException as err:
            if exc_type is ScopeBreak:
                filtered_break = self._strip_scope_break(err)
                if filtered_break is None:
                    self._set_finished_cancelled()
                    return True
                err = filtered_break
            filtered = self._strip_scope_cancel(err)
            if filtered is None:
                if exc_type is asyncio.CancelledError:
                    self._set_finished_cancelled()
                else:
                    self._set_finished_result()
                return exc_type in (None, asyncio.CancelledError)
            self._set_finished_exception(filtered)
            raise filtered
        finally:
            self.task_group = None
            self.exit_stack = None

    @classmethod
    def current(cls, default: Any = _MISSING) -> "Scope":
        """Return the current lexical afor scope.

        Args:
            default:
                Value returned when no scope is active.
                If omitted, a RuntimeError is raised instead.

        Returns:
            The currently active scope for this execution context.
        """
        scope = _CURRENT_SCOPE.get()
        if scope is None:
            if default is _MISSING:
                raise RuntimeError("No active afor.Scope")
            return default
        return scope

    @property
    def finished(self) -> asyncio.Future[None]:
        """Future resolved when scope teardown completes.

        This future is available after the scope has been entered.
        Typical uses:
        - inspect whether another scope finished cleanly
        - await teardown after requesting ``scope.cancel()`` elsewhere

        It is usually not useful to await ``finished`` from inside the same
        scope body unless some other task is responsible for ending that scope.

        States:
        - pending: scope is still running
        - result ``None``: scope completed normally
        - cancelled: scope ended through cancellation / scope-break control flow
        - exception: scope failed

        Calling ``scope.finished.cancel()`` while the scope is still running
        requests cancellation of that scope.
        """
        if self._finished is None:
            raise RuntimeError("Scope.finished is available only after entering the scope")
        return self._finished

    def cancel(self) -> None:
        """Request early stop of this scope.

        This is different from reaching the natural end of the block:
        ``cancel()`` injects an internal cancellation task into the scope's
        TaskGroup so the scope will tear down on exit.

        This is useful when you need to stop a scope that is not the current
        one, or when teardown request and control-flow exit are separate
        concerns.

        To exit the current scope immediately and continue after the block, use
        ``raise afor.ScopeBreak()`` instead.

        Multiple calls are harmless.
        """
        if self._cancel_called:
            return
        self._cancel_called = True
        assert self.task_group is not None
        self.task_group.create_task(self._cancel_scope())

    async def _cancel_scope(self) -> None:
        raise _ScopeCancelled()

    @classmethod
    def _strip_scope_cancel(cls, err: BaseException) -> BaseException | None:
        if isinstance(err, _ScopeCancelled):
            return None
        if isinstance(err, BaseExceptionGroup):
            remaining = []
            for inner in err.exceptions:
                stripped = cls._strip_scope_cancel(inner)
                if stripped is not None:
                    remaining.append(stripped)
            if not remaining:
                return None
            return err.derive(remaining)
        return err

    @classmethod
    def _strip_scope_break(cls, err: BaseException) -> BaseException | None:
        if isinstance(err, ScopeBreak):
            return None
        if isinstance(err, BaseExceptionGroup):
            remaining = []
            for inner in err.exceptions:
                stripped = cls._strip_scope_break(inner)
                if stripped is not None:
                    remaining.append(stripped)
            if not remaining:
                return None
            return err.derive(remaining)
        return err

    def _set_finished_result(self) -> None:
        if self._finished is None:
            return
        if not self._finished.done():
            self._updating_finished = True
            try:
                self._finished.set_result(None)
            finally:
                self._updating_finished = False

    def _set_finished_cancelled(self) -> None:
        if self._finished is None:
            return
        if not self._finished.done():
            self._updating_finished = True
            try:
                self._finished.cancel()
            finally:
                self._updating_finished = False

    def _set_finished_exception(self, exc: BaseException) -> None:
        if self._finished is None:
            return
        if not self._finished.done():
            self._updating_finished = True
            try:
                self._finished.set_exception(exc)
            finally:
                self._updating_finished = False

    def _finished_done_callback(self, fut: asyncio.Future[None]) -> None:
        if self._updating_finished:
            return
        if not fut.cancelled():
            return
        if self.task_group is None:
            return
        self.cancel()

    @staticmethod
    def _finished_consume_exception_callback(fut: asyncio.Future[None]) -> None:
        if fut.cancelled():
            return
        try:
            fut.exception()
        except BaseException:
            return


def scoped(func=None, *, pass_scope: bool = False):
    """Wrap an async function in ``async with afor.Scope():``.

    Usage:
        ```python
        @afor.scoped
        async def main():
            ...
        ```

        ```python
        @afor.scoped(pass_scope=True)
        async def main(scope: afor.Scope):
            ...
        ```
    """

    def decorate(fn):
        if not inspect.iscoroutinefunction(fn):
            raise TypeError("@afor.scoped requires an async function")

        @wraps(fn)
        async def wrapped(*args, **kwargs):
            async with Scope() as scope:
                if pass_scope:
                    if "scope" in kwargs:
                        raise TypeError("scope argument already provided")
                    kwargs["scope"] = scope
                return await fn(*args, **kwargs)

        return wrapped

    if func is None:
        return decorate
    return decorate(func)
