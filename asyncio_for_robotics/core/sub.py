import asyncio
import concurrent.futures
import logging
from asyncio.queues import Queue
from contextlib import suppress
from typing import (
    Any,
    AsyncGenerator,
    Callable,
    Coroutine,
    Generic,
    List,
    Optional,
    Set,
    TypeVar,
)

from .scope import Scope

logger = logging.getLogger(__name__)


class SubClosedException(RuntimeError):
    """Raised when awaiting data from a closed subscriber."""


_MsgType = TypeVar("_MsgType")
_T = TypeVar("_T")
_AUTO_SCOPE = object()


class BaseSub(Generic[_MsgType]):
    """Transport-agnostic async subscriber.

    Feed data in with ``input_data()`` (thread-safe) and consume it with
    the async methods:

    - ``listen()`` : latest-value iteration (may skip messages).
    - ``listen_reliable()`` : queued iteration (no skips).
    - ``wait_for_value()`` : latest message, or wait for the first one.
    - ``wait_for_new()`` : next message after the call.
    - ``wait_for_next()`` : exactly the next received message.
    - ``asap_callback.append()`` : adds a callback (bypassing the event loop).

    Auto-attaches to the current ``afor.Scope`` for cleanup.
    """

    def __init__(
        self,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ) -> None:
        """Create a subscriber, optionally attached to *scope*.

        Args:
            scope: ``afor.Scope`` to attach to.  ``_AUTO_SCOPE`` (default)
                picks the current scope if any.  ``None`` opts out.
        """
        #: Blocking callbacks called on message arrival
        self.asap_callback: List[Callable[[_MsgType], Any]] = []
        #: Event triggering when the first message is received
        self.alive: asyncio.Event = asyncio.Event()
        #: queues associated with the generators
        self._dyncamic_queues: Set[Queue[_MsgType]] = set()

        self._event_loop = self._get_loop()
        #: Value is available event
        self._value_flag: asyncio.Event = asyncio.Event()
        #: Lastest message value
        self._value: Optional[_MsgType] = None
        #: Indicates if self.close has been called.
        #: This is different from self.lifetime as lifetime can be done with or
        #: without close being called yet.
        self._closed = asyncio.Event()
        #: Asyncio-facing lifetime future for this subscriber.
        #:
        #: This is the main finished-state primitive for ``BaseSub``.
        #: Awaiting it tells you when the subscriber has ended.
        #:
        #: Semantics:
        #: - pending: subscriber is still alive
        #: - result ``True``: subscriber closed normally
        #: - exception: subscriber failed
        #:
        #: This is a normal asyncio future owned by this event loop. Scopes
        #: watch it to propagate subscriber failures through their TaskGroup.
        self.lifetime: asyncio.Future[bool] = self._event_loop.create_future()
        self._lifetime_threadsafe = concurrent.futures.Future()
        self.lifetime.add_done_callback(lambda *_: self.close())
        self._scope: Scope | None = None
        if scope is _AUTO_SCOPE:
            scope = Scope.current(default=None)
        if scope is not None:
            self.attach(scope)
        logger.debug("created sub %s", self.name)

    @property
    def name(self) -> str:
        """The friendly name of you subscriber"""
        return "no_name"

    def attach(self, scope: Scope) -> None:
        """Attach this subscriber to an active scope.

        This is advanced API.
        Most users should let subscribers auto-attach to the current scope.

        Attaching a subscriber does two things:
        - registers ``self.close`` on ``scope.exit_stack``
        - creates one watcher task on ``scope.task_group`` so subscriber
          lifetime failures propagate as structured task-group failures

        Args:
            scope: Active afor scope that will own this subscriber.
        """
        if self._scope is not None:
            raise RuntimeError(f"Subscriber '{self.name}' already attached to a scope")
        self._scope = scope
        assert scope.exit_stack is not None
        assert scope.task_group is not None
        scope.exit_stack.callback(self.close)
        scope.task_group.create_task(self._watch_lifetime())

    async def _watch_lifetime(self) -> bool:
        return await asyncio.shield(self.lifetime)

    def _input_data_guarded(self, data: _MsgType):
        """Feed input with Exception handling and propagation.

        If an erros accours, the exception is propagated to ``self.lifetime``
        so scopes and this sub shutsdown.
        """
        if self.lifetime.done():
            return
        try:
            self._input_data_asyncio(data)
        except Exception as e:
            self._set_lifetime_exception(e)

    def input_data(self, data: _MsgType) -> bool:
        """Feed a message into this subscriber.  **Thread-safe.**

        Call this from any thread (e.g. a Zenoh callback).  The message is
        dispatched to the asyncio event loop via ``call_soon_threadsafe``.

        Args:
            data: Message to deliver to listeners.

        Returns:
            ``False`` if the event loop is closed or the subscriber has
            already finished.
        """
        if self._lifetime_threadsafe.done():
            return False
        if self._event_loop.is_closed():
            logger.info("Event loop closed, for sub: %s", self.name)
            return False
        try:
            self._event_loop.call_soon_threadsafe(self._input_data_guarded, data)
        except Exception as exc:
            self._event_loop.call_soon_threadsafe(self._set_lifetime_exception, exc)
            return False
        return True

    async def wait_for_value(self) -> _MsgType:
        """Return the latest message, waiting if there is none."""
        await self._value_flag.wait()
        assert self._value is not None
        return self._value

    def wait_for_new(self) -> Coroutine[Any, Any, _MsgType]:
        """Await a message newer than the one at call time.

        Listening starts *immediately* when this method is called, not when
        the returned coroutine is awaited.
        """
        iterato = self.listen(fresh=True)

        async def func() -> _MsgType:
            async for payload in iterato:
                return payload

        return func()

    def wait_for_next(self) -> Coroutine[Any, Any, _MsgType]:
        """Await exactly the next received message (no skips).

        Listening starts *immediately* when this method is called, not when
        the returned coroutine is awaited.
        """
        q: asyncio.Queue[_MsgType] = asyncio.LifoQueue(maxsize=2)
        self._dyncamic_queues.add(q)

        async def func() -> _MsgType:
            try:
                val_top = await q.get()
                if q.empty():
                    val_deep = val_top
                else:
                    val_deep = q.get_nowait()
                return val_deep
            finally:
                self._dyncamic_queues.discard(q)

        return func()

    def listen(self, fresh=False) -> AsyncGenerator[_MsgType, None]:
        """Iterate over the latest message (queue size 1, may skip).

        Equivalent to ``listen_reliable(fresh, queue_size=1)``.  Good for
        sensor-style consumers that only care about the most recent value.

        Listening starts *immediately* when this method is called, not when
        the returned generator is awaited.

        Args:
            fresh: If ``False``, the first yield may be the current value.
        """
        return self.listen_reliable(fresh, 1)

    def listen_reliable(
        self,
        fresh=False,
        queue_size: int = 10,
        lifo=False,
        exit_on_close: bool = False,
    ) -> AsyncGenerator[_MsgType, None]:
        """Iterate over every message without skipping (queued).

        Good for command/control consumers where every message matters.
        The internal queue buffers up to *queue_size* messages; if the queue
        is full, the oldest message is dropped and a warning is logged.

        Listening starts *immediately* when this method is called, not when
        the returned generator is awaited.

        Args:
            fresh: If ``False``, the first yield may be the current value.
            queue_size: Maximum queued messages.  ``0`` means unbounded.
            lifo: Use last-in-first-out order instead of FIFO.
            exit_on_close: If ``True``, the ``async for`` loop exits cleanly
                when ``.close()`` is called.  If ``False``,
                ``SubClosedException`` is raised instead.
        """
        if not lifo:
            q: asyncio.Queue[_MsgType] = asyncio.Queue(maxsize=queue_size)
        else:
            q: asyncio.Queue[_MsgType] = asyncio.LifoQueue(maxsize=queue_size)
        self._dyncamic_queues.add(q)
        logger.debug("Reliable listener primed %s", self.name)
        if self._value_flag.is_set() and not fresh:
            assert self._value is not None, "impossible if flag set"
            q.put_nowait(self._value)
        return self._unprimed_listen_reliable(q, exit_on_close)

    async def _unprimed_listen_reliable(
        self, queue: asyncio.Queue, exit_on_close: bool = False
    ) -> AsyncGenerator[_MsgType, None]:
        logger.debug("Reliable listener first iter %s", self.name)
        try:
            while True:
                msg = await queue.get()
                yield msg
        finally:
            self._dyncamic_queues.discard(queue)
            logger.debug("Reliable listener closed %s", self.name)

    def _input_data_asyncio(self, msg: _MsgType):
        """Processes incomming data.

        Is only safe to run on the same thread as asyncio.
        If error is raised by this call (typically an error in asap_callback,
                                         the sub will not stop.)

        Args:
            msg: message
        """
        # logger.debug("Input message %s", self.name)
        for f in self.asap_callback:
            f(msg)
        for q in self._dyncamic_queues:
            if q.full():
                if q.qsize() > 2:
                    logger.warning("Queue full (%s) on %s", q.qsize(), self.name)
                q.get_nowait()
            q.put_nowait(msg)
        self._value = msg
        self._value_flag.set()
        if not self.alive.is_set():
            logger.debug("%s is receiving data", self.name)
            self.alive.set()

    def close(self):
        """Close this subscriber.  Idempotent.

        All ``listen*`` loops exit (or raise ``SubClosedException``), all
        ``wait_for_*`` coroutines receive a close sentinel, and ``lifetime``
        resolves.
        """
        if self._closed.is_set():
            return
        logger.debug("%s closed", self.name)
        self._closed.set()
        self._set_lifetime_result(True)

    def _set_lifetime_result(self, result: bool) -> None:
        if not self.lifetime.done():
            self.lifetime.set_result(result)
        if not self._lifetime_threadsafe.done():
            self._lifetime_threadsafe.set_result(result)

    def _set_lifetime_exception(self, exc: BaseException) -> None:
        if not self.lifetime.done():
            self.lifetime.set_exception(exc)
        if not self._lifetime_threadsafe.done():
            self._lifetime_threadsafe.set_exception(exc)

    @staticmethod
    def _get_loop() -> asyncio.AbstractEventLoop:
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            return asyncio.get_event_loop()


_InType = TypeVar("_InType")
_OutType = TypeVar("_OutType")


class ConverterSub(BaseSub[_OutType], Generic[_OutType, _InType]):
    """Subscriber that transforms messages from an upstream ``BaseSub``.

    Conversion runs inline in the upstream callback path (no extra task or
    buffer).  Errors fail *this* subscriber's lifetime, not the upstream one.

    Example::

        raw_sub = Sub(bytes_type, "topic")
        decoded = ConverterSub(raw_sub, lambda b: b.decode("utf-8"))
        async for text in decoded.listen_reliable():
            print(text)
    """

    def __init__(
        self,
        sub: BaseSub[_InType],
        convert_func: Callable[[_InType], _OutType] = lambda x: x,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ) -> None:
        """Create a converter wrapping *sub*.

        Args:
            sub: Upstream subscriber producing input values.
            convert_func: Function applied to each incoming message.
                Identity by default.
            scope: ``afor.Scope`` to attach to.
        """
        #: Upstream subscriber
        self.sub: BaseSub[_InType] = sub
        #: Transformation applied to each received message
        self.convert_func: Callable[[_InType], _OutType] = convert_func
        super().__init__(scope=scope)
        self.sub.asap_callback.append(self._converter_forwarder)

    def _converter_forwarder(self, msg: _InType):
        if self.lifetime.done():
            return
        try:
            new = self.convert_func(msg)
            self._input_data_asyncio(new)
        except Exception as e:
            self._set_lifetime_exception(e)

    def close(self):
        if self._closed.is_set():
            return
        with suppress(ValueError):
            self.sub.asap_callback.remove(self._converter_forwarder)
        super().close()
