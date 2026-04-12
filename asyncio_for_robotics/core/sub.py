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
_CLOSE_SENTINEL = object()


class SubClosedException(RuntimeError):
    """Raised when awaiting data from a closed subscriber."""


_MsgType = TypeVar("_MsgType")
_T = TypeVar("_T")
_AUTO_SCOPE = object()


class BaseSub(Generic[_MsgType]):
    def __init__(
        self,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ) -> None:
        """Abstract (non-specific) asyncio_for_robotics subscriber.

        This defines the different methods to asynchronously get new messages.

        Use ``self.intput_data(your_data)`` to input new messages on this subscriber.

        This object is pure python and not specialize for any transport
        protocol. See asyncio_for_robotics.zenoh.sub for an easy
        implementation.

        To implements your own sheduling methods (new type of queue, buffer,
        genrator ...), please either:

            - inherit from this class, then overide self._input_data_asyncio
            - put a callback inside self.asap_callback
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
        """Feed input unless this subscriber has already finished.

        If ``_input_data_asyncio()`` raises, the exception is stored on
        ``self.lifetime`` so scopes and other owners can observe the failure.
        """
        if self.lifetime.done():
            return
        try:
            self._input_data_asyncio(data)
        except Exception as e:
            self._set_lifetime_exception(e)

    def input_data(self, data: _MsgType) -> bool:
        """Processes incomming data.
        Call this in your subscriber callback.

        .. Note:
            This is threadsafe, thus can run safely on any thread.

        Args:
            data: Data to input on this sub

        Returns:
            False if the even loop has been closed or there is another critical
                problem making this sub unable to work.
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
        """Latest message.

        Returns:
            The latest message (if none, awaits the first message)
        """
        await self._value_flag.wait()
        assert self._value is not None
        return self._value

    def wait_for_new(self) -> Coroutine[Any, Any, _MsgType]:
        """Awaits a new value.

        .. Note:
            "Listening" starts the moment this function is called, not when
            the coroutine is awaited

        See wait_for_next to be sure the data is exactly the next received.

        Returns:
            Coroutine holding a message more recent than the one at time of the call
        """
        iterato = self.listen(fresh=True)

        async def func() -> _MsgType:
            async for payload in iterato:
                return payload

        return func()

    def wait_for_next(self) -> Coroutine[Any, Any, _MsgType]:
        """Awaits exactly the next value.

        .. Note:
            "Listening" starts the moment this function is called, not when
            the coroutine is awaited

        Returns:
            Coroutine holding the first message received after the call.
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
                if val_deep is _CLOSE_SENTINEL:
                    raise SubClosedException(f"Subscriber '{self.name}' was closed")
                return val_deep
            finally:
                self._dyncamic_queues.discard(q)

        return func()

    def listen(self, fresh=False) -> AsyncGenerator[_MsgType, None]:
        """Itterates over the newest message.

        Messages might be skipped, this is not a queue.

        .. Note:
            "Listening" starts the moment this function is called, not when
            the generator is awaited

        Args:
            fresh: If false, first yield can be the latest value

        Returns:
            Async generator itterating over the newest message.
        """
        return self.listen_reliable(fresh, 1)

    def listen_reliable(
        self,
        fresh=False,
        queue_size: int = 10,
        lifo=False,
        exit_on_close: bool = True,
    ) -> AsyncGenerator[_MsgType, None]:
        """Itterates over every incomming messages. (does not miss messages)

        .. Note:
            "Listening" starts the moment this function is called, not when
            the generator is awaited.


        Args:
            fresh: If false, first yield can be the latest value
            queue_size: size of the queue of values
            lifo: If True, uses a last in first out queue instead of default fifo.
            return_on_close: If True, the async for loop will exit when
                `.close()` is called. Else, exception will be raised (default)

        Returns:
            Async generator itterating over every incomming message.
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
        self, queue: asyncio.Queue, exit_on_close: bool = True
    ) -> AsyncGenerator[_MsgType, None]:
        logger.debug("Reliable listener first iter %s", self.name)
        try:
            while True:
                msg = await queue.get()
                if msg is _CLOSE_SENTINEL:
                    if exit_on_close:
                        return
                    raise SubClosedException(f"Subscriber '{self.name}' was closed")
                yield msg
        finally:
            self._dyncamic_queues.discard(queue)
            logger.debug("Reliable listener closed %s", self.name)

    def _input_data_asyncio(self, msg: _MsgType):
        """Processes incomming data.

        Is only safe to run on the same thread as asyncio.

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
        """Stops everything waiting on data.

        Raises exception inside all coroutines/tasks waiting on the next data.
        Async for loops will exit (if setup to do so)."""
        if self._closed.is_set():
            return
        logger.debug("%s closed", self.name)
        self._closed.set()
        for q in list(self._dyncamic_queues):
            if q.full():
                q.get_nowait()
            q.put_nowait(_CLOSE_SENTINEL)
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
    def __init__(
        self,
        sub: BaseSub[_InType],
        convert_func: Callable[[_InType], _OutType] = lambda x: x,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ) -> None:
        """Subscriber that applies a function to another subscriber.

        This subscriber forwards messages from an upstream ``BaseSub``
        through ``convert_func`` and publishes the transformed values as a new
        ``BaseSub`` instance.

        Conversion happens inline in the upstream callback path through
        ``sub.asap_callback``. This keeps forwarding lean and reliable, but it
        also means:

        - only incomming upstream messages are forwarded
        - there is no separate worker task or buffer
        - if convert_func is slow it propagates upstream

        Note:
            - Conversion errors fail this subscriber lifetime, not upstream.
            - the lifetime upstream is not linked in any way to the lifetime downstream

        Args:
            sub: Upstream subscriber producing input values.
            convert_func: Function applied to each incoming message. Identity
                by default.
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
