import asyncio
import logging
from asyncio.queues import Queue
from contextlib import suppress
from typing import (
    Any,
    AsyncGenerator,
    Awaitable,
    Callable,
    Coroutine,
    Deque,
    Final,
    Generic,
    List,
    Optional,
    Set,
    TypeVar,
)

logger = logging.getLogger(__name__)


class SubClosedException(RuntimeError):
    """Raised when awaiting data from a closed subscriber."""


_MsgType = TypeVar("_MsgType")
_T = TypeVar("_T")


class BaseSub(Generic[_MsgType]):
    def __init__(
        self,
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
        #: Number of messages received since start
        self.msg_count: int = 0
        #: Condition that fires on new data
        self.new_value_cond = asyncio.Condition()
        #: queues associated with the generators
        self._dyncamic_queues: Set[Queue[_MsgType]] = set()

        self._event_loop: asyncio.AbstractEventLoop = asyncio.get_event_loop()
        #: Value is available event
        self._value_flag: asyncio.Event = asyncio.Event()
        #: Lastest message value
        self._value: Optional[_MsgType] = None
        #: Event when closing the sub
        self._closed = asyncio.Event()
        self._raise_on_close_exc: Exception = SubClosedException(
            f"Subscriber '{self.name}' has been closed"
        )
        logger.debug("created sub %s", self.name)

    @property
    def name(self) -> str:
        """The friendly name of you subscriber"""
        return "no_name"

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
        if self._event_loop.is_closed():
            logger.info("Event loop closed, for sub: %s", self.name)
            return False
        self._event_loop.call_soon_threadsafe(self._input_data_asyncio, data)
        return True

    async def wait_for_value(self) -> _MsgType:
        """Latest message.

        Returns:
            The latest message (if none, awaits the first message)
        """
        await self._wait_or_raise_closed(self._value_flag.wait())
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
        last_count = self.msg_count

        async def func() -> _MsgType:
            async with self.new_value_cond:
                await self.new_value_cond.wait_for(lambda: self.msg_count > last_count)
            assert self._value is not None
            return self._value

        return self._wait_or_raise_closed(func())

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
                return val_deep
            finally:
                self._dyncamic_queues.discard(q)

        return self._wait_or_raise_closed(func())

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
        return self.listen_reliable(fresh, 1, False)

    def listen_reliable(
        self,
        fresh=False,
        queue_size: int = 10,
        lifo=False,
        exit_on_close: bool = False,
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
        self, queue: asyncio.Queue, exit_on_close: bool = False
    ) -> AsyncGenerator[_MsgType, None]:
        logger.debug("Reliable listener first iter %s", self.name)
        try:
            while True:
                msg = await self._wait_or_raise_closed(queue.get())
                yield msg
        except Exception as e:
            if e == self._raise_on_close_exc and exit_on_close:
                return
            else:
                raise e
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
                    logger.warning("Queue full (%s) on %s", q.qsize, self.name)
                q.get_nowait()
            q.put_nowait(msg)
        self._value = msg
        self._value_flag.set()
        self.msg_count += 1
        asyncio.create_task(self._wakeup_new())
        if not self.alive.is_set():
            logger.debug("%s is receiving data", self.name)
            self.alive.set()

    async def _wakeup_new(self):
        """fires new_value_cond"""
        async with self.new_value_cond:
            self.new_value_cond.notify_all()

    async def _wait_or_raise_closed(
        self, awaitable: Coroutine[Any, Any, _T] | Awaitable[_T]
    ) -> _T:
        """Wait for an awaitable or raise exception on subscriber shutdown.

        Raises:
            SubClosedError: if the subscriber is closed before completion.
            Exception: any exception raised by the awaitable.

        Returns:
            Awaited awaitable
        """
        main_task = asyncio.ensure_future(awaitable)
        close_task = asyncio.ensure_future(self._closed.wait())
        done, pending = await asyncio.wait(
            [main_task, close_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
        try:
            if close_task in done:
                logger.info(f"Terminating task because sub is closed")
                raise self._raise_on_close_exc
            return await main_task
        finally:
            for task in pending:
                task.cancel()
            if len(pending) != 0:
                with suppress(asyncio.CancelledError):
                    await asyncio.wait(pending)

    def close(self):
        self._closed.set()


_InType = TypeVar("_InType")
_OutType = TypeVar("_OutType")


class ConverterSub(BaseSub[_OutType]):
    def __init__(
        self,
        sub: BaseSub[_InType],
        convert_func: Callable[[_InType], _OutType] = lambda x: x,
    ) -> None:
        """Subscriber that applies a transformation to another subscriber.

        This subscriber listens reliably to an upstream ``BaseSub`` and
        publishes transformed messages as a new ``BaseSub`` instance. The
        original subscriber is left unchanged.

        Args:
            sub: Upstream subscriber to listen to.
            convert_func: Function applied to each incoming message. Identity
                by default.
        """
        #: Upstream subscriber
        self.sub: BaseSub = sub
        #: Transformation applied to each received message
        self.convert_func = convert_func
        super().__init__()
        #: Background task running the conversion loop
        self._loop_task = asyncio.create_task(self._converter_loop())
        #: Optional callback invoked on close (typically upstream close).
        self.callback_on_close: Optional[Callable[[], Any]] = None
        if hasattr(self.sub, "close"):
            if callable(self.sub.close):
                self.callback_on_close = self.sub.close

    async def _converter_loop(self):
        async for msg in self.sub.listen_reliable():
            new = self.convert_func(msg)
            self._input_data_asyncio(new)

    def close(self):
        self._loop_task.cancel()
        if self.callback_on_close is not None:
            self.callback_on_close()
        super().close()
