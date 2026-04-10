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
        self.lifetime: asyncio.Future[bool] = asyncio.Future()
        logger.debug("created sub %s", self.name)

    @property
    def name(self) -> str:
        """The friendly name of you subscriber"""
        return "no_name"

    def async_bind(self) -> Coroutine[None, None, None]:
        """This code was a test implementation

        Binds the lifetime to a coroutine (for an asyncio task).

        This is especially useful to capture and propagate exception happening
        in the background input tasks.

        The coroutine returns when the sub is closed or raises any exception
        happening in the internal `call_soon`. Canceling this coroutine
        (through a taskgroup or manually) will close this sub.

        Returns: 
            A coroutine representing the lifetime of this sub and
            propagating exceptions.

        Example:
            async with asyncio.TaskGroup() as tg:
                sub = BaseSub(...)
                tg.create_task(node.async_bind())
        """
        async def bind():
            try:
                await self.lifetime
            finally:
                self.close()

        return bind()

    def _wrap_input_lifetime(self, data: _MsgType):
        """This code was a test implementation

        Calls self._input_data_asyncio, propagating exception to
        self.lifetime and guarding if lifetime is done.
        """
        if self.lifetime.done():
            print("done")
            return
        try:
            self._input_data_asyncio(data)
        except Exception as e:
            print("exc")
            self.lifetime.set_exception(e)

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
            print("dead")
            logger.info("Event loop closed, for sub: %s", self.name)
            return False
        try:
            self._event_loop.call_soon_threadsafe(self._wrap_input_lifetime, data)
        except:
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
                msg = await queue.get()
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
        self._closed.set()
        if not self.lifetime.done():
            self.lifetime.set_result(True)


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
        self._loop_task = asyncio.create_task(
            self._converter_loop(), name="afor_converter"
        )
        #: Optional callback invoked on close (typically upstream close).
        self.callback_on_close: Optional[Callable[[], Any]] = None
        if hasattr(self.sub, "close"):
            if callable(self.sub.close):
                self.callback_on_close = self.sub.close

    async def _converter_loop(self):
        async for msg in self.sub.listen_reliable(queue_size=0):
            new = self.convert_func(msg)
            self._input_data_asyncio(new)

    def close(self):
        self._loop_task.cancel()
        if self.callback_on_close is not None:
            self.callback_on_close()
        super().close()
