import asyncio
import logging
from abc import ABC, abstractmethod
from asyncio.queues import Queue
from collections import deque
from types import CoroutineType
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

_MsgType = TypeVar("_MsgType")


class BaseSub(Generic[_MsgType], ABC):
    def __init__(
        self,
        buff_size: int = 10,
    ) -> None:
        """Abstract subscriber working with python asyncio.

        Args:
            buff_size: Maximum size of the buffer of messages.
        """
        #: FIFO buffer accumulating latest ros2 messages (avoid using this, use the provided async generators)
        self.buffer: Deque[_MsgType] = deque(maxlen=buff_size)
        #: Event triggering when the first message is received
        self.alive = asyncio.Event()
        #: Number of messages received since start
        self.msg_count: int = 0
        #: Condition that fires on new data
        self.new_value_cond = asyncio.Condition()
        #: Blocking callbacks called first when msg is available
        self.asap_callback: List[Callable[[_MsgType], Any]] = []
        #: queues associated with the reliable async generators
        self._dyncamic_queues: Set[Queue[_MsgType]] = set()

        self._event_loop = asyncio.get_event_loop()
        #: Value is available event
        self._value_flag = asyncio.Event()
        #: Lastest message value
        self._value: Optional[_MsgType] = None
        logger.debug("created sub %s", self.name)

    @property
    @abstractmethod
    def name(self) -> str:
        """The friendly name of you subscriber"""
        ...

    async def wait_for_value(self) -> _MsgType:
        """Awaits a value.

        Returns:
            Awaits at least one message received, returns the latest (often immediately)
        """
        await self._value_flag.wait()
        assert self._value is not None
        return self._value

    def wait_for_new(self) -> Coroutine[Any, Any, _MsgType]:
        """Awaits a new value.

        Only ensure that the data is new. See wait_for_next to be sure the data
        is exactly the next after the call.

        Returns:
            Later, with a value more recent than the one at time of the call
        """
        last_count = self.msg_count

        async def func() -> _MsgType:
            async with self.new_value_cond:
                await self.new_value_cond.wait_for(lambda: self.msg_count > last_count)
            assert self._value is not None
            return self._value

        return func()

    def wait_for_next(self) -> Awaitable[_MsgType]:
        """Reliably awaits the next arriving value.

        If several awaits start waiting for this simultaneously,  they will wake up
        with the same value.

        Returns:
            Later, with the exact next value received after the call is made
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
        """Async generator used to coninuously get new values.

        Messages might be skipped as this is not a queue.

        .. Note:
            The listening starts the moment this function is called, not when
            the generator is awaited


        Args:
            fresh: If false, first yield can be the latest value

        Yields:
            awaits data then awaits newer data
        """
        return self.listen_reliable(fresh, 1, False)

    def listen_reliable(
        self, fresh=False, queue_size: int = 10, lifo=False
    ) -> AsyncGenerator[_MsgType, None]:
        """Reliable async generator to get new values without missing any.

        .. Note:
            The listening starts the moment this function is called, not when
            the generator is awaited

        Implements a fifo queue that gets destroyed with this generator.

        Args:
            fresh: If false, first yield can be the latest value
            queue_size: size of the queue of values
            lifo: If True, uses a last in first out queue instead of default lifo.

        Yields:
            awaits data then awaits newer data. Does not miss data, unless queue is full.
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
        return self._unprimed_listen_reliable(q)

    async def _unprimed_listen_reliable(
        self, queue: asyncio.Queue
    ) -> AsyncGenerator[_MsgType, None]:
        logger.debug("Reliable listener first iter %s", self.name)
        try:
            while True:
                # logger.debug("Reliable listener waiting data %s", self.name)
                msg = await queue.get()
                # logger.debug("Reliable listener got data %s", self.name)
                yield msg
                # logger.debug("Reliable listener yielded data %s", self.name)
        finally:
            self._dyncamic_queues.discard(queue)
            logger.debug("Reliable listener closed %s", self.name)

    @abstractmethod
    def callback_for_sub(self, *args, **kwargs) -> Any:
        """Use this as a callback to your data stream (you subscriber, listener...).
        Once your data is processed call self.input_data with your data.

        If your callback is executed by another thread, inside this function is
        not threadsafe! so be carefull when accessing attributes and catching
        errors.

        A notable error that I let you catch happens when self._event_loop is
        closed. It happens rarely, but means the the asyncio process has been
        closed, thus this callback cannot be used anymore. see: self._event_loop.is_closed()

            *args:
            **kwargs:
        """
        ...

    def input_data(self, data: _MsgType) -> bool:
        """Input receiving data. Call this with your data when new message is available.

        This is threadsafe, thus can run safely on any thread.

        Args:
            data: Data to input in this sub

        Returns:
            False if the even loop has been closed or there is another critical
                problem making this sub unable to work.
        """
        if self._event_loop.is_closed():
            logger.info("Event loop closed, for sub: %s", self.name)
            return False
        self._event_loop.call_soon_threadsafe(self._new_message_cbk, data)
        return True

    def _new_message_cbk(self, msg: _MsgType):
        """asyncio safe callback when new message is available.

        Args:
            msg: message from ros
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
        self.buffer.append(msg)
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


class BasePub(Generic[_MsgType], ABC):
    def __init__(self):
        """Standalone ros2 publisher working with python asyncio.

        Args:
            node: A ROS2 BackgroundNode of this async module that is running.
            topic: Topic to publish onto
        """
        ...

    @abstractmethod
    def pub(self, data: _MsgType):
        """Publishes a message"""
        ...
