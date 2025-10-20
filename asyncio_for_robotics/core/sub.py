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
        logger.debug("created sub %s", self.name)

    @property
    @abstractmethod
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
        last_count = self.msg_count

        async def func() -> _MsgType:
            async with self.new_value_cond:
                await self.new_value_cond.wait_for(lambda: self.msg_count > last_count)
            assert self._value is not None
            return self._value

        return func()

    def wait_for_next(self) -> Awaitable[_MsgType]:
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
        self, fresh=False, queue_size: int = 10, lifo=False
    ) -> AsyncGenerator[_MsgType, None]:
        """Itterates over every incomming messages. (does not miss messages)

        .. Note:
            "Listening" starts the moment this function is called, not when
            the generator is awaited.


        Args:
            fresh: If false, first yield can be the latest value
            queue_size: size of the queue of values
            lifo: If True, uses a last in first out queue instead of default fifo.

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
