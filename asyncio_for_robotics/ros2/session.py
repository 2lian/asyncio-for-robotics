import asyncio
import logging
import threading
import time
import uuid
from asyncio.queues import Queue
from collections import deque
from contextlib import contextmanager, suppress
from dataclasses import dataclass, field
from typing import (
    Any,
    AsyncGenerator,
    Awaitable,
    Deque,
    Final,
    Generator,
    Generic,
    List,
    NamedTuple,
    Optional,
    Self,
    Set,
    TypeVar,
    Union,
)

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_system_default,
)
from rclpy.task import Future as FutureRos

from .utils import TopicInfo

logger = logging.getLogger(__name__)



class ThreadedSession:
    _global_node: Optional[Self] = None

    def __init__(
        self,
        node: Union[None, str, Node] = None,
        executor: Union[None, SingleThreadedExecutor, MultiThreadedExecutor] = None,
        make_global: bool = False,
    ) -> None:
        """Ros2 node spinning in its own thread.

        .. Critical:
            Use the lock() context to safely interact with the node from the main thread.

        .. Important:
            (I think) You cannot have more than one per python instance.

        Args:
            name: name of the node, None will give it a UUID
        """
        logger.debug("Initializing Threaded ROS Sessions")
        name = f"ThreadedNode{uuid.uuid4()}".replace("-", "_")
        if isinstance(node, str):
            name = node
        if not isinstance(node, Node):
            node = Node(name)
        self._node = node
        if executor is None:
            executor = SingleThreadedExecutor()
        self._executor = executor
        self._executor.add_node(self._node)
        self.thread = threading.Thread(target=self._spin_thread, daemon=True)
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._can_spin_event = threading.Event()
        self._node.create_timer(0.01, self._ros_pause_check)
        if make_global:
            self.set_global_session(self)
        logger.debug("Initialized Threaded ROS Sessions")

    @classmethod
    def auto_session(cls, session: Optional[Self] = None) -> Self:
        if session is not None:
            return session
        if cls._global_node is None:
            logger.debug("Global Threaded Ros Session set")
            inst = cls()
            cls.set_global_session(inst)
            # cls._global_node: Self
        return cls._global_node  # type: ignore

    @classmethod
    def set_global_session(cls, session: Self) -> None:
        global DEFAULT_SESSION_TYPE
        DEFAULT_SESSION_TYPE = cls
        cls._global_node = session

    @contextmanager
    def lock(self) -> Generator[Node, Any, Any]:
        """This context is required for every operation (aside from start/stop).
        Safely, quickly acquires lock on the whole ros2 thread.

        Use like so:
            ```python
            with session.lock() as node:
                node.create_subscription(...)
            # your code continues safely
            ```

        This will pause every ros2 callbacks and processes of the executor in
        use by this object. Only use it shortly to add something to the node
        (pub/sub, timer...)

        This context pauses the executor spinning, and acquires the _lock around it. To
        be double safe.

        When exiting this context, the internal _lock is freed and executor resumed.
        """
        was_running = self._can_spin_event.is_set()
        try:
            self._pause()
            with self._lock:
                yield self._node
        finally:
            if was_running:
                self._resume()

    def start(self):
        """Starts spinning the ros2 node in its thread"""
        if not self.thread.is_alive():
            logger.debug("RosNode thread started")
            self._resume()
            self.thread.start()

    def stop(self):
        """Stops spinning the ros2 node, destroys it and joins the thread"""
        self._pause()
        self._stop_event.set()
        with self.lock():
            self._node.destroy_node()
        self.thread.join()
        logger.debug("RosNode thread stoped")

    def close(self):
        self.stop()

    def _spin_thread(self):
        """(thrd 2) Executes in a second thread to spin the node"""

        def aok():
            if self._executor is None:
                exec = True
            else:
                exec = self._executor.context.ok()
            return exec and not self._stop_event.is_set()

        while aok():
            try:
                self._can_spin_event.wait(timeout=1)
            except TimeoutError:
                # checks back on aok every timeout
                continue
            logger.debug("ROS waiting for lock")
            with self._lock:
                logger.debug("ROS has lock and is spinning")
                # spins until the pause future is triggered
                self.pause_fut = FutureRos()
                self._executor.spin_until_future_complete(self.pause_fut)
                logger.debug("ROS stopped spinning")
            logger.debug("ROS released lock")

    def _ros_pause_check(self):
        """(thrd 2) Timer checking if ros spin should end"""
        try:
            cannot_spin = not self._can_spin_event.is_set()
            # logger.info(f"{cannot_spin=}")
            if cannot_spin:
                if self.pause_fut.done():
                    return
                self.pause_fut.set_result(None)
                logger.debug("Rclpy pause set")
        except Exception as e:
            print(e)
            logger.critical(e)

    def _pause(self):
        self._can_spin_event.clear()

    def _resume(self):
        self._can_spin_event.set()


class AsyncioSession:
    _global_node: Optional[Self] = None

    def __init__(
        self,
        node: Union[None, str, Node] = None,
        executor: Union[None, SingleThreadedExecutor, MultiThreadedExecutor] = None,
        make_global: bool = False,
    ) -> None:
        """Ros2 node spinning in as periodic asyncio task.

        .. Important:
            Unlike ThreadedSession, lock is not necessary to interact with the node.
        """
        name = f"ThreadedNode{uuid.uuid4()}".replace("-", "_")
        if isinstance(node, str):
            name = node
        if not isinstance(node, Node):
            node = Node(name)
        self.node = node
        if executor is None:
            executor = SingleThreadedExecutor()
        self._executor = executor
        self._executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin_thread, daemon=True)
        if make_global:
            self.set_global_session(self)

    @classmethod
    def auto_session(cls, session: Optional[Self] = None) -> Self:
        if session is not None:
            return session
        if cls._global_node is None:
            inst = cls()
            cls.set_global_session(inst)
            # cls._global_node: Self
        return cls._global_node  # type: ignore

    @classmethod
    def set_global_session(cls, session: Self) -> None:
        global DEFAULT_SESSION_TYPE
        DEFAULT_SESSION_TYPE = cls
        cls._global_node = session

    async def _spin_thread(self):
        """Executes in the thread to spin the node"""
        while 1:
            await asyncio.sleep(0.001)
            self._executor.spin_once(timeout_sec=0.0)

    @contextmanager
    def lock(self) -> Generator[Node, Any, Any]:
        try:
            yield self.node
        finally:
            pass

    def start(self):
        """Starts spinning the ros2 node in its thread"""
        logger.debug("RosNode asyncio started")
        self._event_loop = asyncio.get_event_loop()
        self.rosloop_task = self._event_loop.create_task(self._spin_thread())

    def stop(self):
        """Stops spinning the ros2 node and joins the thread"""
        self.rosloop_task.cancel()
        self.node.destroy_node()
        logger.debug("RosNode asyncio stoped")

    def close(self):
        self.stop()


RosSession = Union[ThreadedSession, AsyncioSession]
DEFAULT_SESSION_TYPE: Union[type[ThreadedSession], type[AsyncioSession]] = (
    ThreadedSession
)


def auto_session(session: Optional[RosSession] = None) -> RosSession:
    if session is not None:
        return session
    ses= DEFAULT_SESSION_TYPE.auto_session()
    ses.start()
    return ses
