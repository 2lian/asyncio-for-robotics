import asyncio
import logging
import threading
import uuid
import warnings
from abc import ABC, abstractmethod
from contextlib import contextmanager
from typing import Any, Generator, Optional, Union

import rclpy
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.node import Node
from rclpy.task import Future as FutureRos

logger = logging.getLogger(__name__)


class BaseSession(ABC):
    def __init__(
        self,
        node: Union[None, str, Node] = None,
        executor: None | SingleThreadedExecutor | MultiThreadedExecutor = None,
    ) -> None:
        """ROS 2 session owning node, executor, and optional rclpy lifecycle.

        .. Critical:
            Use the lock() context to safely interact with the node from the main thread.

        Args:
            name: name of the node, None will give it a UUID
            executor: executor used by this session. Defaults to
                ``SingleThreadedExecutor()``.

        Important:
            If ROS is not already initialized, this session calls
            ``rclpy.init()`` itself and will later call ``rclpy.shutdown()``
            when closed. If ROS was already initialized by something else,
            this session leaves shutdown ownership alone.
        """
        logger.debug("Initializing ROS Sessions")
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            logger.debug("Initializing RCLPY")
            rclpy.init()
        if isinstance(node, str):
            name = node
        else:
            name = f"afor_{uuid.uuid4()}".replace("-", "_")
        if not isinstance(node, Node):
            node = Node(name)
        else:
            name = node.get_name()
        self._node: Node = node
        if executor is None:
            executor = SingleThreadedExecutor()
        self._executor: Union[SingleThreadedExecutor, MultiThreadedExecutor] = executor
        self._executor.add_node(self._node)
        self._lock = threading.RLock()
        self._closed = False

    def set_global_session(self) -> None:
        """Deprecated.

        Session objects should not mutate module-global fallback state.
        Use lexical ``session_context(...)`` / ``auto_context(...)`` instead.
        """
        warnings.warn(
            "BaseSession.set_global_session() is deprecated and does nothing. "
            "Use session_context(...) or auto_context(...) instead.",
            DeprecationWarning,
            stacklevel=2,
        )

    @abstractmethod
    @contextmanager
    def lock(self) -> Generator[Node, Any, Any]:
        """Context to stop the node from spinning momentarly.

        Use like so:
            ```python
            with session.lock() as node:
                node.create_subscription(...)
            # your code continues safely
            ```
        """
        ...

    @abstractmethod
    def start(self):
        """Starts spinning the ros2 node"""
        ...

    def stop(self):
        """Stops the executor and node"""
        with self.lock():
            try:
                self._node.destroy_node()
            except Exception as exc:
                logger.debug("Ignoring ROS node destroy error during shutdown: %s", exc)
            try:
                self._executor.shutdown()
            except Exception as exc:
                logger.debug(
                    "Ignoring ROS executor shutdown error during shutdown: %s", exc
                )

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.stop()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()


class ThreadedSession(BaseSession):
    def __init__(
        self,
        node: Union[None, str, Node] = None,
        executor: Union[None, SingleThreadedExecutor, MultiThreadedExecutor] = None,
    ) -> None:
        """Ros2 node spinning in its own background thread.
        ROS2 Callbacks are therefor short and never-blocking.

        .. Critical:
            Use the lock() context to safely interact with the node from the main thread.

        .. Important:
            (I think) In the current version. You cannot have more than one per
            python instance.
        """
        super().__init__(node, executor)
        self.thread = threading.Thread(target=self._spin_thread, daemon=True)
        self._stop_event = threading.Event()
        self._can_spin_event = threading.Event()
        self._node.create_timer(0.01, self._ros_pause_check)
        logger.debug("Initialized Threaded ROS Sessions")

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
        logger.debug("lock requested")
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
        self._stop_event.clear()
        if not self.thread.is_alive():
            logger.debug("RosNode thread started")
            self._resume()
            self.thread.start()

    def stop(self):
        self._stop_event.set()
        super().stop()
        self.thread.join()
        logger.debug("RosNode thread stoped")

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
                if not aok():
                    return
            except TimeoutError:
                continue
            logger.debug("ROS waiting for lock")
            with self._lock:
                logger.debug("ROS has lock, spinning")
                self.pause_fut = FutureRos()
                try:
                    self._executor.spin_until_future_complete(self.pause_fut)
                except ExternalShutdownException:
                    logger.debug("ROS executor shut down externally")
                    return
                logger.debug("ROS spinning paused")
            logger.debug("ROS released lock")
        logger.debug("ROS spinning TERMINATED")

    def _ros_pause_check(self):
        """(thrd 2) Timer checking if ros spin should pause"""
        try:
            pause_called = not self._can_spin_event.is_set()
            stop_called = self._stop_event.is_set()
            if stop_called or pause_called:
                if self.pause_fut.done():
                    return
                self.pause_fut.set_result(None)
                logger.debug(
                    "Rclpy pause set pause_called=%s, stop_called=%s",
                    pause_called,
                    stop_called,
                )
        except Exception as e:
            print(e)
            logger.critical(e)

    def _pause(self):
        self._can_spin_event.clear()

    def _resume(self):
        self._can_spin_event.set()


class SynchronousSession(BaseSession):
    def __init__(
        self,
        node: Union[None, str, Node] = None,
        executor: Union[None, SingleThreadedExecutor, MultiThreadedExecutor] = None,
    ) -> None:
        """Ros2 node spinning in as periodic asyncio task.

        .. Important:
            Unlike ThreadedSession, lock is not necessary to interact with the node.
        """
        super().__init__(node, executor)
        self.rosloop_task: Optional[asyncio.Task] = None

    async def _spin_periodic(self):
        """Executes in the thread to spin the node"""
        while 1:
            await asyncio.sleep(0)
            self._executor.spin_once(timeout_sec=0.0)

    @contextmanager
    def lock(self) -> Generator[Node, Any, Any]:
        try:
            yield self._node
        finally:
            pass

    def start(self):
        """Starts spinning the ros2 node in a asyncio task"""
        logger.debug("RosNode asyncio started")
        self._event_loop = asyncio.get_event_loop()
        self.rosloop_task = self._event_loop.create_task(self._spin_periodic())

    def stop(self):
        """Stops spinning the ros2 node and joins the thread"""
        super().stop()
        if self.rosloop_task is not None:
            self.rosloop_task.cancel()
        logger.debug("RosNode asyncio stoped")
