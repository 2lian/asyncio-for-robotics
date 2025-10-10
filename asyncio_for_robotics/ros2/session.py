import asyncio
import logging
import threading
import uuid
from abc import ABC, abstractmethod
from contextlib import contextmanager, suppress
from typing import Any, Generator, Optional, Self, Union

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future as FutureRos

logger = logging.getLogger(__name__)


class BaseSession(ABC):
    def __init__(
        self,
        node: Union[None, str, Node] = None,
        executor: Union[None, SingleThreadedExecutor, MultiThreadedExecutor] = None,
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
        name = f"SessionNode{uuid.uuid4()}".replace("-", "_")
        if isinstance(node, str):
            name = node
        if not isinstance(node, Node):
            node = Node(name)
        self._node: Node = node
        if executor is None:
            executor = GLOBAL_SESSION_DEFAULT_EXEC()
        self._executor: Union[SingleThreadedExecutor, MultiThreadedExecutor] = executor
        self._executor.add_node(self._node)
        self._lock = threading.Lock()

    def set_global_session(self) -> None:
        """Make this object instance the global session used by default with auto_session."""
        set_auto_session(self)

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
            self._node.destroy_node()
            self._executor.shutdown()

    def close(self):
        global GLOBAL_SESSION
        self.stop()
        if GLOBAL_SESSION is self:
            GLOBAL_SESSION = None


class ThreadedSession(BaseSession):
    _global_node: Optional[Self] = None

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
                # checks back on aok every timeout
                continue
            logger.debug("ROS waiting for lock")
            with self._lock:
                logger.debug("ROS has lock, spinning")
                # spins until the pause future is triggered
                self.pause_fut = FutureRos()
                self._executor.spin_until_future_complete(self.pause_fut)
                logger.debug("ROS spinning paused")
            logger.debug("ROS released lock")
        logger.debug("ROS spinning TERMINATED")

    def _ros_pause_check(self):
        """(thrd 2) Timer checking if ros spin should pause"""
        try:
            cannot_spin = (not self._can_spin_event.is_set()) or self._stop_event
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


class SynchronousSession(BaseSession):
    _global_node: Optional[Self] = None

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


#: type of executor used by default
GLOBAL_SESSION_DEFAULT_EXEC: Union[
    type[SingleThreadedExecutor], type[MultiThreadedExecutor]
] = SingleThreadedExecutor

#: type of session created by default
GLOBAL_SESSION_DEFAULT_TYPE: type[BaseSession] = ThreadedSession

#: global share session (singleton)
GLOBAL_SESSION: Optional[BaseSession] = None


def set_auto_session(session: Optional[BaseSession] = None) -> None:
    global GLOBAL_SESSION
    GLOBAL_SESSION = session


def auto_session(session: Optional[BaseSession] = None) -> BaseSession:
    global GLOBAL_SESSION, GLOBAL_SESSION_DEFAULT_EXEC, GLOBAL_SESSION_DEFAULT_TYPE
    if session is not None:
        return session
    if GLOBAL_SESSION is None:
        ses = GLOBAL_SESSION_DEFAULT_TYPE(
            node=None, executor=GLOBAL_SESSION_DEFAULT_EXEC()
        )
        GLOBAL_SESSION = ses
    else:
        ses = GLOBAL_SESSION
    ses.start()
    return ses
