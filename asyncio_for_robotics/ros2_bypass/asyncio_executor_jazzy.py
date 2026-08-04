"""Jazzy prototype executor which runs normal ROS callbacks in asyncio.

The ROS wait set is blocked on a worker thread.  Every callback made ready by
one wait-set evaluation is forwarded to the asyncio thread with one
``call_soon_threadsafe`` call.

This module intentionally relies on ``rclpy.Executor`` private APIs from ROS 2
Jazzy.  In return, normal :class:`rclpy.node.Node` objects retain rclpy's
entity-taking, callback-group, waitable, future, and task machinery.
"""

import asyncio
import inspect
import threading
import time
from contextlib import ExitStack, suppress
from typing import Any

import rclpy
from rclpy.context import Context
from rclpy.executors import (
    ConditionReachedException,
    Executor,
    ExternalShutdownException,
    ShutdownException,
)
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from ..core._compat import TaskGroup

ReadyCallback = tuple[Any, Any, Node | None]


class BatchAsyncioExecutor(Executor):
    """Execute rclpy callbacks on an asyncio event loop.

    ``start()`` is the long-lived lifecycle coroutine. It waits for ROS work on
    a worker thread running the wait_set. Once the waitset has some work,
    asyncio executes it. SO there is a single cross-thread asyncio wakeup call
    per wait_set wake, instead of one per callback.

    ROS callback can now all be coroutines executing in the asyncio event-loop,
    HOWEVER not returning from the coroutine will block the ROS thread and
    message collection. So we recommand returning as early as possible from the
    callbacks.
    """

    def __init__(
        self,
        *,
        context: Context | None = None,
    ) -> None:
        super().__init__(context=context)
        self._loop: asyncio.AbstractEventLoop

        self._batch_finished = threading.Event()
        self._batch_finished.set()

        self._stop_requested = threading.Event()
        self._task_group: TaskGroup | None = None
        self._started = False

    def _cancel_handlers(self, batch: list[ReadyCallback]) -> None:
        for handler, entity, _ in batch:
            handler.cancel()
            handler._complete_task()
            if entity is not None:
                entity._executor_event = False
        self._batch_finished.set()

    async def _execute_handler(self, handler: Any) -> None:
        """Execute one rclpy handler, using asyncio for coroutine handlers."""
        if handler.cancelled():
            handler._complete_task()
            return

        coroutine = getattr(handler, "_handler", None)
        if not inspect.iscoroutine(coroutine):
            handler()
            if handler.done() and not handler.cancelled():
                exception = handler.exception()
                if exception is not None:
                    raise exception
            return

        try:
            result = await coroutine
        except asyncio.CancelledError:
            handler.cancel()
            raise
        except BaseException as exception:
            handler.set_exception(exception)
            # Mark the rclpy exception as observed; TaskGroup propagates the
            # original exception from this coroutine.
            handler.exception()
            raise
        else:
            handler.set_result(result)
        finally:
            handler._complete_task()

    async def _execute_batch(self, batch: list[ReadyCallback]) -> None:
        """Execute one complete ready batch as a single asyncio coroutine."""
        handlers = iter(batch)
        try:
            for handler, _, _ in handlers:
                await self._execute_handler(handler)
        finally:
            self._cancel_handlers(list(handlers))

    def _create_batch_task(self, batch: list[ReadyCallback]) -> None:
        task_group = self._task_group
        if task_group is None:
            self._cancel_handlers(batch)
            return

        coroutine = self._execute_batch(batch)
        try:
            task_group.create_task(coroutine)
        except RuntimeError:
            coroutine.close()
            self._cancel_handlers(batch)

    def _schedule_batch(self, batch: list[ReadyCallback]) -> None:
        self._batch_finished.clear()
        try:
            # This is the only ROS-thread -> asyncio-thread operation for the
            # complete wait-set batch.
            self._loop.call_soon_threadsafe(self._create_batch_task, batch)
        except BaseException:
            self._cancel_handlers(batch)
            raise

    def _collect_ready_batch(self) -> list[ReadyCallback]:
        """Wait for one callback, then drain its wait-set generator.

        Calling ``list()`` on Jazzy's generator is subtly unsafe. When it first
        yields a task created with ``Executor.create_task()``, continuing
        iteration can enter a blocking wait before the generator finishes.
        Such tasks have no source entity, so wake the generator only in that
        case. A normal ROS entity was yielded after the wait and needs no wake.
        """
        callback_generator = self._wait_for_ready_callbacks(
            None,
            None,
            self._stop_requested.is_set,
        )
        first = next(callback_generator)
        batch = [first]
        if first[1] is None:
            self.wake()

        while True:
            try:
                batch.append(next(callback_generator))
            except (ConditionReachedException, StopIteration):
                break
            except BaseException:
                self._cancel_handlers(batch)
                raise

        return batch

    def spin(self) -> None:
        """Wait for ROS work until ``start()`` is cancelled or shutdown occurs."""
        self._enter_spin()
        try:
            while (
                self.context.ok()
                and not self._is_shutdown
                and not self._stop_requested.is_set()
            ):
                self._batch_finished.wait()
                try:
                    batch = self._collect_ready_batch()
                except (
                    ConditionReachedException,
                    ExternalShutdownException,
                    ShutdownException,
                ):
                    continue
                self._schedule_batch(batch)
        finally:
            self._exit_spin()

    async def start(self) -> None:
        """Run until cancelled, then stop callbacks and the ROS wait thread."""
        if self._started:
            raise RuntimeError("AsyncioExecutor has already been started")
        self._started = True
        self._loop = asyncio.get_running_loop()

        try:
            async with TaskGroup() as task_group:
                self._task_group = task_group
                spin_future = self._loop.run_in_executor(None, self.spin)
                try:
                    # Shielding keeps the worker future joinable when start()
                    # itself is cancelled.
                    await asyncio.shield(spin_future)
                finally:
                    self._stop_requested.set()
                    self._batch_finished.set()
                    self.wake()
                    if not spin_future.done():
                        await asyncio.shield(spin_future)
        finally:
            self._task_group = None
            super().shutdown()


async def main() -> None:
    """Listen to ``example/talker`` using a completely normal ROS node."""
    with ExitStack() as stack:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        stack.callback(rclpy.try_shutdown)

        node = Node("afor_asyncio_executor_example")
        stack.callback(node.destroy_node)

        asyncio_thread = threading.get_ident()

        queue = asyncio.Queue()

        async def on_message(msg: String) -> None:
            assert threading.get_ident() == asyncio_thread
            await queue.put(msg)

        node.create_subscription(String, "example/talker", on_message, 10)

        executor = BatchAsyncioExecutor(context=node.context)
        executor.add_node(node)
        async with TaskGroup() as task_group:
            executor_task = task_group.create_task(executor.start())
            try:
                print("Listening on example/talker")
                while True:
                    msg = await queue.get()
                    print(f"t={time.time()}  I heard: {msg.data}")
            finally:
                executor_task.cancel()


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
