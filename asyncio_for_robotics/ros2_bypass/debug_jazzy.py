import asyncio
import threading
from contextlib import ExitStack, suppress
from queue import Empty, Queue
from typing import Any, Callable

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from rclpy.signals import SignalHandlerOptions
from rclpy.type_support import check_is_valid_msg_type
from std_msgs.msg import String

from ..core import BaseSub

MessageInfo = Any

_ENTITY_TYPES = (
    (_rclpy.Subscription, "subscription", "add_subscription"),
    (_rclpy.GuardCondition, "guard_condition", "add_guard_condition"),
    (_rclpy.Timer, "timer", "add_timer"),
    (_rclpy.Client, "client", "add_client"),
    (_rclpy.Service, "service", "add_service"),
    (_rclpy.EventHandle, "event", "add_event"),
)


class WaitSetReactor:
    def __init__(self, node_ctx, loop: asyncio.AbstractEventLoop):
        self.loop = loop
        self.entity_update_q = Queue()
        self.callbacks: dict[Any, Callable[[], None]] = {}

        self._node_ctx = node_ctx
        self._entities = set()
        self._wait_indices = {}
        self._in_flight = set()
        self._in_flight_lock = threading.Lock()
        self._stopping = threading.Event()
        self._wait_set = None
        self._thread_task: asyncio.Task[None] | None = None

        with node_ctx.handle:
            self._wake_guard = _rclpy.GuardCondition(node_ctx.handle)
        self.entity_update_q.put(("add", self._wake_guard))

    @staticmethod
    def _entity_info(entity):
        for entity_type, ready_kind, add_method in _ENTITY_TYPES:
            if isinstance(entity, entity_type):
                return ready_kind, add_method
        raise TypeError(f"Unsupported wait-set entity: {type(entity)!r}")

    def _react(self, awake_entities):
        try:
            for entity in awake_entities:
                callback = self.callbacks.get(entity)
                if callback is not None:
                    callback()
        finally:
            with self._in_flight_lock:
                self._in_flight.difference_update(awake_entities)
            if not self._stopping.is_set():
                self.wakeup()

    def _refresh_ws_entities(self):
        assert self._wait_set is not None
        self._wait_set.clear_entities()

        with self._in_flight_lock:
            active_entities = self._entities - self._in_flight

        self._wait_indices.clear()
        for entity in active_entities:
            ready_kind, add_method = self._entity_info(entity)
            index = getattr(self._wait_set, add_method)(entity)
            self._wait_indices[entity] = (ready_kind, index)

    def _rebuild_waitset(self):
        with suppress(Empty):
            while True:
                operation, entity = self.entity_update_q.get_nowait()
                if operation == "add":
                    self._entity_info(entity)
                    self._entities.add(entity)
                elif operation == "remove":
                    self._entities.discard(entity)
                else:
                    raise ValueError(f"Unknown entity operation: {operation!r}")

        entity_counts = [
            sum(isinstance(entity, entity_type) for entity in self._entities)
            for entity_type, _, _ in _ENTITY_TYPES
        ]

        with self._node_ctx.handle:
            new_wait_set = _rclpy.WaitSet(
                *entity_counts,
                self._node_ctx.handle,
            )

        old_wait_set = self._wait_set
        self._wait_set = new_wait_set
        if old_wait_set is not None:
            old_wait_set.destroy_when_not_in_use()

    def _get_awake_entities(self):
        assert self._wait_set is not None
        return [
            entity
            for entity, (ready_kind, index) in self._wait_indices.items()
            if self._wait_set.is_ready(ready_kind, index)
            and entity in self.callbacks
        ]

    def wakeup(self):
        with self._wake_guard:
            self._wake_guard.trigger_guard_condition()

    def spin_waitset(self):
        try:
            self._rebuild_waitset()
            while not self._stopping.is_set():
                self._refresh_ws_entities()
                assert self._wait_set is not None
                self._wait_set.wait(-1)

                # Read readiness before replacing the wait set.
                awake_entities = self._get_awake_entities()
                if self._stopping.is_set():
                    return

                if not self.entity_update_q.empty():
                    self._rebuild_waitset()

                with self._in_flight_lock:
                    awake_entities = [
                        entity
                        for entity in awake_entities
                        if entity not in self._in_flight
                    ]
                    self._in_flight.update(awake_entities)

                if awake_entities:
                    self.loop.call_soon_threadsafe(self._react, awake_entities)
        finally:
            wait_set = self._wait_set
            self._wait_set = None
            if wait_set is not None:
                wait_set.destroy_when_not_in_use()

    def start(self):
        if self._thread_task is not None:
            raise RuntimeError("WaitSetReactor is already running")
        self._thread_task = asyncio.create_task(asyncio.to_thread(self.spin_waitset))
        return self

    async def stop(self):
        if self._thread_task is None:
            return
        self._stopping.set()
        try:
            while not self._thread_task.done():
                self.wakeup()
                await asyncio.wait((self._thread_task,), timeout=0.05)
            await self._thread_task
        finally:
            self._wake_guard.destroy_when_not_in_use()


async def display(sub: BaseSub[tuple[String, MessageInfo]]):
    print("listening start")
    async for msg, info in sub.listen():
        print(f"msg: {msg}\ninfo: {info}")


async def main():
    with ExitStack() as es:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        es.callback(rclpy.try_shutdown)
        node = Node("afor_dbg")
        es.callback(node.destroy_node)

        check_is_valid_msg_type(String)
        ros_sub = _rclpy.Subscription(
            node.handle,
            String,
            "example/talker",
            qos_profile_default.get_c_qos_profile(),
        )
        es.callback(ros_sub.destroy_when_not_in_use)

        afor_sub: BaseSub[tuple[String, MessageInfo]] = BaseSub()

        def consume_one():
            data = ros_sub.take_message(String, False)
            while data is not None:
                afor_sub._input_data_guarded(data)
                data = ros_sub.take_message(String, False)

        reactor = WaitSetReactor(node.context, asyncio.get_running_loop())
        reactor.entity_update_q.put(("add", ros_sub))
        reactor.callbacks[ros_sub] = consume_one
        reactor.start()
        try:
            await display(afor_sub)
        finally:
            await reactor.stop()


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
