import pytest

pytest.importorskip("rclpy")

import asyncio
import logging
from types import SimpleNamespace
from typing import Any, AsyncGenerator, Generator

from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from rclpy.action import CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

import asyncio_for_robotics.ros2 as afor
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.ros2.action import (
    ActionAborted,
    ActionCanceled,
    ActionRejected,
)
from asyncio_for_robotics.ros2.session import ThreadedSession

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")

_action_counter = 0


@pytest.fixture
def action_name() -> str:
    # Unique name per test: reusing one action name lets a not-yet-torn-down
    # server from the previous test answer goals with empty results (flaky).
    global _action_counter
    _action_counter += 1
    return f"test/fibonacci_action_{_action_counter}"

# order < 0           → server aborts immediately
# order == REJECT_ORDER → goal_callback rejects the goal
ABORT_ORDER = -1
REJECT_ORDER = -2


# ── Fixtures ──────────────────────────────────────────────────────────────────


@pytest.fixture(scope="module", autouse=True)
def session() -> Generator[afor.BaseSession, Any, Any]:
    # MultiThreadedExecutor is required: ActionServer._execute blocks an executor thread
    # for each goal, so SingleThreadedExecutor cannot process cancel callbacks or parallel goals.
    with afor.session_context(ThreadedSession(executor=MultiThreadedExecutor)) as ses:
        yield ses


async def _handle_goal(goal_handle: afor.ActionGoalHandle) -> None:
    order = goal_handle.request.order
    if order <= ABORT_ORDER:
        goal_handle.abort(Fibonacci.Result(sequence=[]))
        return
    seq = [0, 1]
    fb = Fibonacci.Feedback()
    for _ in range(1, order):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled(Fibonacci.Result(sequence=seq))
            return
        seq.append(seq[-1] + seq[-2])
        fb.sequence = seq
        goal_handle.publish_feedback(fb)
        await asyncio.sleep(0.05)
    goal_handle.succeed(Fibonacci.Result(sequence=seq))


@pytest.fixture
async def server(
    session: afor.BaseSession, action_name: str
) -> AsyncGenerator[afor.ActionServer, Any]:
    srv = afor.ActionServer(
        Fibonacci,
        action_name,
        goal_callback=lambda req: (
            GoalResponse.REJECT if req.order == REJECT_ORDER else GoalResponse.ACCEPT
        ),
        cancel_callback=lambda _: CancelResponse.ACCEPT,
    )

    async def serve():
        async for goal_handle in srv.listen_reliable():
            asyncio.create_task(_handle_goal(goal_handle))

    task = asyncio.create_task(serve())
    yield srv
    task.cancel()
    srv.close()


@pytest.fixture
async def client(
    session: afor.BaseSession, action_name: str
) -> AsyncGenerator[afor.ActionClient, Any]:
    c = afor.ActionClient(Fibonacci, action_name)
    yield c
    c.close()


# ── Tests ─────────────────────────────────────────────────────────────────────


async def test_wait_for_server_timeout(client: afor.ActionClient):
    """wait_for_server() times out when no server is available."""
    result = await afor.soft_wait_for(client.wait_for_server(), 0.5)
    assert isinstance(result, TimeoutError)


async def test_wait_for_server(server: afor.ActionServer, client: afor.ActionClient):
    """wait_for_server() returns successfully when a server is available."""
    result = await afor.soft_wait_for(client.wait_for_server(), 2)
    assert not isinstance(result, TimeoutError)


async def test_call_returns_result(
    server: afor.ActionServer, client: afor.ActionClient
):
    """call() sends a goal and returns the final result."""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    result = await afor.soft_wait_for(client.call(Fibonacci.Goal(order=5)), 3)
    assert not isinstance(result, TimeoutError)
    assert list(result.sequence) == [0, 1, 1, 2, 3, 5]


async def test_call_server_timeout(client: afor.ActionClient):
    """call() can time out while waiting for the action server."""
    with pytest.raises(asyncio.TimeoutError):
        await client.call(Fibonacci.Goal(order=5), server_timeout=0.1)


async def test_call_result_timeout(
    server: afor.ActionServer, client: afor.ActionClient
):
    """call() can time out while waiting for an accepted goal to finish."""
    with pytest.raises(asyncio.TimeoutError):
        await client.call(
            Fibonacci.Goal(order=30),
            server_timeout=2,
            result_timeout=0.1,
        )


async def test_late_result_after_cancelled_future_is_ignored():
    """A ROS result arriving after asyncio cancellation must not log InvalidStateError."""
    loop = asyncio.get_running_loop()
    gh = afor.ClientGoalHandle(loop, scope=None)
    gh.result.cancel()

    ros_result_future = loop.create_future()
    ros_result_future.set_result(
        SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=Fibonacci.Result(sequence=[0, 1]),
        )
    )

    gh._on_result(ros_result_future)
    assert gh.result.cancelled()


async def test_feedback_streaming(server: afor.ActionServer, client: afor.ActionClient):
    """send_goal() + feedback_until_result() delivers feedback, then result resolves."""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    gh = client.send_goal(Fibonacci.Goal(order=5))
    assert await gh.accepted

    feedback_seqs: list[list[int]] = []
    async for fb in gh.feedback_until_result():
        feedback_seqs.append(list(fb.sequence))

    result = await gh.result
    assert len(feedback_seqs) > 0, "expected at least one feedback message"
    assert list(result.sequence) == [0, 1, 1, 2, 3, 5]
    # each feedback step appends one element
    for i in range(1, len(feedback_seqs)):
        assert len(feedback_seqs[i]) > len(feedback_seqs[i - 1])


async def test_cancel(server: afor.ActionServer, client: afor.ActionClient):
    """Cancelling mid-flight raises ActionCanceled with a partial result."""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    gh = client.send_goal(Fibonacci.Goal(order=30))
    assert await gh.accepted

    async def cancel_later():
        await asyncio.sleep(0.3)
        await gh.cancel_goal()

    cancel_task = asyncio.create_task(cancel_later())
    feedback_count = 0
    try:
        async for _ in gh.feedback_until_result():
            feedback_count += 1
        with pytest.raises(ActionCanceled) as excinfo:
            await gh.result
        assert feedback_count > 0, "expected at least one feedback before cancel"
        assert len(excinfo.value.result.sequence) > 0, (
            "expected a non-empty partial result"
        )
    finally:
        await cancel_task


async def test_abort(server: afor.ActionServer, client: afor.ActionClient):
    """ActionAborted is raised when awaiting the result of an aborted goal."""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    gh = client.send_goal(Fibonacci.Goal(order=ABORT_ORDER))
    assert await gh.accepted

    with pytest.raises(ActionAborted) as excinfo:
        await gh.result
    assert list(excinfo.value.result.sequence) == []


async def test_reject(server: afor.ActionServer, client: afor.ActionClient):
    """gh.accepted is False and gh.result raises when goal_callback returns REJECT."""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    gh = client.send_goal(Fibonacci.Goal(order=REJECT_ORDER))
    assert not await gh.accepted, (
        "goal_callback returned REJECT so accepted should be False"
    )
    with pytest.raises(ActionRejected):
        await gh.result


async def test_call_raises_on_reject(
    server: afor.ActionServer, client: afor.ActionClient
):
    """call() raises ActionRejected when the goal is rejected."""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    with pytest.raises(ActionRejected):
        await client.call(Fibonacci.Goal(order=REJECT_ORDER))


async def test_concurrent_goals(server: afor.ActionServer, client: afor.ActionClient):
    """Multiple goals on the same server are processed in parallel, finishing faster than sequential."""
    import time

    await afor.soft_wait_for(client.wait_for_server(), 2)

    async def call(order: int) -> list[int]:
        result = await client.call(Fibonacci.Goal(order=order))
        return list(result.sequence)

    start = time.monotonic()
    r5, r8, r10 = await asyncio.gather(call(5), call(8), call(10))
    elapsed = time.monotonic() - start

    sequential_est = (4 + 7 + 9) * 0.05  # steps per goal × sleep interval
    assert elapsed < sequential_est * 0.9, (
        f"expected parallel execution but got near-sequential: elapsed={elapsed:.2f}s, estimate={sequential_est:.2f}s"
    )
    assert r5 == [0, 1, 1, 2, 3, 5]
    assert r10[-1] == 55
