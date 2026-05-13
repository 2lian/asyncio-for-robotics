import pytest

pytest.importorskip("rclpy")

import asyncio
import logging
from typing import Any, AsyncGenerator, Generator

from example_interfaces.action import Fibonacci
from rclpy.action import CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

import asyncio_for_robotics.ros2 as afor
from asyncio_for_robotics.core._logger import setup_logger
from asyncio_for_robotics.ros2.action import ActionAborted, ActionCanceled
from asyncio_for_robotics.ros2.session import ThreadedSession

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")

ACTION = "test/fibonacci_action"

# order < 0  → サーバーが即 abort する
# order == REJECT_ORDER → goal_callback が reject する
ABORT_ORDER = -1
REJECT_ORDER = -2


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module", autouse=True)
def session() -> Generator[afor.BaseSession, Any, Any]:
    # ActionServer の _execute は executor thread をブロックするため MultiThreadedExecutor が必須。
    # SingleThreadedExecutor だと cancel callback も並列ゴールも処理できない。
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
async def server(session: afor.BaseSession) -> AsyncGenerator[afor.ActionServer, Any]:
    srv = afor.ActionServer(
        Fibonacci,
        ACTION,
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
async def client(session: afor.BaseSession) -> AsyncGenerator[afor.ActionClient, Any]:
    c = afor.ActionClient(Fibonacci, ACTION)
    yield c
    c.close()


# ── Tests ─────────────────────────────────────────────────────────────────────

async def test_wait_for_server_timeout(client: afor.ActionClient):
    """サーバーが存在しないとき wait_for_server() はタイムアウトする。"""
    result = await afor.soft_wait_for(client.wait_for_server(), 0.5)
    assert isinstance(result, TimeoutError)


async def test_wait_for_server(
    server: afor.ActionServer, client: afor.ActionClient
):
    """サーバーが存在するとき wait_for_server() は正常に返る。"""
    result = await afor.soft_wait_for(client.wait_for_server(), 2)
    assert not isinstance(result, TimeoutError)


async def test_call_returns_result(
    server: afor.ActionServer, client: afor.ActionClient
):
    """call() がゴールを送信して最終結果を返す。"""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    result = await afor.soft_wait_for(client.call(Fibonacci.Goal(order=5)), 3)
    assert not isinstance(result, TimeoutError)
    assert list(result.sequence) == [0, 1, 1, 2, 3, 5]


async def test_feedback_streaming(
    server: afor.ActionServer, client: afor.ActionClient
):
    """send_goal() + async for でフィードバックが届き、ループ後に result が取れる。"""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    gh = await client.send_goal(Fibonacci.Goal(order=5))
    assert gh.accepted

    feedback_seqs: list[list[int]] = []
    async for fb in gh:
        feedback_seqs.append(list(fb.sequence))

    assert len(feedback_seqs) > 0, "少なくとも1件のフィードバックが届くはず"
    assert list(gh.result.sequence) == [0, 1, 1, 2, 3, 5]
    # フィードバックは毎ステップで伸びていく
    for i in range(1, len(feedback_seqs)):
        assert len(feedback_seqs[i]) > len(feedback_seqs[i - 1])


async def test_cancel(server: afor.ActionServer, client: afor.ActionClient):
    """途中でキャンセルすると ActionCanceled が上がり、部分結果が取れる。"""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    gh = await client.send_goal(Fibonacci.Goal(order=30))
    assert gh.accepted

    async def cancel_later():
        await asyncio.sleep(0.3)
        await gh.cancel_goal()

    cancel_task = asyncio.create_task(cancel_later())
    feedback_count = 0
    try:
        async for _ in gh:
            feedback_count += 1
        pytest.fail("ActionCanceled が上がるはず")
    except ActionCanceled as e:
        assert feedback_count > 0, "キャンセル前に少なくとも1件のフィードバックが届くはず"
        assert len(e.result.sequence) > 0, "部分結果が入っているはず"
    finally:
        await cancel_task


async def test_abort(server: afor.ActionServer, client: afor.ActionClient):
    """サーバーが abort() を呼ぶと ActionAborted が上がる。"""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    gh = await client.send_goal(Fibonacci.Goal(order=ABORT_ORDER))
    assert gh.accepted

    try:
        async for _ in gh:
            pass
        pytest.fail("ActionAborted が上がるはず")
    except ActionAborted as e:
        assert list(e.result.sequence) == []


async def test_reject(server: afor.ActionServer, client: afor.ActionClient):
    """goal_callback が REJECT を返すと gh.accepted=False になる。"""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    gh = await client.send_goal(Fibonacci.Goal(order=REJECT_ORDER))
    assert not gh.accepted, "goal_callback が REJECT を返したので accepted=False のはず"


async def test_call_raises_on_reject(
    server: afor.ActionServer, client: afor.ActionClient
):
    """reject されたゴールで call() を呼ぶと RuntimeError が上がる。"""
    await afor.soft_wait_for(client.wait_for_server(), 2)
    with pytest.raises(RuntimeError):
        await client.call(Fibonacci.Goal(order=REJECT_ORDER))


async def test_concurrent_goals(
    server: afor.ActionServer, client: afor.ActionClient
):
    """同一サーバーへの複数ゴールが並列処理され、逐次合計より速く終わる。"""
    import time

    await afor.soft_wait_for(client.wait_for_server(), 2)

    async def call(order: int) -> list[int]:
        result = await client.call(Fibonacci.Goal(order=order))
        return list(result.sequence)

    start = time.monotonic()
    r5, r8, r10 = await asyncio.gather(call(5), call(8), call(10))
    elapsed = time.monotonic() - start

    sequential_est = (4 + 7 + 9) * 0.05  # 各ゴールの step 数 × sleep
    assert elapsed < sequential_est * 0.9, (
        f"並列実行のはずが逐次に近い: elapsed={elapsed:.2f}s, estimate={sequential_est:.2f}s"
    )
    assert r5 == [0, 1, 1, 2, 3, 5]
    assert r10[-1] == 55
