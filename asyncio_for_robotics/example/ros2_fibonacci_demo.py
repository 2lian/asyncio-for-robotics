"""ros2_fibonacci_demo.py — comprehensive asyncio patterns demo via Action.

Starts 4 Fibonacci action servers internally and runs 14 demos showing how
standard asyncio primitives compose naturally with ROS 2 actions.

  01 call             — simple request/result action call
  02 asyncio.sleep    — sleep runs concurrently while an action is in-flight
  03 asyncio.gather   — 3 goals dispatched in parallel on one server
  04 asyncio.Lock     — Lock serialises concurrent goals one-at-a-time
  05 asyncio.wait     — FIRST_COMPLETED returns as soon as the fastest goal finishes
  06 asyncio.wait_for — TimeoutError on a slow goal
  07 asyncio.Event    — Event gates a goal until a signal is set
  08 asyncio.Semaphore— Semaphore(2) caps concurrent goals at 2
  09 asyncio.Queue    — producer/consumer pipeline over actions
  10 feedback helper  — stream feedback until the result arrives
  11 feedback raw     — handle ActionFeedbackDone yourself
  12 cancel           — cancel a long-running goal mid-flight
  13 abort            — handle ActionAborted raised by the server
  14 multi-server     — asyncio.gather across 4 independent servers

Run:
    source /opt/ros/jazzy/setup.bash
    python3 ros2_fibonacci_demo.py
"""

import asyncio
import time
from contextlib import suppress

from example_interfaces.action import Fibonacci
from rclpy.action import CancelResponse
from rclpy.executors import MultiThreadedExecutor

import asyncio_for_robotics.ros2 as afor

# ── Server ────────────────────────────────────────────────────────────────────


async def _handle_goal(goal_handle: afor.ActionGoalHandle) -> None:
    order = goal_handle.request.order
    if order < 0:
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


async def _run_server(action_name: str) -> None:
    server = afor.ActionServer(
        Fibonacci,
        action_name,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
    )
    with suppress(asyncio.CancelledError):
        async for goal_handle in server.listen_reliable():
            afor.Scope.current().task_group.create_task(_handle_goal(goal_handle))


# ── Helper ────────────────────────────────────────────────────────────────────


async def fibonacci(
    client: afor.ActionClient,
    order: int,
    *,
    feedback_label: str | None = None,
) -> list[int]:
    goal_handle = client.send_goal(Fibonacci.Goal(order=order))
    if not await goal_handle.accepted:
        raise afor.ActionRejected("goal was rejected by server")

    async for feedback in goal_handle.feedback_until_result():
        if feedback_label is not None:
            print(f"    {feedback_label}: {list(feedback.sequence)}")

    result = await goal_handle.result
    return list(result.sequence)


# ── Demos 01–14 ───────────────────────────────────────────────────────────────


async def demo_01_call(c: afor.ActionClient) -> None:
    print("[01 call]  simple request/result action call (Does not consider feedback)")
    # server_timeout : wait up to 2s for the server to accept the goal
    # result_timeout : wait up to 2s for the result after acceptance
    result = await c.call(Fibonacci.Goal(order=8), server_timeout=2, result_timeout=2)
    print(f"    PASS  result={list(result.sequence)}\n")


async def demo_02_sleep(c: afor.ActionClient) -> None:
    print("[02 sleep]  sleep runs concurrently while action is in-flight")
    sleep_done = asyncio.Event()

    async def sleeper():
        await asyncio.sleep(0.3)
        sleep_done.set()

    asyncio.create_task(sleeper())
    result = await fibonacci(c, 10, feedback_label="feedback")  # ~0.5 s

    assert sleep_done.is_set()
    print(
        f"    PASS  result[-1]={result[-1]}, sleep fired before action returned\n")


async def demo_03_gather(c: afor.ActionClient) -> None:
    print("[03 gather]  3 goals run in parallel on one server")
    start = time.monotonic()
    r5, r8, r10 = await asyncio.gather(
        fibonacci(c, 5),
        fibonacci(c, 8),
        fibonacci(c, 10),
    )
    elapsed = time.monotonic() - start
    print(
        f"    PASS  elapsed={elapsed:.2f}s  results={r5[-1]}, {r8[-1]}, {r10[-1]}\n")


async def demo_04_lock(c: afor.ActionClient) -> None:
    print("[04 lock]  Lock serialises goals one-at-a-time")
    lock = asyncio.Lock()
    active = 0
    peak = 0

    async def locked_fib(order: int) -> list[int]:
        nonlocal active, peak
        async with lock:
            active += 1
            peak = max(peak, active)
            print(f"    lock acquired: active={active}")
            try:
                return await fibonacci(c, order)
            finally:
                active -= 1
                print(f"    lock released: active={active}")

    results = await asyncio.gather(locked_fib(5), locked_fib(5), locked_fib(5))
    assert peak == 1
    print(
        f"    PASS  peak concurrent={peak}, results={[r[-1] for r in results]}\n")


async def demo_05_wait_first(c: afor.ActionClient) -> None:
    print("[05 wait]  FIRST_COMPLETED returns when the fastest goal finishes")
    fast = asyncio.create_task(fibonacci(c, 3))
    slow = asyncio.create_task(fibonacci(c, 20))

    done, pending = await asyncio.wait(
        {fast, slow}, return_when=asyncio.FIRST_COMPLETED
    )

    assert fast in done
    for t in pending:
        t.cancel()
    print(f"    PASS  fast result={fast.result()[-1]}\n")


async def demo_06_wait_for(c: afor.ActionClient) -> None:
    print("[06 wait_for]  TimeoutError on a slow goal")
    try:
        await asyncio.wait_for(fibonacci(c, 30), timeout=0.2)
    except asyncio.TimeoutError:
        print("    PASS  TimeoutError raised as expected\n")


async def demo_07_event(c: afor.ActionClient) -> None:
    print("[07 event]  Event gates a goal until a signal is set")
    ready = asyncio.Event()

    async def producer():
        await asyncio.sleep(0.2)
        ready.set()

    async def consumer():
        await ready.wait()
        return await fibonacci(c, 5)

    _, result = await asyncio.gather(producer(), consumer())
    print(f"    PASS  result[-1]={result[-1]}\n")


async def demo_08_semaphore(c: afor.ActionClient) -> None:
    print("[08 semaphore]  Semaphore(2) caps concurrent goals at 2")
    sem = asyncio.Semaphore(2)
    concurrent = 0
    peak = 0

    async def limited(order: int) -> list[int]:
        nonlocal concurrent, peak
        async with sem:
            concurrent += 1
            peak = max(peak, concurrent)
            result = await fibonacci(c, order)
            concurrent -= 1
            return result

    await asyncio.gather(*(limited(5) for _ in range(5)))
    assert peak <= 2
    print(f"    PASS  peak concurrent={peak} (limit=2)\n")


async def demo_09_queue(c: afor.ActionClient) -> None:
    print("[09 queue]  producer/consumer pipeline over actions")
    queue: asyncio.Queue[int | None] = asyncio.Queue()
    results: list[int] = []

    async def producer():
        for order in [3, 5, 7, 10]:
            await queue.put(order)
        await queue.put(None)

    async def consumer():
        while True:
            order = await queue.get()
            if order is None:
                break
            result = await fibonacci(c, order)
            results.append(result[-1])

    await asyncio.gather(producer(), consumer())
    print(f"    PASS  results={results}\n")


async def demo_10_feedback(c: afor.ActionClient) -> None:
    print("[10 feedback helper]  stream feedback until the result arrives")
    goal_handle = c.send_goal(Fibonacci.Goal(order=8))
    await goal_handle.accepted

    async for feedback in goal_handle.feedback_until_result():
        print(f"    feedback: {list(feedback.sequence)}")

    result = await goal_handle.result
    print(f"    PASS  result={list(result.sequence)}\n")


async def demo_11_feedback_raw(c: afor.ActionClient) -> None:
    print("[11 feedback raw]  handle ActionFeedbackDone yourself")
    goal_handle = c.send_goal(Fibonacci.Goal(order=8))
    await goal_handle.accepted

    async for item in goal_handle.feedback.listen():
        if isinstance(item, afor.ActionFeedbackDone):
            break
        print(f"    feedback: {list(item.sequence)}")

    result = await goal_handle.result
    print(f"    PASS  result={list(result.sequence)}\n")


async def demo_12_cancel(c: afor.ActionClient) -> None:
    print("[12 cancel]  cancel a long-running goal mid-flight")
    goal_handle = c.send_goal(Fibonacci.Goal(order=30))
    await goal_handle.accepted

    async def cancel_later():
        await asyncio.sleep(0.3)
        await goal_handle.cancel_goal()

    asyncio.create_task(cancel_later())

    try:
        async for feedback in goal_handle.feedback_until_result():
            print(f"    feedback: {list(feedback.sequence)}")
        await goal_handle.result
    except afor.ActionCanceled as e:
        print(
            f"    PASS  cancelled, partial result={list(e.result.sequence)}\n")


async def demo_13_abort(c: afor.ActionClient) -> None:
    print("[13 abort]  handle ActionAborted raised by the server")
    goal_handle = c.send_goal(Fibonacci.Goal(order=-1))
    await goal_handle.accepted

    try:
        async for _ in goal_handle.feedback_until_result():
            pass
        await goal_handle.result
    except afor.ActionAborted as e:
        print(f"    PASS  server aborted, result={list(e.result.sequence)}\n")


async def demo_14_multi_server(
    c0: afor.ActionClient,
    c1: afor.ActionClient,
    c2: afor.ActionClient,
    c3: afor.ActionClient,
) -> None:
    print("[14 multi-server]  asyncio.gather across 4 independent servers")
    start = time.monotonic()

    r0, r1, r2, r3 = await asyncio.gather(
        fibonacci(c0, 10),
        fibonacci(c1, 15),
        fibonacci(c2, 8),
        fibonacci(c3, 12),
    )

    elapsed = time.monotonic() - start
    print(
        f"    PASS  elapsed={elapsed:.2f}s\n"
        f"    server0→{r0[-1]}, server1→{r1[-1]}, server2→{r2[-1]}, server3→{r3[-1]}\n"
    )


# ── Entry point ───────────────────────────────────────────────────────────────


@afor.scoped
async def main() -> None:
    # Start 4 servers as background tasks
    server_tasks = [
        asyncio.create_task(_run_server(f"fibonacci_{i}")) for i in range(4)
    ]

    client_0 = afor.ActionClient(Fibonacci, "fibonacci_0")
    client_1 = afor.ActionClient(Fibonacci, "fibonacci_1")
    client_2 = afor.ActionClient(Fibonacci, "fibonacci_2")
    client_3 = afor.ActionClient(Fibonacci, "fibonacci_3")

    print("Waiting for servers...")
    await asyncio.gather(
        client_0.wait_for_server(),
        client_1.wait_for_server(),
        client_2.wait_for_server(),
        client_3.wait_for_server(),
    )
    print("All servers ready.\n")

    await demo_01_call(client_0)
    await demo_02_sleep(client_0)
    await demo_03_gather(client_0)
    await demo_04_lock(client_0)
    await demo_05_wait_first(client_0)
    await demo_06_wait_for(client_0)
    await demo_07_event(client_0)
    await demo_08_semaphore(client_0)
    await demo_09_queue(client_0)
    await demo_10_feedback(client_0)
    await demo_11_feedback_raw(client_0)
    await demo_12_cancel(client_0)
    await demo_13_abort(client_0)
    await demo_14_multi_server(client_0, client_1, client_2, client_3)

    print("All demos done!")

    for t in server_tasks:
        t.cancel()


if __name__ == "__main__":
    # MultiThreadedExecutor is required for concurrent goals and cancel support
    session = afor.ThreadedSession(executor=MultiThreadedExecutor)
    with afor.session_context(session):
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(main())
