"""ros2_action_client.py — afor ActionClient example with Fibonacci.

Shows three usage patterns:
  1. call()      — fire and wait for the final result
  2. send_goal() — stream feedback with ``async for``
  3. cancel      — cancel a long-running goal mid-flight

Run alongside ros2_action_server.py:
    source /opt/ros/jazzy/setup.bash
    python3 ros2_action_server.py &
    python3 ros2_action_client.py
"""

import asyncio
from contextlib import suppress

from example_interfaces.action import Fibonacci

import asyncio_for_robotics.ros2 as afor


@afor.scoped
async def fib_client():
    client = afor.ActionClient(Fibonacci, "fibonacci")

    print("Waiting for action server...")
    await client.wait_for_server()
    print("Server ready.\n")

    # ── 1. call() : ゴールを送って結果を待つ ──────────────────────────────────
    print("[1] call(order=8)")
    result = await client.call(Fibonacci.Goal(order=8))
    print(f"    result: {list(result.sequence)}\n")

    # ── 2. send_goal() : feedback を受け取りながら結果を待つ ──────────────────
    print("[2] send_goal(order=8) with feedback streaming")
    goal_handle = await client.send_goal(Fibonacci.Goal(order=8))
    async for feedback in goal_handle:
        print(f"    feedback: {list(feedback.sequence)}")
    print(f"    result:   {list(goal_handle.result.sequence)}\n")

    # ── 3. cancel : 実行中にゴールをキャンセルする ───────────────────────────
    print("[3] send_goal(order=30) then cancel after 0.3s")
    goal_handle = await client.send_goal(Fibonacci.Goal(order=30))

    async def cancel_later():
        await asyncio.sleep(0.3)
        await goal_handle.cancel_goal()

    asyncio.create_task(cancel_later())

    try:
        async for feedback in goal_handle:
            print(f"    feedback: {list(feedback.sequence)}")
    except afor.ActionCanceled as e:
        print(f"    cancelled — partial result: {list(e.result.sequence)}\n")


if __name__ == "__main__":
    with afor.auto_context():
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(fib_client())
