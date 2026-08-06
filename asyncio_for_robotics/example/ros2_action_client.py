"""ros2_action_client.py — afor ActionClient example with Fibonacci.

Shows four usage patterns:
  1. call()                — fire and wait for the final result
  2. feedback helper       — stream feedback until the result arrives
  3. raw feedback sub      — handle ActionFeedbackDone yourself
  4. cancel                — cancel a long-running goal mid-flight

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

    # ── 1. call() : send a goal and wait for the final result ─────────────────
    print("[1] call(order=8)")
    result = await client.call(Fibonacci.Goal(order=8))
    print(f"    result: {list(result.sequence)}\n")

    # ── 2. feedback_until_result() : simple feedback streaming ────────────────
    print("[2] send_goal(order=8) with feedback_until_result()")
    goal_handle = client.send_goal(Fibonacci.Goal(order=8))
    await goal_handle.accepted
    async for feedback in goal_handle.feedback_until_result():
        print(f"    feedback: {list(feedback.sequence)}")
    result = await goal_handle.result
    print(f"    result:   {list(result.sequence)}\n")

    # ── 3. feedback.listen() : full control over the done marker ──────────────
    print("[3] send_goal(order=8) with raw feedback.listen()")
    goal_handle = client.send_goal(Fibonacci.Goal(order=8))
    await goal_handle.accepted
    async for item in goal_handle.feedback.listen():
        if isinstance(item, afor.ActionFeedbackDone):
            break
        print(f"    feedback: {list(item.sequence)}")
    result = await goal_handle.result
    print(f"    result:   {list(result.sequence)}\n")

    # ── 4. cancel : cancel a goal mid-flight ──────────────────────────────────
    print("[4] send_goal(order=30) then cancel after 0.3s")
    goal_handle = client.send_goal(Fibonacci.Goal(order=30))
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
        print(f"    cancelled — partial result: {list(e.result.sequence)}\n")


if __name__ == "__main__":
    with afor.auto_context():
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(fib_client())
