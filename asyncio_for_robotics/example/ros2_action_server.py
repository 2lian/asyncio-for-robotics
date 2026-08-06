"""ros2_action_server.py — afor ActionServer example with Fibonacci.

Run alongside ros2_action_client.py:
    source /opt/ros/jazzy/setup.bash
    python3 ros2_action_server.py &
    python3 ros2_action_client.py

Important:
    ActionServer requires a MultiThreadedExecutor (enforced by auto_context).
    A SingleThreadedExecutor will deadlock because the execute callback blocks
    an executor thread for each goal.
"""

import asyncio
from contextlib import suppress

from example_interfaces.action import Fibonacci
from rclpy.action import CancelResponse
from rclpy.executors import MultiThreadedExecutor

import asyncio_for_robotics.ros2 as afor


async def handle_goal(goal_handle: afor.ActionGoalHandle) -> None:
    order = goal_handle.request.order
    print(f"Goal received: order={order}")

    if order < 0:
        print(f"Invalid order={order}, aborting")
        goal_handle.abort(Fibonacci.Result(sequence=[]))
        return

    seq = [0, 1]
    fb = Fibonacci.Feedback()
    for step in range(1, order):
        if goal_handle.is_cancel_requested:
            print(f"Goal cancelled at step {step}")
            goal_handle.canceled(Fibonacci.Result(sequence=seq))
            return
        seq.append(seq[-1] + seq[-2])
        fb.sequence = seq
        goal_handle.publish_feedback(fb)
        await asyncio.sleep(0.05)

    print(f"Goal succeeded: {seq}")
    goal_handle.succeed(Fibonacci.Result(sequence=seq))


@afor.scoped
async def fib_server():
    server = afor.ActionServer(
        Fibonacci,
        "fibonacci",
        cancel_callback=lambda _: CancelResponse.ACCEPT,
    )
    print("Fibonacci action server ready, waiting for goals...")

    async for goal_handle in server.listen_reliable():
        afor.Scope.current().task_group.create_task(handle_goal(goal_handle))


if __name__ == "__main__":
    # ActionServer requires MultiThreadedExecutor
    with afor.session_context(afor.ThreadedSession(executor=MultiThreadedExecutor)):
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(fib_server())
