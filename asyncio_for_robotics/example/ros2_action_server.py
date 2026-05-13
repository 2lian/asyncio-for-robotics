"""ros2_action_server.py — afor ActionServer example with Fibonacci.

Run:
    source /opt/ros/jazzy/setup.bash
    python3 ros2_action_server.py
"""

import asyncio
import time
from contextlib import suppress

from example_interfaces.action import Fibonacci

import asyncio_for_robotics.ros2 as afor


@afor.scoped
async def fib_server():
    server = afor.ActionServer(Fibonacci, "fibonacci")
    print("Fibonacci action server ready, waiting for goals...")

    async for goal_handle in server.listen_reliable():
        order = goal_handle.request.order
        print(f"Received goal: order={order}")

        seq = [0, 1]
        fb = Fibonacci.Feedback()
        for _ in range(1, order):
            if goal_handle.is_cancel_requested:
                print("Goal cancelled")
                result = Fibonacci.Result(sequence=seq)
                goal_handle.canceled(result)
                break
            seq.append(seq[-1] + seq[-2])
            fb.sequence = seq
            goal_handle.publish_feedback(fb)
            await asyncio.sleep(0.05)
        else:
            result = Fibonacci.Result(sequence=seq)
            goal_handle.succeed(result)
            print(f"Goal succeeded: last={seq[-1]}")


if __name__ == "__main__":
    from rclpy.executors import MultiThreadedExecutor

    with afor.auto_context():
        import rclpy
        # ActionServer requires MultiThreadedExecutor
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(fib_server())
