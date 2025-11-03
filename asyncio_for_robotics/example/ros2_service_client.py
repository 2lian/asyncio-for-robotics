import asyncio
from contextlib import suppress

import rclpy
from example_interfaces.srv import AddTwoInts

import asyncio_for_robotics.ros2 as afor

TOPIC = afor.TopicInfo("add_two_ints", AddTwoInts)


async def fibo_client_loop():
    client = afor.Client(*TOPIC.as_arg())
    a: int = 1
    b: int = 0
    while 1:
        print(f"Requesting: {a} + {b}")
        try:
            response = await asyncio.wait_for(
                client.call(AddTwoInts.Request(a=a, b=b)), timeout=3
            )
        except TimeoutError:
            print("No response, requesting again")
            continue
        b = a
        a = response.sum
        print(f"Got: {a}")
        await asyncio.sleep(0.5)




if __name__ == "__main__":
    rclpy.init()
    try:
        # suppress, just so we don't flood the terminal on exit
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(fibo_client_loop())  # starts asyncio executor
    finally:
        # cleanup. `finally` statment always executes
        afor.auto_session().close()
        rclpy.shutdown()
