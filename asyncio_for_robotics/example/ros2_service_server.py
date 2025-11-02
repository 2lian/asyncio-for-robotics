import asyncio

import rclpy
from example_interfaces.srv import AddTwoInts

import asyncio_for_robotics.ros2 as afor

TOPIC = afor.TopicInfo("add_two_ints", AddTwoInts)


async def fibo_server():
    server = afor.Server(*TOPIC.as_arg())
    print(f"Awaiting request")
    async for responder in server.listen_reliable():
        # responder object contains the request
        a = responder.request.a
        b = responder.request.b
        # responder object also contains the response field that we should set
        responder.response.sum = a + b
        # we send the response
        responder.send()
        print(f"Replied: {a} + {b} = {a+b}")


def main():
    rclpy.init()
    afor.auto_session()
    try:
        asyncio.run(fibo_server())
    finally:
        afor.auto_session().close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
