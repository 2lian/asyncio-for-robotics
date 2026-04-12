import asyncio
from contextlib import suppress

from example_interfaces.srv import AddTwoInts

import asyncio_for_robotics.ros2 as afor

TOPIC = afor.TopicInfo("add_two_ints", AddTwoInts)


@afor.scoped
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


if __name__ == "__main__":
    with afor.auto_context():
        with suppress(KeyboardInterrupt, asyncio.CancelledError):
            asyncio.run(fibo_server())
