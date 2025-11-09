# Asyncio For Robotics --- For Testing Applications

The tests of `asyncio_for_robotics` are great practical examples!

For ROS 2 in particular, `test_ros2_pubsub_thrd.py` goes through every subscription method, then triggers and verifies their different outcome.

> [!NOTE]
> Install the dependencies required to run the tests with: `pip install asyncio_for_robotics[zenoh,dev]`

Notes about testing with asyncio:
- Async test functions automatically use an `asyncio` event loop fixture.
- The fixture has `scope="function"`, meaning each test closes the previous
event loop and starts a new one.
- Because of this, you cannot share `asyncio` state (e.g.,
  asyncio_for_robotics.ros2.Sub, awaitables, tasks) between tests.
- ROS 2 or Zenoh sessions, however, can be shared across tests, since they do
  not rely on the `asyncio` loop.
