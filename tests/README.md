# Asyncio For Robotics --- For Testing Applications

The tests in `asyncio_for_robotics` provide practical inspiration for setting up
similar tests in your ROS 2 or Zenoh applications.

You can install the dependencies required to run the tests with:

```bash
pip install git+https://github.com/2lian/asyncio-for-robotics.git[jazzy,zenoh,dev]
```

Important notes:
- Async test functions automatically use an `asyncio` event loop fixture.
- The fixture has `scope="function"`, meaning each test closes the previous
event loop and starts a new one.
- Because of this, you cannot share `asyncio` state (e.g.,
  asyncio_for_robotics.ros2.Sub, awaitables, tasks) between tests.
- ROS 2 or Zenoh sessions, however, can be shared across tests, since they do
  not rely on the `asyncio` loop.
