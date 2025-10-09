# Asyncio For Robotics (WIP)

The Asyncio For Robotics (`afor`) library makes `asyncio` usable with ROS 2, Zenoh and more, letting you write linear, testable, and non-blocking Python code.

- Make your code linear, easy to write, easy to follow.
- No gigantic spaghetti of callback and future.
- Nobody responsible for spinning an executor.
- Fantastic for software testing with many small tasks and checks to be
  maintained and run.
- Ability to `asyncio.sleep` while other tasks are continuing.
- No dependencies, light weight, just native python.

*Will this make my code slower?* [No.](https://github.com/2lian/asyncio-for-robotics/tree/main?tab=readme-ov-file#about-speed)

*Will this make my code faster?* No. However `asyncio` will help YOU write
better, faster code.

## Install

### For ROS 2

Compatible with: `jazzy`, `humble` and newer. This library is pure python, without dependencies, so it easily installs anywhere.

```bash
pip install git+https://github.com/2lian/asyncio-for-robotics.git
```

### For Zenoh

```bash
pip install git+https://github.com/2lian/asyncio-for-robotics.git[zenoh]
```

## Read more

- [Usage with other ROS 2 tooling](./using_with_ros.md)
- [Detailed examples](./asyncio_for_robotics/example)
- [Usage for software testing](./tests)

## Code sample

Syntax is identical between ROS 2 and Zenoh.

### Wait for messages one by one

Application:
- Get the latest sensor data
- Get clock value
- Wait for trigger
- Wait for system to be operational

```python
sub = Sub("afor/example")

# get the latest message
latest = await sub.wait_for_value()

# get a new message
new = await sub.wait_for_new()

# get the next message received
next = await sub.wait_for_next()
```

### Continuously listen to a data stream

Application:
- Process a whole data stream
- React to changes in sensor data

```python
# Continuously process the latest messages
async for msg in sub.listen():
    status = process(msg)
    if status == DONE:
        break

# Continuously process all incoming messages
async for msg in sub.listen_reliable():
    status = process(msg)
    if status == DONE:
        break
```

### Process for the right amount of time

Application:
- Test if the system is responding as expected
- Run small tasks with small and local code

```python
# Listen with a timeout
data = await soft_wait_for(sub.wait_for_new(), timeout=1)
if isinstance(data, TimeoutError):
    pytest.fail(f"Failed to get new data in under 1 second")


# Process a codeblock with a timeout
async with soft_timeout(1):
    sum = 0
    total = 0
    async for msg in sub.listen_reliable():
        number = process(msg)
        sum += number
        total += 1

last_second_average = sum/total
assert last_second_average == pytest.approx(expected_average)
```

## About Speed

The inevitable question: *“But isn’t this slower than the ROS 2 executor? ROS 2 is the best!”*

Benchmark code is available in [`./tests/bench/`](tests/bench/), it consists in two pairs of pub/sub infinitely echoing a message (using one single node). The messaging rate, thus measures the request to response latency. 

| With `afor`  | Transport | Executor                        | | Frequency (kHz) | Latency (ms) |
|:----------:|:----------|:----------------------------------|-|---------:|---------:|
| ✔️         | Zenoh     | None                              | | **95** | **0.01** |
| ✔️         | ROS 2     | [Experimental Asyncio](https://github.com/ros2/rclpy/pull/1399)              | | **17** | **0.06** |
| ❌         | ROS 2     | [Experimental Asyncio](https://github.com/ros2/rclpy/pull/1399)              | | 13 | 0.08 |
| ❌         | ROS 2     | SingleThreaded                    | | 9 | 0.11 |
| ✔️         | ROS 2     | SingleThreaded                    | | **7**  | **0.15** |
| ✔️         | ROS 2     | MultiThreaded                     | | **3**  | **0.3** |
| ❌         | ROS 2     | MultiThreaded                     | | **3**  | **0.3** |
| ✔️         | ROS 2     | [`ros_loop` Method](https://github.com/m2-farzan/ros2-asyncio)                     | | 3  | 0.3 |


In short: `rclpy`'s executor is the bottleneck. 
- Comparing to the best ROS 2 Jazzy can do (`SingleThreadedExecutor`), `afor` increases latency from 110us to 150us.
- Comparing to other execution methods, `afor` is equivalent if not faster.
- If you find it slow, you should use C++ or Zenoh (or contribute to this repo?).

Details:
- `uvloop` was used, replacing the asyncio executor (more or less doubles the performances for Zenoh)
- RMW was set to `rmw_zenoh_cpp`
- ROS2 benchmarks uses `afor`'s `ros2.ThreadedSession` (this is the default in `afor`). 
- Only the Benchmark of the [`ros_loop` method](https://github.com/m2-farzan/ros2-asyncio) uses `afor`'s second type of session: `ros2.SynchronousSession`.
- ROS 2 executors can easily be changed in `afor` when creating a session.
- The experimental `AsyncioExecutor` PR on ros rolling by nadavelkabets is incredible [https://github.com/ros2/rclpy/pull/1399](https://github.com/ros2/rclpy/pull/1399). Maybe I will add proper support for it (but only a few will want to use an unmerged experimental PR of ROS 2 rolling).
- If there is interest in those benchmarks I will improve them, so others can run them all easily.

Analysis:
- Zenoh is extremely fast, proving that `afor` is not the bottleneck.
- This `AsyncioExecutor` having better perf when using `afor` is interesting, because `afor` does not bypass code.
  - I think this is due to `AsyncioExecutor` having some overhead that affects its own callback.
  - Without `afor` the ROS 2 callback executes some code and publishes.
  - With `afor` the ROS 2 callback returns immediately, and fully delegates execution to `asyncio`.
- The increase of latency on the `SingleThreaded` executors proves that getting data in and out of the `rclpy` executor and thread is the main bottleneck. 
  - `AsyncioExecutor` does not have such thread, thus can directly communicate.
  - Zenoh has its own thread, however it is built exclusively for multi-thread operations, without any executor. Thus achieves far superior performances.
- `MultiThreadedExecutor` is just famously slow.
- Very surprisingly, the well known `ros_loop` method detailed here [https://github.com/m2-farzan/ros2-asyncio](https://github.com/m2-farzan/ros2-asyncio) is slow.
