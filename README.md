# Asyncio For Robotics
| Requirements | Compatibility | Tests |
|---|---|---|
| [![python](https://img.shields.io/pypi/pyversions/asyncio_for_robotics?logo=python&logoColor=white&label=Python&color=%20blue)](https://pypi.org/project/asyncio_for_robotics/)<br>[![mit](https://img.shields.io/badge/License-MIT-gold)](https://opensource.org/license/mit) | [![ros](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Jazzy-blue?logo=ros)](https://github.com/ros2)<br>[![zenoh](https://img.shields.io/badge/Zenoh-%3E%3D1.0-blue)](https://zenoh.io/) | [![Python](https://github.com/2lian/asyncio-for-robotics/actions/workflows/python-pytest.yml/badge.svg)](https://github.com/2lian/asyncio-for-robotics/actions/workflows/python-pytest.yml)<br>[![ROS 2](https://github.com/2lian/asyncio-for-robotics/actions/workflows/ros-pytest.yml/badge.svg)](https://github.com/2lian/asyncio-for-robotics/actions/workflows/ros-pytest.yml) |

The Asyncio For Robotics (`afor`) library makes `asyncio` usable with ROS 2, Zenoh and more, letting you write linear, testable, and non-blocking Python code.

- Better syntax.
- Only native python: Better docs and support.
- Simplifies testing.

*Will this make my code slower?* [No.](https://github.com/2lian/asyncio-for-robotics/tree/main/README.md#about-speed)

*Will this make my code faster?* No. However, `asyncio` will help YOU write
better, faster code.

> [!TIP]
> `asyncio_for_robotics` interfaces do not replace their primary interfaces! We add capabilities, giving you more choices, not less.


## Install

### Barebone

Compatible with ROS 2 (`jazzy`,`humble` and newer) out of the box. This library is pure python (>=3.10), so it installs easily.

```bash
pip install asyncio_for_robotics
```

### Along with Zenoh

```bash
pip install asyncio_for_robotics[zenoh]
```

## Read more

- [Detailed ROS 2 tutorial](https://github.com/2lian/asyncio-for-robotics/blob/main/using_with_ros.md)
- [Detailed examples](https://github.com/2lian/asyncio-for-robotics/blob/main/asyncio_for_robotics/example)
  - [no talking 🦍 show me code 🦍](https://github.com/2lian/asyncio-for-robotics/blob/main/asyncio_for_robotics/example/ros2_pubsub.py)
- [Cross-Platform deployment even with ROS](https://github.com/2lian/asyncio-for-robotics/blob/main/cross_platform.md) [![Pixi Badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)
- [Usage for software testing](https://github.com/2lian/asyncio-for-robotics/blob/main/tests)

### Available interfaces:
- **Rate**: Every tick of a clock. (native)
- **TextIO**: `stdout` lines of a `Popen` process (and other `TextIO` files). (native)
- **ROS 2**: Subscriber, Service Client, Service Server.
- **Zenoh**: Subscriber.
- [Implement your own interface!](https://github.com/2lian/asyncio-for-robotics/blob/main/own_proto_example.md)

> [!TIP]
> An interface is not required for every operation. ROS 2 native publishers and
> nodes work just fine. Furthermore, advanced behavior can be composed of
> generic `afor` object, this is done in our [ROS2 Event Callback
> Example](./asyncio_for_robotics/example/ros2_event_callback.py).

## Code sample

Syntax is identical between ROS 2, Zenoh, TextIO, Rate...

### Wait for messages one by one

Application:
- Get the latest sensor data
- Get clock value
- Wait for trigger
- Wait for next tick of the Rate
- Wait for system to be operational

```python
sub = afor.Sub(...)

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
- Execute on every tick of the Rate

```python
# Continuously process the latest messages
async for msg in sub.listen():
    status = foo(msg)
    if status == DONE:
        break

# Continuously process all incoming messages
async for msg in sub.listen_reliable():
    status = foo(msg)
    if status == DONE:
        break
```

### Improved Services / Queryable for ROS 2

> [!NOTE]
> This is only for ROS 2.

Application:
- Client request reply from a server.
- Servers can delay their response without blocking (not possible in native ROS 2)

```python
# Server is once again a afor subscriber, but generating responder objects
server = afor.Server(...)

# processes all requests.
# listen_reliable method is recommanded as it cannot skip requests
async for responder in server.listen_reliable():
    if responder.request == "PING!":
        reponder.response = "PONG!"
        await asyncio.sleep(...) # reply can be differed
        reponder.send()
    else:
        ... # reply is not necessary
```

```python
# the client implements a async call method
client = afor.Client(...)

response = await client.call("PING!")
```

### Process for the right amount of time

Application:
- Test if the system is responding as expected
- Run small tasks with small and local code

```python
# Listen with a timeout
data = await afor.soft_wait_for(sub.wait_for_new(), timeout=1)
if isinstance(data, TimeoutError):
    pytest.fail(f"Failed to get new data in under 1 second")


# Process a codeblock with a timeout
async with afor.soft_timeout(1):
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

In short: `rclpy`'s executor is the bottleneck. 
- Comparing to the best ROS 2 Jazzy can do (`SingleThreadedExecutor`), `afor` increases latency from 110us to 150us.
- Comparing to other execution methods, `afor` is equivalent if not faster.
- If you find it slow, you should use C++ or Zenoh (or contribute to this repo?).

Benchmark code is available in [`./tests/bench/`](https://github.com/2lian/asyncio-for-robotics/blob/main/tests/bench/), it consists in two pairs of pub/sub infinitely echoing a message (using one single node). The messaging rate, thus measures the request to response latency. 

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


Details:
- `uvloop` was used, replacing the asyncio executor (more or less doubles the performances for Zenoh)
- RMW was set to `rmw_zenoh_cpp`
- ROS2 benchmarks uses `afor`'s `ros2.ThreadedSession` (the default in `afor`). 
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
