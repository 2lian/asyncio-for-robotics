# Asyncio For Robotics
| Requirements | Compatibility | Tests |
|---|---|---|
| [![python](https://img.shields.io/pypi/pyversions/asyncio_for_robotics?logo=python&logoColor=white&label=Python&color=%20blue)](https://pypi.org/project/asyncio_for_robotics/)<br>[![mit](https://img.shields.io/badge/License-MIT-gold)](https://opensource.org/license/mit) | [![ros](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Jazzy-blue?logo=ros)](https://github.com/ros2)<br>[![zenoh](https://img.shields.io/badge/Zenoh-%3E%3D1.0-blue)](https://zenoh.io/) | [![Python](https://github.com/2lian/asyncio-for-robotics/actions/workflows/python-pytest.yml/badge.svg)](https://github.com/2lian/asyncio-for-robotics/actions/workflows/python-pytest.yml)<br>[![ROS 2](https://github.com/2lian/asyncio-for-robotics/actions/workflows/ros-pytest.yml/badge.svg)](https://github.com/2lian/asyncio-for-robotics/actions/workflows/ros-pytest.yml) |

The Asyncio For Robotics (`afor`) library makes `asyncio` usable with ROS 2, Zenoh and more, letting you write linear, testable, and non-blocking Python code.

- Better syntax.
- Only native python: Better docs and support.
- Simplifies testing.

*Will this make my code slower?* [Likely not.](https://github.com/2lian/asyncio-for-robotics/tree/main/README.md#about-speed)

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
pip install asyncio_for_robotics eclipse-zenoh
```

## Read more

- [Detailed ROS 2 tutorial](https://github.com/2lian/asyncio-for-robotics/blob/main/using_with_ros.md)
- [Detailed examples](https://github.com/2lian/asyncio-for-robotics/blob/main/asyncio_for_robotics/example)
  - [no talking 🦍 show me code 🦍](https://github.com/2lian/asyncio-for-robotics/blob/main/asyncio_for_robotics/example/ros2_pubsub.py)
- [Cross-Platform deployment even with ROS](https://github.com/2lian/asyncio-for-robotics/blob/main/cross_platform.md) [![Pixi Badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)
- [Usage for software testing](https://github.com/2lian/asyncio-for-robotics/blob/main/tests)

## Available interfaces:
- **Rate**: Every tick of a clock. (native)
- **TextIO**: `stdout` lines of a `Popen` process (and other `TextIO` files). (native)
- **ROS 2**: Subscriber, Service Client, Service Server.
- **Zenoh**: Subscriber.
- [Implement your own interface!](https://github.com/2lian/asyncio-for-robotics/blob/main/own_proto_example.md)

> [!TIP]
> An interface is not required for every operation. ROS 2 native publishers and
> nodes work just fine. Furthermore, advanced behavior can be composed of
> generic `afor` object (see [ROS2 Event Callback
> Example](./asyncio_for_robotics/example/ros2_event_callback.py)).

### Additional Projects and Interfaces
- **[gogo_keyboard](https://github.com/2lian/gogo_keyboard)**: Subscribe to keyboard key presses and release.
- **[asyncio_gazebo](https://github.com/2lian/asyncio-gazebo)**: Subscribe to Gazebo transport.

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

# Get the latest message
latest = await sub.wait_for_value()

# Get a new message
new = await sub.wait_for_new()

# Get the next message received
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

### Apply pre-processing to a data-stream

Application:
- Parse payload of different transport into a common type.

```python
# ROS2 String type afor subscriber
inner_sub: Sub[String] = afor.ros2.Sub(String, "topic_name")
# converted into a subscriber generating python `str`
ros2str_func = lambda msg: msg.data
sub: Sub[str] = afor.ConverterSub(sub=inner_sub, convert_func=ros2str_func)
```

## About Speed

The obvious question is whether this adds latency compared to native ROS 2.

In this benchmark, the answer is: a little on ROS 2, very little on Zenoh.

- On ROS 2 Jazzy with `SingleThreadedExecutor` and `rmw_zenoh_cpp`, trip
  duration increases from 70 μs to 140 μs when using afor, for an added
  overhead of about 70 μs.
- On Zenoh, `afor` adds only about 7 μs over the native path.
  This suggests that most of the ROS 2 cost comes from cross-thread operations
  with the `rclpy` machinery.
- Even with this added overhead, Zenoh + `afor` remains about an order of
  magnitude faster than ROS 2 + `rclpy` in this benchmark.
- For many Python robotics applications, an extra few dozen microseconds is
  negligible relative to the benefits of a uniform `asyncio` interface.
- This benchmark measures latency; it **does not measure throughput**. A *2x*
  latency increase, does not imply a *2x* throughput decrease.

| Backend         | Interface     | Latency (μs) |
| :---------      | :------------ | -----------: | 
| No-backend      | `afor`        |            4 |
| Zenoh           | *native*      |            3 |
| Zenoh           | `afor`        |           10 |
| ROS Single Thrd | *native*      |           70 |
| ROS Single Thrd | `afor`        |          136 |
| ROS Multi Thrd  | *native*      |          125 |
| ROS Multi Thrd  | `afor`        |          217 |
| [`ros_loop` Method](https://github.com/m2-farzan/ros2-asyncio)  | [`afor`](https://github.com/2lian/asyncio-for-robotics/blob/main/asyncio_for_robotics/ros2/session.py#L211C7-L211C25)        |          280 |

Benchmark code is available at
[https://github.com/2lian/afor_benchmarks](https://github.com/2lian/afor_benchmarks).
The benchmark uses two pub/sub pairs that continuously echo messages on
localhost, with a single participant and a local Zenoh router.

