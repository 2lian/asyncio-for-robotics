# Asyncio For Robotics (WIP)

The Asyncio For Robotics library makes asyncio usable with ROS 2, Zenoh and more, letting you write linear, testable, and non-blocking Python code.

- Make your code linear, easy to write, easy to follow.
- No gigantic spaghetti of callback and future.
- Nobody responsible for spinning an executor.
- Fantastic for software testing with many small tasks and checks to be
  maintained and run.
- Ability to `asyncio.sleep` while other tasks are continuing.
- No dependencies, light weight, just native python.

*Will this make my code slower?* No.

*Will this make my code faster?* No. However `asyncio` will help YOU write
better, faster code.

## Install

### For ROS 2

Compatible with: `jazzy`, `humble` and newer. This library is pure python, without dependencies so it easily installs anywhere.

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

- We’re in Python, time-critical processes should not live here.  
- `sleep` is accurate within ~1–5 ms, whether ROS 2 is involved or not.  
- Benchmarks on localhost (`./tests/bench/`) show:
  - Pure ROS 2 + SingleThreaded executor → latency: **0.1 ms**
  - Asyncio for Robotics + ROS 2 + SingleThreaded executor → latency: **0.2 ms**
  - Pure ROS 2 + MultiThreaded executor → latency: **0.4 ms**
  - Asyncio for Robotics + ROS 2 + MultiThreaded executor → latency: **0.4 ms**
- The nail on the coffin: **Asyncio for Robotics with Zenoh transport (no ros) is 10× faster**
  - → latency: **0.01 ms**

In short: `rclpy` is the bottleneck. If you find it slow, you should use C++ or Zenoh (or contribute to this repo?).
