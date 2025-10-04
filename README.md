# Asyncio For Robotics (WIP)

ROS2, Zenoh, and other robotic communication systems through asyncio python.

- Make your code linear, easy to write, easy to follow.
- No gigantic spaghetti of callback and future.
- Nobody responsible for spinning an executor.
- Fantastic for software testing where many small tasks and checks need to be
  written and run.
- Ability to `asyncio.sleep` while other tasks are continuing.
- No dependencies, just native python with great support.

*Will this make my code slower?* No.

*Will this make my code faster?* No. But I am sure that `asyncio` will help you understand you code better, and you will make it faster.

## Install

### For ROS2

```bash
pip install git+https://github.com/2lian/asyncio-for-robotics.git[ros2]
```

### For Zenoh

```bash
pip install git+https://github.com/2lian/asyncio-for-robotics.git[zenoh]
```

## Code sample

Syntax is identical between ROS2 and Zenoh.

### Wait for messages one by one

Application:
- Get the latest sensor data
- Get clock value
- Wait for trigger
- Wait for system to be operational

```python
sub = Sub("afr/example")

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
assert not isinstance(data, TimeoutError), f"Failed to get new data in under 1 second"

# Process a codeblock with a timeout
async with soft_timeout(1) as did_timeout:
    sum = 0
    total = 0
    async for msg in sub.listen_reliable():
        number = process(msg)
        sum += number
        total += 1

last_second_average = sum/total
```
