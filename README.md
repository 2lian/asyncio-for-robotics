# Asyncio For Robotics (WIP)

Use ROS 2, Zenoh, and other robotic communication systems through clean and powerful asyncio commands.

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

```python
sub = Sub("afr/example")

# get the latest message
latest = await sub.wait_for_value()

# get a new message
new = await sub.wait_for_new()

# get the next message received
next = await sub.wait_for_next()

# Continuously process the latest messages
async for msg in sub.listen():
    result = process(msg)
    if result == DONE:
        break

# Continuously process all incoming messages
async for msg in sub.listen_reliable():
    result = process(msg)
    if result == DONE:
        break
```
