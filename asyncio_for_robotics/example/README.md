# Asyncio For Robotics Examples

This directory contains runtime examples demonstrating the usage of
`asyncio_for_robotics` with ROS 2, Zenoh and the `stdout` of a ping command.

The examples are intentionally verbose and well-documented in the code.  

For readers familiar with ROS 2, these examples offer a chance to compare
how similar tasks can be implemented with `asyncio_for_robotics`. In
particular, `ros2_double_talker.py` and `ros2_double_listener.py` are the
most advanced, showcasing the power of asyncio for handling multiple
streams concurrently. Achieving the same functionality in plain ROS 2 would
require convoluted timers, callback chaining, and manual future management.

---

## Example Scripts

### 0. Source necessary environments

```bash
. /opt/ros/$ROS_DISTRO/setup.bash  # (if using ROS)
. .venv/bin/activate               # (if using a virtual environment)
```

### `ros2_discusion.py` or `zenoh_discusion.py` 

- Verbose runtime examples demonstrating session creation and cleanup
- Illustrates all subscriber methods for consuming messages on a data stream.
- Shows the behavioral differences between `wait_for_value`, `wait_for_new`,
`wait_for_next`, `listen`, and `listen_reliable`.

```bash
# Zenoh example
python3 -m asyncio_for_robotics.example.zenoh_discussion

# ROS 2 example
python3 -m asyncio_for_robotics.example.ros2_discussion
```

### `ros2_talker.py` and `ros2_listener.py`

- Simple talker/listener pair demonstrating basic async publishing and subscribing.

```bash
# Terminal #1
python3 -m asyncio_for_robotics.example.ros2_talker

# Terminal #2
python3 -m asyncio_for_robotics.example.ros2_listener
```

Or run both simultaneously in the same terminal using:
```bash
python3 -m asyncio_for_robotics.example.ros2_pubsub
```

### `ros2_double_talker.py` and `ros2_double_listener.py`

- Publishes and listens to two data stream simultaneously.
- Demonstrates advanced asyncio usage, including concurrent async tasks and
  event management.
- Ideal for understanding how asyncio_for_robotics can simplify complex
  multi-stream communication compared to traditional ROS 2 code.

```bash
# Terminal #1
python3 -m asyncio_for_robotics.example.ros2_double_talker

# Terminal #2
python3 -m asyncio_for_robotics.example.ros2_double_listener
```

### `ros2_service_client.py` and `ros2_service_server.py`

- Simple client / server implementing `add_two_int` example and the
  Fibonacci sequence.

```bash
# Terminal #1
python3 -m asyncio_for_robotics.example.ros2_service_server

# Terminal #2
python3 -m asyncio_for_robotics.example.ros2_service_client
```

### `ros2_event_callback.py`

- Uses a `BaseSub` to accumulate a generic datastream.
- `asyncio_for_robotics` subscriptions with ROS 2 subscription event callbacks
  (e.g. matched publishers, lost messages).
- Same method can work with other ROS2 objects (pub, sub, service, client...)

```bash
# Terminal #1
python3 -m asyncio_for_robotics.example.ros2_event_callback

# Terminal #2
ros2 topic pub /example std_msgs/msg/String "data: hey"
```

### `custom_stdout.py`

- Implements a custom subscriber capturing the `stdout` of `ping localhost`

```bash
# Terminal #1
python3 -m asyncio_for_robotics.example.custom_stdout
```
