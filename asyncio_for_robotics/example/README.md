# asyncio_for_robotics Examples

This directory contains runtime examples demonstrating the usage of
`asyncio_for_robotics` with ROS 2 and Zenoh.

The examples are very verbose and documented in the code.

These are designed to show:
- How to initialize and use sessions (`auto_session`) for ROS 2 and Zenoh.
- How to create publishers and subscribers safely.
- How to consume messages asynchronously using different methods.
- How to handle background publishing tasks while processing subscriber data.
- How to cleanly shutdown sessions to exit Python processes correctly.

---

## Example Scripts

### 0. Source necessary things

```bash
. /opt/ros/$ROS_DISTRO/setup.bash # (if you use ros)
. .venv/bin/activate # (if you use a venv)
```

### 1. `ros2_discusion.py` or `zenoh_discusion.py` 

- Runtime verbose example
- Creates and destroys session, explaining along the way
- Uses all methods to consume messages on a datastream.
- Shows difference in output between methods

```bash
python3 -m python3 -m asyncio_for_robotics.example.zenoh_discusion
```
or 
```bash
python3 -m python3 -m asyncio_for_robotics.example.ros2_discusion
```

### 2. `ros2_talker.py` and `ros2_listener.py`

- Very simple talker/listener

```bash
# Terminal #1
python3 -m python3 -m asyncio_for_robotics.example.ros2_talker

# Terminal #2
python3 -m python3 -m asyncio_for_robotics.example.ros2_listener
```

### 3. `ros2_double_talker.py` and `ros2_double_listener.py`

- Publishes and listens to 2 data stream simultaneously.
- Demonstrates more advanced used of asyncio.
- Demonstrates multiple async data and event manipulation

```bash
# Terminal #1
python3 -m python3 -m asyncio_for_robotics.example.ros2_double_talker

# Terminal #2
python3 -m python3 -m asyncio_for_robotics.example.ros2_double_listener
```
