# Using Sessions

`afor` sessions are simply a shortcut. When a session is active, `afor` uses it as default, so you do need to provide it every time.

- ROS sessions are composed of:
  - `rclpy`
  - node
  - executor
- ROS experimental sessions are exactly a `rclpy.experimental.AsyncNode`.
- Zenoh sessions are exactly a `zenoh.Session`.

## Main idea

Use a session context outside your `afor.Scope`. If you provide a session it will be set as default, else one will be created.

```python
with backend.auto_context(...):
    async with afor.Scope():
        ...
```

This means:

- leaving `Scope` destroys subscriptions, clients, servers, timers
- leaving the session context closes the transport session itself

> [!NOTE]
> For `afor.ros2_exp`, the session binds itself to the asyncio event loop, so you need to enter it from inside a async coroutine with `async with async_context(...):`.

## Resolution order

Every `afor` class, when instantiated, will look for the session to use in this order:

1. explicit `session=...` on a class constructor (most precise)
2. current session context (easiest)
3. create a global session (not recommended fallback)

## ROS

Convenience session: when the normal default session behavior is enough. It handles `rclpy.init`, `rclpy.Executor`, `rclpy.Node` and their shutdown.

> [!NOTE]
> Calling `auto_context` inside an already existing afor session context will create a new one (executor and node).

```python
import asyncio_for_robotics.ros2 as afor


with afor.auto_context(node="my_node"):
    async with afor.Scope():
        sub = afor.Sub(String, "/chatter")
```

Explicit session: when you want to build the session (node or executor) yourself and set it as default. It does not handle the shutdown of the provided session.

```python
import asyncio_for_robotics.ros2 as afor
import rclpy

rclpy.init()
my_node = Node(name="my_node")
my_session = afor.ThreadedSession(node=my_node)

with afor.auto_context(my_session):
    async with afor.Scope():
        sub = afor.Sub(String, "/chatter")
        client = afor.Client(MySrv, "/compute")

my_session.close()
rclpy.shutdown()
```

## Zenoh (similar to ROS 2)

```python
import zenoh
import asyncio_for_robotics.zenoh as afor

my_session = zenoh.open(zenoh.Config())
# or
# my_session = None

with afor.auto_context(my_session):
    async with afor.Scope():
        sub = afor.Sub("demo/**")
        ...
```

## Getting the current active node / session to use it yourself

The node or session is not hidden from you.

ROS example:

```python
my_session = afor.auto_session()  # creates a session if resolution fails
my_session = afor.current_session()  # raises Exception is resolution fails
with my_session.lock() as node:
    pub = node.create_publisher(String, "/chatter", 10)
```

ROS experimental example:

```python
my_session = afor.auto_session()  # creates a session if resolution fails
my_session = afor.current_session()  # raises Exception is resolution fails
pub = node.create_publisher(String, "/chatter", 10)
```

Zenoh example:

```python
my_session = afor.auto_session()  # creates a session if resolution fails
my_session = afor.current_session()  # raises Exception is resolution fails
pub = session.declare_publisher("demo/chatter")
```

## Relation with `Scope`

Session and scope solve different problems:

- Session owns the transport runtime and is (potentially) synchronous
- Scope manage objects whose lifetimes are bound to a task, and cannot be garbage collected.
