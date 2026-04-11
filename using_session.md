# Using Sessions

`afor` sessions own the transport runtime, you usually use only one per process. Session are synchronous thus do not require asyncio. Hence, sessions are more lenient than scopes, and a global session is created if you do not create one.

- ROS sessions are composed of:
  - `rclpy`
  - node
  - executor
- Zenoh sessions are exactly a `zenoh.Session`.

## Main idea

Use a session context outside your `afor.Scope`:

```python
with backend.session_context(...):
    async with afor.Scope():
        ...
```

This means:

- leaving `Scope` destroys subscriptions, clients, servers, timers
- leaving the session context closes the transport session itself

## Resolution order

1. explicit `session=...` on a class constructor
2. current `session_context` context
3. global fallback

So lexical session context is the normal non-global way to work. Global is usually set automatically if the user does not enter a `session_context`.

## ROS

Convenience session: when the normal default session behavior is enough. It handles `rclpy.init`, `rclpy.Executor`, `rclpy.Node` and their shutdown.

> [!NOTE]
> Calling `auto_context` inside an already existing afor session context does nothing.

```python
import asyncio_for_robotics.ros2 as afor


with afor.auto_context(node="my_node"):
    async with afor.Scope():
        sub = afor.Sub(String, "/chatter")
```

Explicit session: when you want to build the session yourself. It handles only the shutdown of the provided session if `close_on_exit=True`, else you are in charge.

```python
import asyncio_for_robotics.ros2 as afor
import rclpy

rclpy.init()
my_node = Node(name="my_node")
my_session = afor.ThreadedSession(node=my_node)

with afor.session_context(my_session, close_on_exit=False):
    async with afor.Scope():
        sub = afor.Sub(String, "/chatter")
        client = afor.Client(MySrv, "/compute")

my_session.close()
rclpy.shutdown()
```

## Zenoh (similar to ROS 2)

Convenience session:

```python
import asyncio_for_robotics.zenoh as afor


with afor.auto_context():
    async with afor.Scope():
        sub = afor.Sub("demo/**")
        ...
```

Explicit session:

```python
import zenoh
import asyncio_for_robotics.zenoh as afor

my_session = zenoh.open(zenoh.Config())

with afor.session_context(my_session, close_on_exit=False):
    async with afor.Scope():
        sub = afor.Sub("demo/**")
        ...

my_session.undeclare()
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

Zenoh example:

```python
my_session = afor.auto_session()  # creates a session if resolution fails
my_session = afor.current_session()  # raises Exception is resolution fails
pub = session.declare_publisher("demo/chatter")
```

## About globals

Global fallback exists for convenience, but it is no longer the recommended
usage.

Prefer:

- `with session_context(...)`
- `with auto_context(...)`

## Relation with `Scope`

Session and scope solve different problems:

- Session owns the transport runtime and is synchronous
- Scope owns `afor` asynchronous objects and tasks registered on -- possibly
  multiple -- different transports/sessions

That is why the recommended nesting is:

```python
with backend.auto_context():
    async with afor.Scope():
        ...
```

For scope-specific behavior, see [using_scope.md](./using_scope.md).
