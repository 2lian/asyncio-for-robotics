# Session Structure

This note explains the rationale behind the current session structure for ROS 2
and Zenoh.

This is internal design documentation. It is meant for advanced development,
not normal end-user onboarding.


## Purpose

Sessions solve a different problem than `afor.Scope`.

- a session owns the transport runtime
- a scope owns `afor` objects created inside that runtime

Examples:

- ROS session:
  - `rclpy` init / shutdown
  - node
  - executor
  - executor thread or asyncio spin task
- Zenoh session:
  - underlying `zenoh.Session`

Examples of things *not* owned by the session:

- ROS subscriptions
- ROS clients / servers
- Zenoh subscribers
- `Rate`
- transformed `BaseSub` objects

Those belong to `Scope`.


## Ownership Layers

Recommended ownership stack:

```python
with backend.auto_context(...):
    async with afor.Scope():
        ...
```

Why this order:

- leaving `Scope` destroys subscriptions, clients, servers, timers first
- leaving the session context then closes the underlying transport session

This avoids transport shutdown while child objects are still alive.


## Why lexical session context exists

Originally, sessions had to be:

- passed manually through constructors, or
- resolved from a global singleton

Passing manually is explicit, but repetitive.
Global fallback is convenient, but too implicit and easy to misuse.

The current design adds a middle layer:

- explicit `session=...`
- lexical session context
- backend fallback

This gives us:

- convenience without fallback-only behavior
- explicit ownership in code
- no hidden wrapper around the real session object


## Resolution order

`auto_session(...)` resolves in this order:

1. explicit `session`
2. current lexical session context
3. backend fallback, when supported

This order is intentional.

Explicit argument must always win.
Lexical context must beat fallback state.
Fallback creation exists only for compatibility and simple legacy code.


## Why sessions stay visible

The user must still be able to access the real session object.

For ROS, that means:

```python
with afor.auto_context() as session:
    with session.lock() as node:
        pub = node.create_publisher(...)
```

For Zenoh, that means:

```python
with afor.auto_context() as session:
    pub = session.declare_publisher(...)
```

We do not hide sessions behind a custom façade because that would reduce
clarity and remove transport-specific power from advanced users.


## Why session objects are passive about fallback policy

Session objects themselves do **not** mutate helper-layer fallback state when
they are closed.

This is deliberate.

Reasons:

- a session object should own its own transport resources, not resolution policy
- a session does not know whether it was used:
  - explicitly
  - lexically
  - as a fallback
- automatic fallback mutation is confusing once lexical contexts exist

This is also why `BaseSession.set_global_session()` is deprecated.

Fallback creation, when needed, is owned by the helper layer in
`ros2/session.py` and `zenoh/session.py`, not by session objects themselves.


## Why `set_auto_session()` was removed

`set_auto_session()` was effectively just global assignment with a friendlier
name.

That was acceptable before lexical contexts existed, but it became actively
confusing afterward:

- it looks scoped, but is global
- it interacts poorly with lexical session context
- it encourages policy mutation from arbitrary places

The current position is:

- use ROS `auto_context(...)` for normal ownership
- use Zenoh `auto_context(...)` for normal ownership
- keep fallback state managed by the helper layer


## Why `auto_context()` exists

For both ROS and Zenoh, `auto_context()` handles explicit sessions and
convenient session creation.

It exists because many users do not want to think about session construction at
all in the common case.

Examples:

```python
with afor.auto_context(node="my_node"):
    ...
```

```python
with afor.auto_context():
    ...
```

Nested contexts restore the outer lexical session when the inner context exits.
Zenoh may directly reuse an already-active session.


## Explicit-session ownership

ROS `auto_context(session)` binds an explicit session without closing it. This
supports:

- integrating with an externally owned transport runtime
- temporarily rebinding an already-managed session lexically

The caller remains responsible for closing an explicit session. The context
closes sessions that it creates itself.


## ROS-specific decisions

### ROS init ownership

ROS sessions own `rclpy.init()` / `rclpy.shutdown()` only when they started ROS
themselves.

This is tracked per session:

- if `rclpy` was already initialized, the session does not own shutdown
- if the session had to initialize ROS, it shuts ROS down when closed

This keeps ownership coherent while avoiding shutdown of ROS that belongs to
someone else.


### Default ROS session shape

The default ROS fallback is intentionally simple:

- `ThreadedSession`
- `SingleThreadedExecutor`

We do not keep configurable global default type / executor knobs anymore.

Reason:

- they add policy surface
- they are rarely needed
- when needed, the user can just instantiate the desired session explicitly


## Zenoh-specific decisions

Zenoh is simpler than ROS:

- no extra runtime init step
- no executor
- no node

So the Zenoh helper layer mostly manages:

- lexical binding
- optional close on exit
- global fallback creation from environment/default config


## Why session logic is split in ROS

ROS has more machinery than Zenoh, so the code is split into:

- `ros2/session_types.py`
  - `BaseSession`
  - `ThreadedSession`
  - `SynchronousSession`
- `ros2/session.py`
  - current lexical session
  - context managers
  - fallback resolution
  - `auto_session()`

This keeps the class implementations separate from resolution policy.


## Summary

The session structure is built around a few strong rules:

- session owns transport runtime
- scope owns `afor` objects
- explicit beats lexical
- lexical beats fallback
- session objects do not mutate fallback policy
- helper modules own fallback policy
- convenience APIs must remain thin

That is the current design center and should stay stable unless there is a
strong reason to change it.
