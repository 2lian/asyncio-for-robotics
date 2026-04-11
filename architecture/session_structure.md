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
with backend.session_context(...):
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
- global fallback

This gives us:

- convenience without global-only behavior
- explicit ownership in code
- no hidden wrapper around the real session object


## Resolution order

`auto_session(...)` resolves in this order:

1. explicit `session`
2. current lexical session context
3. global singleton fallback

This order is intentional.

Explicit argument must always win.
Lexical context must beat global state.
Global fallback exists only for compatibility and simple legacy code.


## Why sessions stay visible

The user must still be able to access the real session object.

For ROS, that means:

```python
with afor.session_context(afor.ThreadedSession()) as session:
    with session.lock() as node:
        pub = node.create_publisher(...)
```

For Zenoh, that means:

```python
with afor.session_context(zenoh.open(...)) as session:
    pub = session.declare_publisher(...)
```

We do not hide sessions behind a custom façade because that would reduce
clarity and remove transport-specific power from advanced users.


## Why session objects are passive about globals

Session objects themselves do **not** mutate `GLOBAL_SESSION` when they are
closed.

This is deliberate.

Reasons:

- a session object should own its own transport resources, not module-global policy
- a session does not know whether it was used:
  - explicitly
  - lexically
  - as the global fallback
- automatic global mutation is confusing once lexical contexts exist

This is also why `BaseSession.set_global_session()` is deprecated.

Global fallback, when needed, is owned by the helper layer in
`ros2/session.py` and `zenoh/session.py`, not by the session objects
themselves.


## Why `set_auto_session()` was removed

`set_auto_session()` was effectively just global assignment with a friendlier
name.

That was acceptable before lexical contexts existed, but it became actively
confusing afterward:

- it looks scoped, but is global
- it interacts poorly with lexical session context
- it encourages policy mutation from arbitrary places

The current position is:

- use `session_context(...)` / `auto_context(...)` for normal ownership
- use `GLOBAL_SESSION` only as legacy fallback state managed by the helper layer


## Why `auto_context()` exists

`session_context(session)` is the explicit API.
`auto_context()` is the convenience API.

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

Important semantic rule:

- if a lexical session already exists, `auto_context()` reuses it and does not
  take ownership away from the outer context

This keeps nesting predictable.


## Why `close_on_exit` exists

`session_context(session, close_on_exit=True)` owns the session by default.

That is the normal case.

But sometimes the user provides a session they do **not** want closed by the
context. For example:

- integrating with an externally owned transport runtime
- temporarily rebinding an already-managed session lexically

That is why `close_on_exit` exists.

It is a small escape hatch, not the main path.


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

The default global fallback is intentionally simple:

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
  - global fallback
  - `auto_session()`

This keeps the class implementations separate from resolution policy.


## Summary

The session structure is built around a few strong rules:

- session owns transport runtime
- scope owns `afor` objects
- explicit beats lexical
- lexical beats global
- session objects do not mutate global fallback policy
- helper modules own fallback policy
- convenience APIs must remain thin

That is the current design center and should stay stable unless there is a
strong reason to change it.
