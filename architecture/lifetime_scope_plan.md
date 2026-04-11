# Lifetime Scope Plan

## Goal

Make lifetime management:

- lean
- explicit
- understandable at a glance
- based on standard Python structured-concurrency tools
- consistent with the library philosophy: do not hide the real objects

The key point is that `Scope` should not reinvent task orchestration.
It should be a thin afor-specific wrapper around:

- `TaskGroup` for failure propagation and managed tasks
- `AsyncExitStack` for cleanup ordering

## Main Model

Inside:

```python
async with afor.Scope():
    ...
```

active afor objects join the current scope automatically unless another scope is passed explicitly.

This keeps the common path flat:

```python
async with afor.Scope():
    sub_a = afor.ros2.Sub(String, "/a")
    sub_b = afor.ros2.Sub(String, "/b")
    sub_c = afor.SomePollingSub(...)
```

The user sees one clear lifetime boundary and does not need to manually attach every object.

## Why `TaskGroup` and `AsyncExitStack`

### `TaskGroup`

We want real structured concurrency:

- child failure cancels sibling tasks
- failure propagates out of the scope
- advanced users can access the underlying task group directly

This should come from Python tooling, not a custom scheduler.

### `AsyncExitStack`

We want ordered teardown:

- scoped resources register `close()` callbacks
- those cleanup callbacks run on scope exit
- cleanup runs before the task group is fully exited

That ordering is important because many watcher tasks wait on resource lifetime.
The resources must close first, then the task group can finish cleanly.

## Python Version Strategy

This design fundamentally depends on `TaskGroup`.

Cleanest option:

- require Python 3.11+

If Python 3.10 support is retained:

- use a thin compatibility import around a `TaskGroup` backport
- do not reimplement `TaskGroup` semantics manually

The compatibility layer should stay tiny:

```python
try:
    from asyncio import TaskGroup
except ImportError:
    from taskgroup import TaskGroup
```

That is acceptable.
A custom home-grown task manager is not.

## Ownership Rules

### 1. `Scope` owns active objects

If an afor object needs cleanup, worker tasks, or transport teardown, it belongs to a scope.

### 2. Active objects auto-join the current scope

Use `contextvars` for lexical current-scope binding:

- inside a scope block: new active afor objects join that scope by default
- `scope=` can override the current scope explicitly
- outside a scope block: object creation should either fail clearly, or stay cold if the type supports that

No fallback global scope.

### 3. Object close does not close parent scope

```python
sub.close()
```

must only close `sub`.

### 4. Child failure fails the scope

If a scoped object hits an internal exception, the scope behaves like structured concurrency:

- the failure reaches the task group
- sibling managed tasks are cancelled by `TaskGroup`
- the exception propagates out of the scope
- in practice that may mean a task-group exception group, not the raw inner exception

### 5. Reaching block end closes resources

`Scope` is not a nursery.

Normal block exit means:

- execution is leaving the lifetime boundary
- owned resources are closed
- then the task group exits

This is different from plain `TaskGroup` usage, where normal block exit means “wait for children to complete naturally”.

For afor, the intended meaning is:

- while execution is inside the block, scoped resources are alive
- when execution leaves the block, scoped resources are closed

### 6. `scope.cancel()` is an early-stop request

`scope.cancel()` exists because many afor objects are intentionally long-lived.

It should:

- stop the scope early
- lead to normal scope teardown

Implementation detail:

- since `TaskGroup` has no direct “cancel group now” API, `Scope` may inject one internal task that raises a private exception
- `Scope` should suppress only that private internal exception on exit
- all real user exceptions must still propagate

`Scope.cancel()` should not add a second hidden control-flow system on top of `TaskGroup`.

### 7. `ScopeBreak` is the "exit here" strategy

Use:

```python
raise afor.ScopeBreak()
```

inside a scope body when you want to:

- stop the current scope now
- run normal scope teardown
- continue after the `async with` block

This gives the user a scope-local escape hatch without inventing hidden
control-flow semantics inside `scope.cancel()`.

### 8. `scope.finished` gives observability

Every active scope should expose a future that resolves when teardown is
complete.

That future should use standard `asyncio.Future` state directly:

- still running: `scope.finished.done() == False`
- completed normally: future resolved with `None`
- cancelled: `scope.finished.cancelled() == True`
- failed: `scope.finished.exception()` returns the propagated error

Additionally:

- `scope.finished.cancel()` while the scope is still active should request
  cancellation of that scope

### 9. `@afor.scoped` is the convenience entrypoint

This removes boilerplate for common async entrypoints:

```python
@afor.scoped
async def main():
    ...
```

Optionally:

```python
@afor.scoped(pass_scope=True)
async def main(scope: afor.Scope):
    ...
```

This is cleaner than inventing a hidden global scope.

## Naming

Names should stay short and sharp.

Recommended:

- per-object cleanup: `close()`
- current-scope immediate exit: `ScopeBreak`
- scope-wide early stop: `cancel()`
- scope observability: `finished`
- function convenience: `scoped`
- advanced deferred ownership: `attach(scope)`

Reason:

- `close()` implies local resource teardown
- `ScopeBreak` is explicit control flow
- `cancel()` clearly signals control-flow interruption
- `finished` is explicit observability
- `scoped` is explicit convenience
- `attach(scope)` is readable and explicit for advanced cases

## Public Surface

### Common path

```python
async with afor.Scope():
    sub = afor.ros2.Sub(String, "/topic")
```

### Explicit scope selection

```python
async with afor.Scope() as scope:
    sub = afor.ros2.Sub(String, "/topic", scope=scope)
```

### Exit current scope now

```python
async with afor.Scope():
    ...
    raise afor.ScopeBreak()
# continues here
```

### Observe scope completion

```python
scope = afor.Scope()

async with scope:
    ...

await scope.finished
```

### Convenience decorator

```python
@afor.scoped
async def main():
    ...
```

### Deferred attach for advanced cases

```python
sub = SomeSpecialSub(scope=None)

async with afor.Scope() as scope:
    sub.attach(scope)
```

This is important when construction and ownership happen at different times.

Most users should not need it.

### Advanced direct task-group access

```python
async with afor.Scope() as scope:
    scope.task_group.create_task(do_work())
```

This should remain available, because advanced users may need direct structured-concurrency access.

## Lean Implementation Sketch

```python
import asyncio
import contextvars
from contextlib import AsyncExitStack

try:
    from asyncio import TaskGroup
except ImportError:
    from taskgroup import TaskGroup


_current_scope = contextvars.ContextVar("afor_current_scope", default=None)


class _ScopeCancelled(Exception):
    pass


class Scope:
    def __init__(self):
        self.exit_stack = None
        self.task_group = None
        self._finished = None
        self._updating_finished = False
        self._token = None

    async def __aenter__(self):
        self._finished = asyncio.get_running_loop().create_future()
        self._finished.add_done_callback(self._finished_done_callback)
        self.exit_stack = AsyncExitStack()
        await self.exit_stack.__aenter__()
        self.task_group = await self.exit_stack.enter_async_context(TaskGroup())
        self._token = _current_scope.set(self)
        return self

    async def __aexit__(self, exc_type, exc, tb):
        _current_scope.reset(self._token)
        try:
            suppressed = await self.exit_stack.__aexit__(exc_type, exc, tb)
            self._finished.set_result(None)
            return suppressed or exc_type is ScopeBreak
        except BaseException as err:
            if exc_type is ScopeBreak:
                filtered_break = self._strip_scope_break(err)
                if filtered_break is None:
                    self._finished.cancel()
                    return True
                err = filtered_break
            filtered = self._strip_scope_cancel(err)
            if filtered is None:
                self._finished.cancel()
                return exc_type in (None, asyncio.CancelledError)
            self._finished.set_exception(filtered)
            raise filtered

    def cancel(self):
        self.task_group.create_task(self._cancel_scope(), name="afor_scope_cancel")

    async def _cancel_scope(self):
        raise _ScopeCancelled()

    @classmethod
    def _strip_scope_cancel(cls, err):
        if isinstance(err, (ScopeBreak, _ScopeCancelled)):
            return None
        ...

    @staticmethod
    def current():
        scope = _current_scope.get()
        if scope is None:
            raise RuntimeError("No active afor.Scope")
        return scope


def scoped(func=None, *, pass_scope=False):
    ...
```

The important thing is not the exact helper names.
The important thing is that the implementation stays a thin layer over standard primitives.

## BaseSub Implication

`BaseSub` itself does not need an internal worker task.
Its real background behavior is cross-thread input through `call_soon_threadsafe`.

What `Scope` must give `BaseSub` is:

- cleanup registration: `scope.exit_stack.callback(self.close)`
- one watcher task: `scope.task_group.create_task(self._watch_lifetime())`

That watcher task is how exceptions raised from cross-thread callback scheduling become structured failures instead of loose event-loop callback errors.

## Recommendation

Implement and document this model:

- `Scope` is built directly on `TaskGroup` and `AsyncExitStack`
- current scope is lexical via `contextvars`
- active objects auto-join the current scope
- `ScopeBreak` exits the current scope now
- `scope.finished` is a normal Future for scope completion state
- `@afor.scoped` wraps async entrypoints in a scope
- `scope=` remains available
- `attach(scope)` exists for advanced deferred ownership
- `scope.task_group` exposes the real task group for advanced users
- `scope.exit_stack` exposes the real exit stack for advanced users
- `close()` is per-object
- `cancel()` is per-scope

This gives:

- flat syntax
- strong ownership
- real structured concurrency
- minimal custom machinery
- behavior users can understand by reading the standard Python docs plus a small afor layer

