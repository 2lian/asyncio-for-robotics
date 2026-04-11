# Using Scope

> [!NOTE]
> Scopes are optional, but strongly recommended. Long-running asyncio tasks,
> subscriptions, and helper threads are not garbage collected by Python.
> Without an explicit lifetime owner, they can keep running longer than you
> intended.

`afor.Scope` is the normal lifetime owner for active `afor` objects.

Inside a scope block:

- new `afor` objects join the current scope automatically
- leaving the block closes them
- internal failures propagate out of the block
- `raise afor.ScopeBreak()` exits the current scope now
- `scope.cancel()` requests teardown of a scope without changing local control flow
- `scope.finished` reports how a scope ended once teardown is done

This keeps ownership visible without forcing the user to attach every object one by one.

Typical use:

```python
import asyncio_for_robotics as afor


async with afor.Scope():
    sub = afor.ros2.Sub(String, "/chatter")
    client = afor.ros2.Client(MySrv, "/compute")
    timer = afor.Rate(10)
```

All three objects belong to the same lexical lifetime. Leaving the block closes
them automatically and releases all of their ressources (ROS 2 subscriptions,
client, server; Zenoh sub ...).

## `@afor.scoped`

Use `@afor.scoped` when you want scope ownership without writing the
`async with` block yourself.

```python
import asyncio
import asyncio_for_robotics as afor


@afor.scoped
async def main():
    sub = afor.BaseSub[str]()
    sub.input_data("hello")
    print(await sub.wait_for_value())


asyncio.run(main())
```

If you need direct access to the scope object:

```python
import asyncio
import asyncio_for_robotics as afor


@afor.scoped(pass_scope=True)
async def main(scope: afor.Scope):
    sub = afor.BaseSub[str](scope=scope)
    sub.input_data("hello")
    print(await sub.wait_for_value())


asyncio.run(main())
```

This is often the cleanest way to give one whole async function a single,
explicit lifetime.

## Different from `asyncio.TaskGroup`

`afor.Scope` is lexical lifetime ownership, not a task nursery.

When execution reaches the end of the `async with afor.Scope():` block, the
scope exits normally and closes the objects it owns, even if those objects are
designed to run forever.

This is different from `TaskGroup`, where normal block exit means "wait for
child tasks to complete".

With `Scope`, the meaning is:

- while execution is inside the block, the owned objects are alive
- when execution leaves the block, the owned objects are closed

So this exits normally and closes `sub` at block end:

```python
async def main():
    async with afor.Scope():
        sub = afor.BaseSub[str]()
        sub.input_data("hello")
        value = await sub.wait_for_value()
        print(value)
```

You do not need `scope.cancel()` to leave a scope at the end of the block.

If you want a scope to stay alive until something else stops it, write that
explicitly:

```python
async def main():
    async with afor.Scope():
        sub = afor.BaseSub[str]()
        await asyncio.Future()
```

This is the important semantic point:

- end of block means "close what this block owns"
- it does not mean "wait forever for these objects to finish naturally"

`await asyncio.Future()` is a good explicit pattern when you intentionally want
the scope to remain open.

## Normal use

```python
import asyncio
import asyncio_for_robotics as afor


async def main():
    async with afor.Scope():
        sub = afor.BaseSub[str]()
        sub.input_data("hello")
        value = await sub.wait_for_value()
        print(value)


asyncio.run(main())
```

## Many objects in one scope

```python
import asyncio
import asyncio_for_robotics as afor


async def main():
    async with afor.Scope():
        subs = [afor.BaseSub[str]() for _ in range(3)]

        for idx, sub in enumerate(subs):
            sub.input_data(f"value-{idx}")

        values = [await sub.wait_for_value() for sub in subs]
        print(values)


asyncio.run(main())
```

## Deferred attach

Use this only when construction and ownership need to happen at different times.

```python
import asyncio
import asyncio_for_robotics as afor


async def main():
    sub = afor.BaseSub[str](scope=None)

    async with afor.Scope() as scope:
        sub.attach(scope)
        sub.input_data("late-binding")
        value = await sub.wait_for_value()
        print(value)


asyncio.run(main())
```

## Exiting the current scope

Use `raise afor.ScopeBreak()` when you want to leave the current scope now and continue after the block.

```python
import asyncio
import asyncio_for_robotics as afor


async def main():
    async with afor.Scope():
        sub = afor.BaseSub[str]()
        sub.input_data("hello")
        raise afor.ScopeBreak()

    print("continues here")


asyncio.run(main())
```

This is the closest thing to a `break` for a scope.

## Cancelling a scope

`scope.cancel()` requests early stop of the scope. This is useful when your subscriptions are long-lived and you want the scope to tear down instead of waiting for a natural end condition.

```python
import asyncio
import asyncio_for_robotics as afor


async def main():
    async with afor.Scope() as scope:
        sub = afor.BaseSub[str]()
        scope.cancel()
        await asyncio.sleep(0)
        print("cleanup was requested")

    await sub.lifetime
    print("scope stopped cleanly")


asyncio.run(main())
```

`scope.cancel()` does not change the local control flow of the current block.
It requests teardown of that scope, but it does not mean "leave this block right now".

If you want to stop the current scope and also leave the block immediately, use `raise afor.ScopeBreak()`:

```python
async def main():
    async with afor.Scope() as scope:
        sub = afor.BaseSub[str]()
        raise afor.ScopeBreak()
```

## Observing scope end

`scope.finished` is a future resolved when scope teardown is complete.
Use the future state directly:

- running: `not scope.finished.done()`
- completed normally: `scope.finished.done()` and `scope.finished.exception() is None`
- cancelled: `scope.finished.cancelled()`
- failed: `scope.finished.exception()` returns the propagated error

Example:

```python
import asyncio
import asyncio_for_robotics as afor


async def worker(scope: afor.Scope):
    await scope.finished
    print("completed")


async def main():
    scope = afor.Scope()
    watcher = asyncio.create_task(worker(scope))
    async with scope:
        sub = afor.BaseSub[str]()
        sub.input_data("hello")
        await sub.wait_for_value()
    await watcher


asyncio.run(main())
```

`scope.finished` is mainly for observers.
Awaiting it from inside the same still-running scope body is usually not useful unless some other task is responsible for ending that scope.

Calling `scope.finished.cancel()` while the scope is still running requests cancellation of that scope.

## Failure propagation

If a scoped subscriber fails internally, the scope fails too.
Because `Scope` uses `TaskGroup`, the propagated error may be a task-group exception group rather than the raw inner exception.

```python
import asyncio
import asyncio_for_robotics as afor


async def main():
    try:
        async with afor.Scope():
            sub = afor.BaseSub[str]()

            def boom(_: str) -> None:
                raise RuntimeError("boom")

            sub.asap_callback.append(boom)
            sub.input_data("hello")
            await asyncio.sleep(0)
    except Exception as exc:
        print(f"scope failed: {exc}")


asyncio.run(main())
```

## Notes

- `close()` is for one object.
- `raise afor.ScopeBreak()` exits the current scope now.
- `cancel()` requests teardown of a scope without acting like a local `break`.
- `finished` exposes normal `asyncio.Future` state for scope completion.
- `@afor.scoped` wraps an async function in a scope.
- `attach(scope)` is advanced API.
- `scope.task_group` exposes the real `TaskGroup`.
- `scope.exit_stack` exposes the real `AsyncExitStack`.
