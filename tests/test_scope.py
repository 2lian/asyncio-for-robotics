import asyncio

import pytest

import asyncio_for_robotics as afor


def _flatten_exceptions(exc: BaseException) -> list[BaseException]:
    if hasattr(exc, "exceptions"):
        out: list[BaseException] = []
        for inner in exc.exceptions:
            out.extend(_flatten_exceptions(inner))
        return out
    return [exc]


async def test_scope_auto_attaches_sub():
    async with afor.Scope() as scope:
        sub = afor.BaseSub[str]()
        assert sub._scope is scope


async def test_detached_sub_can_attach_later():
    sub = afor.BaseSub[str](scope=None)

    async with afor.Scope() as scope:
        sub.attach(scope)
        assert sub.input_data("hello")
        assert await sub.wait_for_value() == "hello"


async def test_scope_cancel_closes_sub():
    async with afor.Scope() as scope:
        sub = afor.BaseSub[str]()
        scope.cancel()
        await asyncio.sleep(0)

    assert sub._closed.is_set()
    assert await sub.lifetime is True


async def test_scope_break_exits_current_scope_and_continues():
    reached_after_scope = False
    scope = afor.Scope()

    async with scope:
        sub = afor.BaseSub[str]()
        sub.input_data("hello")
        raise afor.ScopeBreak()

    reached_after_scope = True
    assert reached_after_scope is True
    assert sub._closed.is_set()
    assert await sub.lifetime is True
    assert scope.finished.done()
    assert scope.finished.cancelled()


async def test_scope_propagates_input_callback_failure():
    def boom(_: str) -> None:
        raise RuntimeError("boom in callback")

    with pytest.raises(BaseException) as exc_info:
        async with afor.Scope():
            sub = afor.BaseSub[str]()
            sub.asap_callback.append(boom)
            assert sub.input_data("hello")
            await asyncio.sleep(0)

    errors = _flatten_exceptions(exc_info.value)
    assert any(
        isinstance(exc, RuntimeError) and str(exc) == "boom in callback"
        for exc in errors
    )


async def test_scope_propagates_call_soon_threadsafe_failure(monkeypatch):
    with pytest.raises(BaseException) as exc_info:
        async with afor.Scope():
            sub = afor.BaseSub[str]()

            def broken_call_soon_threadsafe(*args, **kwargs):
                raise RuntimeError("call_soon_threadsafe failed")

            monkeypatch.setattr(
                sub._event_loop,
                "call_soon_threadsafe",
                broken_call_soon_threadsafe,
            )
            assert sub.input_data("hello") is False
            await asyncio.sleep(0)

    errors = _flatten_exceptions(exc_info.value)
    assert any(
        isinstance(exc, RuntimeError) and str(exc) == "call_soon_threadsafe failed"
        for exc in errors
    )


async def test_scope_finished_completed():
    scope = afor.Scope()

    async with scope:
        sub = afor.BaseSub[str]()
        sub.input_data("hello")
        assert await sub.wait_for_value() == "hello"

    assert await scope.finished is None
    assert scope.finished.done()
    assert not scope.finished.cancelled()
    assert scope.finished.exception() is None


async def test_scope_finished_failed():
    scope = afor.Scope()

    with pytest.raises(BaseException):
        async with scope:
            sub = afor.BaseSub[str]()

            def boom(_: str) -> None:
                raise RuntimeError("boom in callback")

            sub.asap_callback.append(boom)
            sub.input_data("hello")
            await asyncio.sleep(0)

    assert scope.finished.done()
    assert not scope.finished.cancelled()
    error = scope.finished.exception()
    assert error is not None
    assert any(
        isinstance(exc, RuntimeError) and str(exc) == "boom in callback"
        for exc in _flatten_exceptions(error)
    )


async def test_scope_finished_cancel_requests_scope_cancel():
    scope = afor.Scope()

    async def cancel_later():
        await asyncio.sleep(0)
        scope.finished.cancel()

    async with scope:
        asyncio.create_task(cancel_later())
        await asyncio.sleep(0.01)

    assert scope.finished.cancelled()


async def test_scoped_decorator_auto_scope():
    @afor.scoped
    async def wrapped():
        sub = afor.BaseSub[str]()
        sub.input_data("hello")
        return await sub.wait_for_value()

    assert await wrapped() == "hello"


async def test_scoped_decorator_pass_scope():
    @afor.scoped(pass_scope=True)
    async def wrapped(scope: afor.Scope):
        sub = afor.BaseSub[str](scope=scope)
        sub.input_data("hello")
        value = await sub.wait_for_value()
        return value, scope

    value, scope = await wrapped()
    assert value == "hello"
    assert await scope.finished is None
