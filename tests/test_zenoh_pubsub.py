import pytest

import asyncio_for_robotics
from asyncio_for_robotics.core.sub import ConverterSub

pytest.importorskip("zenoh")
import asyncio
import logging
from contextlib import suppress
from typing import Any, AsyncGenerator, Callable, Generator, Optional, Union

import zenoh

from asyncio_for_robotics.core import BaseSub
from asyncio_for_robotics.zenoh import (
    Sub,
    auto_context,
    auto_session,
    current_session,
    session_context,
    soft_timeout,
    soft_wait_for,
)

from .base_tests import (
    test_freshness,
    test_listen_one_by_one,
    test_listen_too_fast,
    test_loop_cancellation,
    test_reliable_extremely_fast,
    test_reliable_one_by_one,
    test_reliable_too_fast,
    test_wait_cancellation,
    test_wait_for_value,
    test_wait_new,
    test_wait_next,
)

logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module", autouse=True)
def session() -> Generator[zenoh.Session, Any, Any]:
    session = zenoh.open(zenoh.Config())
    try:
        with auto_context(session) as active_session:
            yield active_session
    finally:
        session.close()


def test_auto_context_binds_explicit_session_without_closing() -> None:
    outer_session = current_session()
    explicit_session = zenoh.open(zenoh.Config())
    try:
        with auto_context(explicit_session) as active_session:
            assert active_session is explicit_session
            assert current_session() is explicit_session
            assert auto_session() is explicit_session

        assert current_session() is outer_session
        assert not explicit_session.is_closed()
    finally:
        explicit_session.close()


def test_auto_context_closes_created_session() -> None:
    outer_session = current_session()
    with auto_context() as owned_session:
        assert current_session() is owned_session

    assert owned_session.is_closed()
    assert current_session() is outer_session


def test_session_context_compatibility() -> None:
    outer_session = current_session()
    explicit_session = zenoh.open(zenoh.Config())
    try:
        with pytest.warns(DeprecationWarning, match="session_context.*deprecated"):
            with session_context(
                explicit_session,
                close_on_exit=False,
            ) as active_session:
                assert active_session is explicit_session
                assert current_session() is explicit_session

        assert current_session() is outer_session
        assert not explicit_session.is_closed()
    finally:
        explicit_session.close()


@pytest.fixture
def pub(session) -> Generator[Callable[[str], None], Any, Any]:
    pub_topic = "test/something"
    logger.debug("Creating PUB-%s", pub_topic)
    p: zenoh.Publisher = session.declare_publisher(
        pub_topic, reliability=zenoh.Reliability.RELIABLE
    )

    def pub_func(input: str):
        p.put(input.encode())

    yield pub_func
    p.undeclare()


@pytest.fixture
async def sub(session) -> AsyncGenerator[BaseSub[str], Any]:
    inner_sub = Sub("test/**")
    s: BaseSub[str] = ConverterSub(inner_sub, lambda sample: sample.payload.to_string())
    yield s
