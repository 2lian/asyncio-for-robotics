import pytest

from asyncio_for_robotics.core.sub import ConverterSub

pytest.importorskip("zenoh")
import asyncio
import logging
from contextlib import suppress
from typing import Any, AsyncGenerator, Callable, Generator, Optional, Union

import zenoh
from base_tests import (
    test_freshness,
    test_listen_one_by_one,
    test_listen_too_fast,
    test_reliable_extremely_fast,
    test_reliable_one_by_one,
    test_reliable_too_fast,
    test_wait_cancellation,
    test_wait_for_value,
    test_wait_new,
    test_wait_next,
)

from asyncio_for_robotics.core import BaseSub
from asyncio_for_robotics.zenoh import (
    Sub,
    auto_session,
    set_auto_session,
    soft_timeout,
    soft_wait_for,
)

logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module", autouse=True)
def session() -> Generator[zenoh.Session, Any, Any]:
    set_auto_session(zenoh.open(zenoh.Config()))
    ses = auto_session()
    yield ses
    if not auto_session().is_closed():
        with suppress(zenoh.ZError):
            ses.close()


@pytest.fixture
def pub(session) -> Generator[Callable[[str], None], Any, Any]:
    pub_topic = "test/something"
    logger.debug("Creating PUB-%s", pub_topic)
    p: zenoh.Publisher = auto_session().declare_publisher(
        pub_topic, reliability=zenoh.Reliability.RELIABLE
    )

    def pub_func(input: str):
        p.put(input.encode())

    yield pub_func
    if not auto_session().is_closed():
        logger.debug("closing PUB-%s", pub_topic)
        p.undeclare()

@pytest.fixture
async def sub(session) -> AsyncGenerator[BaseSub[str], Any]:
    inner_sub = Sub("test/**")
    s: BaseSub[str] = ConverterSub(inner_sub, lambda sample: sample.payload.to_string())
    yield s
    s.close()
