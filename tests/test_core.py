import asyncio
import logging
from typing import Any, AsyncGenerator, Callable, Generator

import pytest
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

import asyncio_for_robotics.core as afor
from asyncio_for_robotics.core._logger import setup_logger

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture
async def sub() -> AsyncGenerator[afor.sub.BaseSub[str], Any]:
    s = afor.sub.BaseSub()
    yield s
    s.close()


@pytest.fixture
def pub(sub: afor.sub.BaseSub[str]) -> Generator[Callable[[str], None], Any, Any]:
    def write_in_proc(input: str) -> None:
        sub._input_data_asyncio(input)

    yield write_in_proc
