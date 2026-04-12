import pytest
import sys

import asyncio_for_robotics
from asyncio_for_robotics.core._logger import setup_logger

setup_logger(debug_path=".")

pytest.register_assert_rewrite("tests.base_tests")

@pytest.fixture(autouse=True)
async def afor_scope():
    if sys.version_info >= (3, 11):
        async with asyncio_for_robotics.Scope():
            yield
    else:
        yield
