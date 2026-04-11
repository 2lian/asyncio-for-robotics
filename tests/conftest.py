import pytest

import asyncio_for_robotics
from asyncio_for_robotics.core._logger import setup_logger

setup_logger(debug_path=".")


@pytest.fixture(autouse=True)
async def afor_scope():
    async with asyncio_for_robotics.Scope():
        yield
