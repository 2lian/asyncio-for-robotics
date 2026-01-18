import asyncio
import logging
import subprocess
import sys
from typing import Any, AsyncGenerator, Callable, Generator

import pytest
import pytest_asyncio
from base_tests import (
    test_freshness,
    test_listen_one_by_one,
    test_listen_too_fast,
    test_reliable_extremely_fast,
    test_reliable_one_by_one,
    test_reliable_too_fast,
    test_wait_for_value,
    test_wait_new,
    test_wait_next,
)

import asyncio_for_robotics.textio as afor
from asyncio_for_robotics.core._logger import setup_logger

is_win = (
    sys.version_info[0] == 3
    and sys.version_info[1] >= 8
    and sys.platform.startswith("win")
)

# pytestmark = pytest.mark.skipif(is_win, reason="Requires a special EvenLoop")

setup_logger(debug_path="tests")
logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture
def session() -> Generator[subprocess.Popen[str], Any, Any]:
    logger.info("Starting process")
    proc = subprocess.Popen(
        "cmd.exe" if sys.platform.startswith("win") else "bash",
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    yield proc
    logger.info("Closing process")
    proc.terminate()
    proc.wait()


@pytest.fixture
def pub(session: subprocess.Popen[str]) -> Generator[Callable[[str], None], Any, Any]:
    def write_in_proc(input: str) -> None:
        assert session.stdin is not None
        res = session.stdin.write(f"""echo "{input}"\n""")
        session.stdin.flush()

    yield write_in_proc


@pytest.fixture
async def sub(session: subprocess.Popen[str]) -> AsyncGenerator[afor.Sub[str], Any]:
    assert session.stdout is not None
    s = afor.from_proc_stdout(session)
    yield s
    s.close()


# for f in [
#     test_freshness,
#     test_listen_one_by_one,
#     test_listen_too_fast,
#     test_reliable_extremely_fast,
#     test_reliable_one_by_one,
#     test_reliable_too_fast,
#     test_wait_for_value,
#     test_wait_new,
#     test_wait_next,
#         ]:
#     pytestmark(f)
