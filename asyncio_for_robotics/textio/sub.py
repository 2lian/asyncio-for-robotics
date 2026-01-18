import asyncio
from contextlib import suppress
import logging
import os
import subprocess
import sys
import warnings
from typing import IO, Callable, TypeVar, Union

from asyncio.constants import SENDFILE_FALLBACK_READBUFFER_SIZE

from asyncio_for_robotics.core.sub import BaseSub

_StrOrBytes = TypeVar("_StrOrBytes", str, bytes)
logger = logging.getLogger(__name__)


def default_preprocess(input: bytes) -> str:
    return input.decode().strip()


class Sub(BaseSub[_StrOrBytes]):
    def __init__(
        self,
        stream: IO,
        pre_process: Callable[[bytes], Union[_StrOrBytes, None]] = default_preprocess,
    ) -> None:
        """Subscriber streaming updates of a file.

        Notably useful to subscribe to text printed to stdout of a process.

        Args:
            stream: IO - Generic base class for TextIO and BinaryIO.
            pre_process: Function applied before any other processing. If
                returns None, line is skipped.
        """
        fd = os.dup(stream.fileno())
        stream = os.fdopen(fd, 'rb', closefd=True)
        self.stream: IO = stream
        self.pre_process: Callable[[bytes], Union[_StrOrBytes, None]] = pre_process
        super().__init__()
        if sys.platform.startswith("win"):
            warnings.warn("Windows requires changing the asyncio loop type")
        self.is_closed: bool = False
        self._close_event: asyncio.Event = asyncio.Event()
        self._read_task: asyncio.Task = asyncio.create_task(self._read_loop())

    async def _read_loop(self):
        reader = asyncio.StreamReader()
        protocol = asyncio.StreamReaderProtocol(reader)
        tra, _ = await self._event_loop.connect_read_pipe(lambda: protocol, self.stream)

        try:
            async for line in reader:
                self._io_update_cbk(line)
        except asyncio.CancelledError:
            pass
        finally:
            protocol.connection_lost(None)
            tra.close()

    @property
    def name(self) -> str:
        return f"sub io-{self.stream.name}"

    def _io_update_cbk(self, line_bytes: bytes):
        """Is called on updates to the IO file."""
        line = self.pre_process(line_bytes)
        healthy = True
        if line is not None:
            healthy = self.input_data(line)
        if not healthy:
            self.close()

    def close(self) -> asyncio.Task:
        """Closes the file reader (not the file)."""
        if self.is_closed:
            return self._read_task
        self._read_task.cancel()
        logger.debug(f"closing {self.name}")
        self.is_closed = True
        self._close_event.set()
        return self._read_task


def from_proc_stdout(
    process: subprocess.Popen,
    pre_process: Callable[[bytes], Union[_StrOrBytes, None]] = default_preprocess,
) -> Sub[_StrOrBytes]:
    """Creates a textio sub, relaying the lines printed in the process stdout.

    Automatically closes the sub when the process finishes

    Args:
        process: process on which to grab stdout
        pre_process: Function applied before any other processing. If
            returns None, line is skipped.

    Returns:
        textio sub of stdout.
    """
    if process.stdout is None:
        raise TypeError(
            "process.stdout is None. Please use `stdout=subprocess.PIPE` when calling Popen."
        )
    stdout: IO = process.stdout
    sub = Sub(stdout, pre_process)

    async def close_reader():
        # this blocks the loop destruction somehow
        # await asyncio.to_thread(process.wait)
        while process.poll() is None:
            await asyncio.sleep(1)
        logger.debug(f"{sub.name} closed because of process end.")
        sub.close()

    proc_wait_task = asyncio.create_task(close_reader())

    async def closed_so_stop_waiting():
        nonlocal proc_wait_task
        await sub._close_event.wait()
        logger.debug(f"{sub.name} closed cancelling process monitoring task")
        proc_wait_task.cancel()
        await proc_wait_task

    closed_wait_task = asyncio.create_task(closed_so_stop_waiting())
    return sub
