import asyncio
import logging
import subprocess
import sys
import threading
import time
from typing import IO, Callable, TypeVar, Union
import warnings

from asyncio_for_robotics.core.sub import BaseSub

_MsgType = TypeVar("_MsgType", str, bytes)
logger = logging.getLogger(__name__)


def default_preprocess(input: _MsgType) -> _MsgType:
    return input.strip()


class Sub(BaseSub[_MsgType]):
    def __init__(
        self,
        stream: IO[_MsgType],
        pre_process: Callable[[_MsgType], Union[_MsgType, None]] = default_preprocess,
    ) -> None:
        """Subscriber streaming updates of a file.

        Notably useful to subscribe to text printed to stdout of a process.

        Args:
            stream: IO - Generic base class for TextIO and BinaryIO.
            pre_process: Function applied before any other processing. If
                returns None, line is skipped.
        """
        self.stream = stream
        self.pre_process = pre_process
        super().__init__()
        if sys.platform.startswith("win"):
            warnings.warn("Windows requires changing the asyncio loop type")
        else:
            self._event_loop.add_reader(self.stream.fileno(), self._io_update_cbk)
        self.is_closed = False
        self._close_event = asyncio.Event()

    @property
    def name(self) -> str:
        return f"sub io-{self.stream.name}"

    def _win_io_update_cbk(self, line):
        healthy = True
        if line is not None:
            healthy = self.input_data(line)
        if not healthy:
            return
            # self._event_loop.call_soon_threadsafe(self.close)

    def _io_update_cbk(self):
        """Is called on updates to the IO file."""
        line = self.pre_process(self.stream.readline())
        healthy = True
        if line is not None:
            healthy = self.input_data(line)
        if not healthy:
            self.close()

    def close(self):
        """Closes the file reader (not the file)."""
        if self.is_closed:
            return
        logger.debug(f"closing {self.name}")
        self.is_closed = True
        self._close_event.set()
        self._event_loop.remove_reader(self.stream.fileno())

def from_proc_stdout(
    process: subprocess.Popen[_MsgType],
    pre_process: Callable[[_MsgType], Union[_MsgType, None]] = default_preprocess,
) -> Sub[_MsgType]:
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
    stdout: IO[_MsgType] = process.stdout
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
