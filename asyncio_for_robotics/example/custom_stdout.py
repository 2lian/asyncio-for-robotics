import asyncio
from contextlib import suppress
import subprocess
from typing import IO, Any, Awaitable, List

from asyncio_for_robotics.core.sub import BaseSub


def make_afor_stdout_monitor(process: subprocess.Popen[str]) -> BaseSub[str]:
    """Creates an afor subscriber that returns every line of stdout of the given process."""
    loop = asyncio.get_running_loop()
    assert process.stdout is not None
    stdout: IO[str] = process.stdout
    afor_sub: BaseSub[str] = BaseSub()

    def reader():
        line = stdout.readline().strip()
        healthy = True
        if line:
            healthy = afor_sub.input_data(line)
        proc_ended = process.poll() is not None
        if proc_ended or not healthy:
            loop.remove_reader(stdout.fileno())

    loop.add_reader(stdout.fileno(), reader)
    return afor_sub


async def main():
    proc = subprocess.Popen(
        ["ping", "localhost"],
        stdout=subprocess.PIPE,
        text=True,
    )
    stdout_sub = make_afor_stdout_monitor(proc)
    async for line in stdout_sub.listen_reliable():
        print(f"I heard: \n   {line}")


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
