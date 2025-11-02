# Implement your own protocol

If you have a stream of data, you can use it through `afor`. This quick tutorial will implement a `afor` subscriber getting data from `stdout` of a process. So every line the process is prints to the terminal will be available as a string.

## Code

The following runs `ping localhost` and gets the process `stdout` to a `BaseSub` of asyncio_for_robotics.

The final code is available in the examples: [asyncio_for_robotics.example.custom_stdout](./asyncio_for_robotics/example/custom_stdout.py). You can run it using `python3 -m asyncio_for_robotics.example.custom_stdout`.

```python
import asyncio
from contextlib import suppress
import subprocess
from typing import IO

from asyncio_for_robotics.core.sub import BaseSub

def make_afor_stdout_monitor(process: subprocess.Popen[str]) -> BaseSub[str]:
    """Creates an afor subscriber that returns every line of stdout of the given process."""
    loop = asyncio.get_running_loop()
    assert process.stdout is not None
    stdout: IO[str] = process.stdout
    afor_sub: BaseSub[str] = BaseSub()

    def reader():
        line = stdout.readline()
        healthy = True
        if line:
            healthy = afor_sub.input_data(line)
        proc_ended = process.poll() is not None
        if proc_ended or not healthy:  # process ended
            loop.remove_reader(stdout.fileno())

    loop.add_reader(stdout.fileno(), reader)
    return afor_sub

async def main():
    proc = subprocess.Popen(
        ["ping", "google.com"],
        stdout=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    stdout_sub = make_afor_stdout_monitor(proc)
    async for line in stdout_sub.listen_reliable():
        print(f"I heard: \n   {line}")


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
```

## Analyse the code

`main` is very straight forward. The process is started using python's `Popen` with it's `stdout` captured and buffered. `make_afor_stdout_monitor` creates a subscriber of it then every line is printed.

```python
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
```

`make_afor_stdout_monitor` is where everything happens. First a few objects are initialized.

```python
def make_afor_stdout_monitor(process: subprocess.Popen[str]) -> BaseSub[str]:
    """Creates an afor subscriber that returns every line of stdout of the given process."""
    loop = asyncio.get_running_loop()
    assert process.stdout is not None
    stdout: IO[str] = process.stdout
    afor_sub: BaseSub[str] = BaseSub()
```

We define a callback function. In this function, `line = stdout.readline()` reads the next line of `stdout`, then `afor_sub.input_data(line)` inputs it into our subscriber. That's it, our job here is done. We just do a little cleanup to stop the reader when the process is closed.

```python
    def reader():
        line = stdout.readline()
        healthy = True
        if line:
            healthy = afor_sub.input_data(line)
        proc_ended = process.poll() is not None
        if proc_ended or not healthy:
            loop.remove_reader(stdout.fileno())
```

The following method from asyncio will call our `reader` callback every time something happens onto `stdout`.

```python
    loop.add_reader(stdout.fileno(), reader)
```

## Important Note: Be careful to not miss data!

In this example `.readline()` reads the next line on each call, and `read()` is called on every line change. So we cannot miss a line.
