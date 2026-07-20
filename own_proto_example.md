# Implement a callback-based interface

`BaseSub.input_data()` is thread-safe. Use it from a callback exposed by any
event source (driver, middleware, or server) to create an `afor` data stream.
Callbacks already running in the asyncio thread can use
`BaseSub._input_data_guarded()`.

```python
afor_sub = afor.BaseSub()

def callback(data) -> None:
    afor_sub.input_data(data)

event_source.register_callback(callback)
```

## Native Python: UDP datagrams

Let's implement a simple UDP server that is natively available in python. Execution is simply:

1. `UDPServer` receives a datagram in its background thread.
2. `handle()` calls the thread-safe `input_data()` method.
3. `listen_reliable()` yields the bytes in the asyncio thread.

```python
import asyncio
import socketserver
from threading import Thread

import asyncio_for_robotics as afor


@afor.scoped
async def main() -> None:
    samples = afor.BaseSub[bytes]()

    class Handler(socketserver.BaseRequestHandler):
        def handle(self) -> None:
            data, _socket = self.request
            samples.input_data(data)

    server = socketserver.UDPServer(("127.0.0.1", 9999), Handler)
    Thread(target=server.serve_forever, daemon=True).start()
    try:
        async for sample in samples.listen_reliable():
            print(sample)
    finally:
        server.shutdown()
        server.server_close()


asyncio.run(main())
```

Run the example:

```bash
python3 -m asyncio_for_robotics.example.custom_udp
```
Send a sample from another terminal:

```bash
printf 'Hello World' | nc -u -w1 localhost 9999
```

## Cyclone DDS

Let's implement an interface to Cyclone DDS. Install it with
`pip install cyclonedds`, or use the repository's `dds` Pixi environment.

The code does the following:

1. `CycloneSub[MsgT]` creates a listener and `DataReader` for the supplied topic.
2. Cyclone DDS calls `_dds_callback()` from a DDS receive thread when data is available.
3. The thread-safe `input_data()` transfers the sample to the asyncio `afor` data stream.
4. Additionally here, the active `afor` scope calls `close()` on exit, cleaning the DDS subscriber.



```python
from dataclasses import dataclass
from typing import TypeVar

import asyncio_for_robotics as afor
from cyclonedds.core import Listener
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic


MsgT = TypeVar("MsgT")


@dataclass
class MyString(IdlStruct, typename="AforTutorial.MyString"):
    data: str


class CycloneSub(afor.BaseSub[MsgT]):
    def __init__(
        self,
        topic: Topic[MsgT],
    ) -> None:
        super().__init__()
        self._listener = Listener(on_data_available=self._dds_callback)
        self._reader = DataReader(topic.participant, topic, listener=self._listener)

    def _dds_callback(self, reader: DataReader[MsgT]) -> None:
        sample = reader.take_next()
        if sample is not None:
            self.input_data(sample)

    def close(self) -> None:
        try:
            if not self._closed.is_set():
                self._reader.set_listener(None)
        finally:
            super().close()
```

Create and consume it like any other `afor` subscriber:

```python
@afor.scoped
async def my_func():
    participant = DomainParticipant()
    topic = Topic(participant, "MyString", MyString)
    afor_sub = CycloneSub(topic)

    async for sample in afor_sub.listen_reliable():
        print(sample.data)
```

Run the complete example in two terminals:

```bash
# Terminal 1
pixi run -e dds python -m asyncio_for_robotics.example.custom_cyclonedds subscribe

# Terminal 2
pixi run -e dds python -m asyncio_for_robotics.example.custom_cyclonedds publish
```
