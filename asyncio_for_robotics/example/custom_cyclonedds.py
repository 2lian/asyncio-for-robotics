"""Adapt a Cyclone DDS DataReader listener to an afor subscriber.

Requires ``pip install cyclonedds``.

Run in two terminals:

    python3 -m asyncio_for_robotics.example.custom_cyclonedds subscribe
    python3 -m asyncio_for_robotics.example.custom_cyclonedds publish
"""

import argparse
import asyncio
from contextlib import suppress
from dataclasses import dataclass
from typing import TypeVar

import asyncio_for_robotics as afor
from cyclonedds.core import Listener
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic


MsgT = TypeVar("MsgT")


@dataclass
class MyString(IdlStruct, typename="AforTutorial.MyString"):
    data: str


class CycloneSub(afor.BaseSub[MsgT]):
    """Feed valid samples from a Cyclone DDS receive thread into asyncio."""

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


def make_topic() -> tuple[DomainParticipant, Topic[MyString]]:
    participant = DomainParticipant()
    return participant, Topic(participant, "MyString", MyString)


@afor.scoped
async def subscribe() -> None:
    participant, topic = make_topic()
    samples = CycloneSub(topic)

    async for sample in samples.listen_reliable():
        print(sample.data)


async def publish() -> None:
    participant, topic = make_topic()
    writer = DataWriter(participant, topic)
    sequence = 0
    await asyncio.sleep(0.3)

    while True:
        sample = MyString(data=f"Hello {sequence}")
        writer.write(sample)
        print(sample.data)
        sequence += 1
        await asyncio.sleep(1.0)


async def run(mode: str) -> None:
    if mode == "subscribe":
        await subscribe()
    else:
        await publish()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("subscribe", "publish"))
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    with suppress(KeyboardInterrupt):
        asyncio.run(run(args.mode))
