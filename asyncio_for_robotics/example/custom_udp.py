"""Adapt UDP datagrams to an afor subscriber."""

import asyncio
from contextlib import suppress
import socketserver
from threading import Thread

import asyncio_for_robotics as afor


@afor.scoped
async def main() -> None:
    sub = afor.BaseSub[bytes]()

    class Handler(socketserver.BaseRequestHandler):
        def handle(self) -> None:
            data, _socket = self.request
            sub.input_data(data)

    server = socketserver.UDPServer(("127.0.0.1", 9999), Handler)
    Thread(target=server.serve_forever, daemon=True).start()
    try:
        async for data in sub.listen_reliable():
            print(data)
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
