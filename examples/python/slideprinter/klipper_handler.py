"""
KlipperHandler
---------------

Purpose
  - WebSocket broadcaster for raw MCU bytes.
  - Optional PTY source: create a stubbed serial port (pty) for Klipper's
    [mcu] `serial:` setting (eg /tmp/pseudoserial) and forward bytes to WS.
  - Optional UDP injection: accept raw datagrams and forward to WS.
  - For first bring-up, we just log the raw bytes (no translation).

Usage (typical workflow)
  1) Start this handler to create the WS broadcaster.
     Options:
       - PTY source (standalone):
           python -m examples.python.slideprinter.klipper_handler --port 8770 \
                  --symlink /tmp/pseudoserial
       - UDP injection (pair with simulavr bridge):
           python -m examples.python.slideprinter.klipper_handler --port 8770 \
                  --udp-in-port 8771

     It will print the real PTY slave path and maintain a symlink at /tmp/pseudoserial.

  2) Start Klipper host so it opens /tmp/pseudoserial and streams MCU bytes.
     Two common options:

     A) Full service route (recommended for fidelity):
        - Ensure your Klipper host config uses:
            [mcu]
            serial: /tmp/pseudoserial
        - Start Klipper service (or run klippy.py normally) so it creates
          /tmp/printer for G-code input. Then feed G-code from OctoPrint or any
          client that speaks to /tmp/printer.

     B) Batch-ish route (experimental):
        - Try running klippy.py with an input gcode file while using the same
          config (so it attempts to open the serial). Depending on Klipper
          version, the batch mode may bypass serial, so prefer (A).

  3) Point the browser demo at ws://localhost:8770 (or wss if you terminate TLS)
     using the new "Klipper raw in" toggle in examples/js/slideprinter/index.html.
     You should see raw hex lines in the browser console if bytes flow.

Notes
  - This module does not parse or translate the MCU protocol. It just forwards
    raw bytes so we can confirm connectivity.
  - Later we can extend this to translate MCU traffic into the JSON commands
    the RemoteSpoolSystem expects.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import pty
import signal
import sys
from pathlib import Path
from typing import Optional, Set

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except Exception as e:  # pragma: no cover - optional dependency in some envs
    websockets = None
    WebSocketServerProtocol = object  # type: ignore


class PTYBridge:
    """Create a PTY pair, symlink slave to fixed path, and read master bytes."""

    def __init__(self, symlink_path: Path):
        self.symlink_path = symlink_path
        self.master_fd = None
        self.slave_path = None

    def open(self):
        master_fd, slave_fd = pty.openpty()
        self.master_fd = master_fd
        self.slave_path = Path(os.ttyname(slave_fd))

        # Refresh symlink
        try:
            if self.symlink_path.exists() or self.symlink_path.is_symlink():
                self.symlink_path.unlink()
        except FileNotFoundError:
            pass
        self.symlink_path.parent.mkdir(parents=True, exist_ok=True)
        os.symlink(self.slave_path, self.symlink_path)

        print(f"PTY created. Slave: {self.slave_path}")
        print(f"Symlinked to: {self.symlink_path}")

    def read_chunk(self, max_bytes: int = 4096) -> bytes:
        if self.master_fd is None:
            return b""
        try:
            return os.read(self.master_fd, max_bytes)
        except BlockingIOError:
            return b""


class WSRawBroadcaster:
    """Broadcast raw bytes to all connected websocket clients."""

    def __init__(self):
        self.clients: Set[WebSocketServerProtocol] = set()

    async def register(self, ws: WebSocketServerProtocol):
        self.clients.add(ws)
        try:
            await ws.wait_closed()
        finally:
            self.clients.discard(ws)

    async def broadcast(self, payload: bytes):
        if not self.clients:
            return
        coros = []
        for ws in list(self.clients):
            try:
                coros.append(ws.send(payload))
            except Exception:
                self.clients.discard(ws)
        if coros:
            await asyncio.gather(*coros, return_exceptions=True)


async def ws_handler(websocket: WebSocketServerProtocol, path: str, broadcaster: WSRawBroadcaster):
    # Register and also relay any binary messages from clients to all.
    broadcaster.clients.add(websocket)
    try:
        async for message in websocket:
            if isinstance(message, (bytes, bytearray)):
                await broadcaster.broadcast(message)
    finally:
        broadcaster.clients.discard(websocket)


async def pump_pty_to_ws(ptyb: PTYBridge, broadcaster: WSRawBroadcaster):
    loop = asyncio.get_event_loop()
    # Make master_fd non-blocking
    if ptyb.master_fd is not None:
        import fcntl
        import os as _os
        flags = fcntl.fcntl(ptyb.master_fd, fcntl.F_GETFL)
        fcntl.fcntl(ptyb.master_fd, fcntl.F_SETFL, flags | _os.O_NONBLOCK)

    while True:
        await asyncio.sleep(0.001)  # slight yield
        data = await loop.run_in_executor(None, ptyb.read_chunk)
        if data:
            # Log in hex lines for easier eyeballing
            hexstr = data.hex()
            print(f"MCU bytes: {hexstr[:128]}{'…' if len(hexstr) > 128 else ''}")
            await broadcaster.broadcast(data)


class UDPInjectProtocol(asyncio.DatagramProtocol):
    def __init__(self, broadcaster: WSRawBroadcaster):
        self.broadcaster = broadcaster
        self.loop = asyncio.get_event_loop()

    def datagram_received(self, data: bytes, addr):
        # Fire-and-forget broadcast
        self.loop.create_task(self.broadcaster.broadcast(data))


async def main_async(args):
    if websockets is None:
        print("The 'websockets' package is required. Please install it in your env.")
        sys.exit(2)
    broadcaster = WSRawBroadcaster()

    # Start WS server
    serve = websockets.serve(
        lambda ws, path: ws_handler(ws, path, broadcaster),
        host="localhost",
        port=args.port,
        max_size=None,
        max_queue=None,
    )

    # Optional PTY source
    ptyb: Optional[PTYBridge] = None
    pump_task = None
    if args.symlink is not None:
        ptyb = PTYBridge(args.symlink)
        ptyb.open()

    async with serve:
        if ptyb is not None:
            pump_task = asyncio.create_task(pump_pty_to_ws(ptyb, broadcaster))

        # Optional UDP injection
        transport = None
        if args.udp_in_port is not None:
            loop = asyncio.get_running_loop()
            transport, _ = await loop.create_datagram_endpoint(
                lambda: UDPInjectProtocol(broadcaster), local_addr=("127.0.0.1", args.udp_in_port)
            )
            print(f"Listening for UDP injections on 127.0.0.1:{args.udp_in_port}")

        # Keep running until interrupted
        stop = asyncio.Future()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                asyncio.get_running_loop().add_signal_handler(sig, stop.set_result, None)
            except NotImplementedError:
                pass
        await stop

        if pump_task:
            pump_task.cancel()
            try:
                await pump_task
            except asyncio.CancelledError:
                pass
        if transport:
            transport.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Klipper PTY and WebSocket bridge (raw MCU bytes)")
    parser.add_argument("--port", type=int, default=8770, help="WebSocket port for raw stream")
    parser.add_argument(
        "--symlink",
        type=Path,
        default=None,
        help="Create PTY and symlink here (eg /tmp/pseudoserial). Omit to disable PTY mode.",
    )
    parser.add_argument(
        "--udp-in-port",
        type=int,
        default=None,
        help="Listen for raw bytes via UDP on 127.0.0.1:PORT and broadcast to WS clients.",
    )
    args = parser.parse_args(argv)

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
