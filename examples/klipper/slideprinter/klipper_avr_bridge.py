#!/usr/bin/env python3
"""
Klipper AVR Bridge (SimulAVR + WebSocket)
-----------------------------------------

Purpose
  - Emulate an AVR MCU using pysimulavr, exposing a PTY that Klipper can open.
  - Forward ALL serial-link bytes (both directions) to connected WebSocket
    clients as raw binary frames (no parsing).

Rationale
  Merges the prior two-step chain (simulavr -> UDP -> WS) into one script.

Usage
  PYTHONPATH=/path/to/simulavr/build/pysimulavr \
    /path/to/hp-sim5/examples/klipper/slideprinter/klipper_avr_bridge.py \
    /path/to/klipper.elf

  Optional flags:
    --machine atmega644     AVR device (default: atmega644)
    --speed 16000000        CPU clock in Hz (default: 16000000)
    --rate 0.0              Real-time pacing (0 disables)
    --baud 250000           UART baud rate (default: 250000)
    --port /tmp/pseudoserial  PTY symlink Klipper opens (default as shown)
    --trace <signals>       Enable VCD tracing (use '?' to list)
    --tracefile file.vcd    VCD output path
    --ws-host localhost     WebSocket bind host (default: localhost)
    --ws-port 8770          WebSocket port (default: 8770)

Clients connect to: ws://<ws-host>:<ws-port>
"""

from __future__ import annotations

import argparse
import asyncio
import errno
import fcntl
import os
import pty
import signal
import sys
import termios
import threading
import time
from pathlib import Path

try:
    import pysimulavr
except Exception as e:
    print("pysimulavr is required. Ensure PYTHONPATH points to pysimulavr.")
    raise

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except Exception as e:
    websockets = None
    WebSocketServerProtocol = object  # type: ignore


SERIALBITS = 10  # 8N1
SIMULAVR_FREQ = 10 ** 9


class WSRawBroadcaster:
    def __init__(self):
        self.clients: set[WebSocketServerProtocol] = set()

    async def register(self, ws: WebSocketServerProtocol):
        self.clients.add(ws)
        try:
            await ws.wait_closed()
        finally:
            self.clients.discard(ws)

    async def broadcast(self, payload: bytes):
        if not self.clients:
            return
        senders = []
        for ws in list(self.clients):
            try:
                senders.append(ws.send(payload))
            except Exception:
                self.clients.discard(ws)
        if senders:
            await asyncio.gather(*senders, return_exceptions=True)


async def ws_handler(ws: WebSocketServerProtocol, _path: str, broadcaster: WSRawBroadcaster):
    # Accept connections; if clients send binary/text, ignore for now.
    broadcaster.clients.add(ws)
    try:
        async for _ in ws:
            pass
    finally:
        broadcaster.clients.discard(ws)


class Tracing:
    def __init__(self, filename, signals):
        self.filename = filename
        self.signals = signals
        if not signals:
            self.dman = None
            return
        self.dman = pysimulavr.DumpManager.Instance()
        self.dman.SetSingleDeviceApp()

    def show_help(self):
        ostr = pysimulavr.ostringstream()
        self.dman.save(ostr)
        sys.stdout.write(ostr.str())
        sys.exit(1)

    def load_options(self):
        if self.dman is None:
            return
        if self.signals.strip() == '?':
            self.show_help()
        sigs = "\n".join(["+ " + s for s in self.signals.split(',')])
        self.dman.addDumpVCD(self.filename, sigs, "ns", False, False)

    def start(self):
        if self.dman is not None:
            self.dman.start()

    def finish(self):
        if self.dman is not None:
            self.dman.stopApplication()


class Pacing(pysimulavr.PySimulationMember):
    def __init__(self, rate):
        pysimulavr.PySimulationMember.__init__(self)
        self.sc = pysimulavr.SystemClock.Instance()
        self.pacing_rate = 1.0 / (rate * SIMULAVR_FREQ)
        self.next_check_clock = 0
        self.rel_time = time.time()
        self.best_offset = 0.0
        self.delay = SIMULAVR_FREQ // 10000
        self.sc.Add(self)

    def DoStep(self, trueHwStep):
        curtime = time.time()
        clock = self.sc.GetCurrentTime()
        offset = clock * self.pacing_rate - (curtime - self.rel_time)
        self.best_offset = max(self.best_offset, offset)
        if offset > 0.000050:
            time.sleep(offset - 0.000040)
        if clock >= self.next_check_clock:
            self.rel_time -= min(self.best_offset, 0.0)
            self.next_check_clock = clock + self.delay * 500
            self.best_offset = -999999999.0
        return self.delay


class SerialRxPin(pysimulavr.PySimulationMember, pysimulavr.Pin):
    """AVR TXD -> Terminal.write(data)

    Captures bits from the device TX pin and emits whole bytes to the
    terminal, which writes them to the PTY and mirrors to WS.
    """

    def __init__(self, baud, terminal):
        pysimulavr.Pin.__init__(self)
        pysimulavr.PySimulationMember.__init__(self)
        self.terminal = terminal
        self.sc = pysimulavr.SystemClock.Instance()
        self.delay = SIMULAVR_FREQ // baud
        self.current = 0
        self.pos = -1

    def SetInState(self, pin):
        pysimulavr.Pin.SetInState(self, pin)
        self.state = pin.outState
        if self.pos < 0 and pin.outState == pin.LOW:
            self.pos = 0
            self.sc.Add(self)

    def DoStep(self, trueHwStep):
        ishigh = self.state == self.HIGH
        self.current |= ishigh << self.pos
        self.pos += 1
        if self.pos == 1:
            return int(self.delay * 1.5)
        if self.pos >= SERIALBITS:
            data = bytearray([(self.current >> 1) & 0xFF])
            self.terminal.write(data)
            self.pos = -1
            self.current = 0
            return -1
        return self.delay


class SerialTxPin(pysimulavr.PySimulationMember, pysimulavr.Pin):
    """Terminal.read() -> AVR RXD

    Fetches bytes from the terminal (PTY master) and clocks them out on the
    device RX pin.
    """

    def __init__(self, baud, terminal):
        pysimulavr.Pin.__init__(self)
        pysimulavr.PySimulationMember.__init__(self)
        self.terminal = terminal
        self.SetPin('H')
        self.sc = pysimulavr.SystemClock.Instance()
        self.delay = SIMULAVR_FREQ // baud
        self.current = 0
        self.pos = 0
        self.queue = bytearray()
        self.sc.Add(self)

    def DoStep(self, trueHwStep):
        if not self.pos:
            if not self.queue:
                data = self.terminal.read()
                if not data:
                    return self.delay * 100
                self.queue.extend(data)
            self.current = (self.queue.pop(0) << 1) | 0x200
        newstate = 'L'
        if self.current & (1 << self.pos):
            newstate = 'H'
        self.SetPin(newstate)
        self.pos += 1
        if self.pos >= SERIALBITS:
            self.pos = 0
        return self.delay


class TerminalIO:
    """PTY-backed terminal with WS mirroring.

    - write(data): AVR->Host direction (device TX). Writes to PTY master and
      schedules WS broadcast of the same bytes.
    - read(): Host->AVR direction (host TX). Reads from PTY master and
      schedules WS broadcast of the same bytes.
    """

    def __init__(self, loop: asyncio.AbstractEventLoop, ws_queue: asyncio.Queue[bytes]):
        self.fd = -1
        self.loop = loop
        self.ws_queue = ws_queue

    def run(self, fd: int):
        self.fd = fd

    def _mirror_ws(self, data: bytes):
        if not data:
            return
        # Thread-safe queue put into asyncio loop
        self.loop.call_soon_threadsafe(self.ws_queue.put_nowait, data)

    def write(self, data: bytes):
        # Bytes from AVR TX go to PTY and WS
        if self.fd >= 0 and data:
            os.write(self.fd, data)
            self._mirror_ws(data)

    def read(self) -> bytes:
        try:
            data = os.read(self.fd, 64)
            if data:
                self._mirror_ws(data)
            return data
        except os.error as e:
            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                pysimulavr.SystemClock.Instance().stop()
        return b""


def create_pty(ptyname: str) -> int:
    mfd, sfd = pty.openpty()
    try:
        os.unlink(ptyname)
    except os.error:
        pass
    os.symlink(os.ttyname(sfd), ptyname)
    # Non-blocking
    fcntl.fcntl(mfd, fcntl.F_SETFL, fcntl.fcntl(mfd, fcntl.F_GETFL) | os.O_NONBLOCK)
    # Raw mode
    tcattr = termios.tcgetattr(mfd)
    tcattr[0] &= ~(
        termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
        termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
    tcattr[1] &= ~termios.OPOST
    tcattr[3] &= ~(
        termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG |
        termios.IEXTEN)
    tcattr[2] &= ~(termios.CSIZE | termios.PARENB)
    tcattr[2] |= termios.CS8
    tcattr[6][termios.VMIN] = 0
    tcattr[6][termios.VTIME] = 0
    termios.tcsetattr(mfd, termios.TCSAFLUSH, tcattr)
    return mfd


def run_simulavr(options, elffile: str, loop: asyncio.AbstractEventLoop, ws_queue: asyncio.Queue[bytes]):
    # launch simulator
    sc = pysimulavr.SystemClock.Instance()
    trace = Tracing(options.tracefile, options.trace)
    dev = pysimulavr.AvrFactory.instance().makeDevice(options.machine)
    dev.Load(elffile)
    dev.SetClockFreq(SIMULAVR_FREQ // options.speed)
    sc.Add(dev)
    pysimulavr.cvar.sysConHandler.SetUseExit(False)
    trace.load_options()

    if options.pacing_rate:
        _ = Pacing(options.pacing_rate)

    io = TerminalIO(loop, ws_queue)

    # RX from device (TXD) on D1
    rxpin = SerialRxPin(options.baud, io)
    net = pysimulavr.Net()
    net.Add(rxpin)
    net.Add(dev.GetPin("D1"))

    # TX to device (RXD) on D0
    txpin = SerialTxPin(options.baud, io)
    net2 = pysimulavr.Net()
    net2.Add(dev.GetPin("D0"))
    net2.Add(txpin)

    # Info
    msg = f"Starting AVR simulation: machine={options.machine} speed={options.speed}\n"
    msg += f"Serial: port={options.port} baud={options.baud}\n"
    if options.trace:
        msg += f"Trace file: {options.tracefile}\n"
    sys.stdout.write(msg)
    sys.stdout.flush()

    # Create terminal device
    fd = create_pty(options.port)

    # Run until stopped
    try:
        io.run(fd)
        trace.start()
        sc.RunTimeRange(0x7fff0000ffff0000)
        trace.finish()
    finally:
        try:
            os.unlink(options.port)
        except Exception:
            pass


async def main_async(argv=None):
    if websockets is None:
        print("The 'websockets' package is required. Please install it.")
        sys.exit(2)

    parser = argparse.ArgumentParser(description="SimulAVR PTY + WebSocket raw-bytes bridge")
    parser.add_argument("elffile", type=str, help="Path to AVR ELF (e.g., klipper.elf)")
    parser.add_argument("-m", "--machine", default="atmega644", help="AVR device type")
    parser.add_argument("-s", "--speed", type=int, default=16000000, help="CPU speed (Hz)")
    parser.add_argument("-r", "--rate", dest="pacing_rate", type=float, default=0.0, help="Real-time pacing rate")
    parser.add_argument("-b", "--baud", type=int, default=250000, help="UART baud rate")
    parser.add_argument("-t", "--trace", default=None, help="Signals to trace (? for list)")
    parser.add_argument("-p", "--port", default="/tmp/pseudoserial", help="PTY symlink path for Klipper")
    deffile = Path(sys.argv[0]).with_suffix(".vcd").name
    parser.add_argument("-f", "--tracefile", default=deffile, help="Trace VCD filename")
    parser.add_argument("--ws-host", default="localhost", help="WebSocket bind host")
    parser.add_argument("--ws-port", type=int, default=8770, help="WebSocket port")

    args = parser.parse_args(argv)

    broadcaster = WSRawBroadcaster()

    # WS server
    ws_server = websockets.serve(
        lambda ws, path: ws_handler(ws, path, broadcaster),
        host=args.ws_host,
        port=args.ws_port,
        max_size=None,
        max_queue=None,
    )

    # Queue for mirroring serial bytes from simulavr thread -> WS task
    ws_queue: asyncio.Queue[bytes] = asyncio.Queue()

    async def pump_queue_to_ws():
        while True:
            data = await ws_queue.get()
            try:
                if data:
                    # Truncate hex print for readability
                    hx = data.hex()
                    print(f"MCU link bytes: {hx[:128]}{'…' if len(hx) > 128 else ''}")
                    await broadcaster.broadcast(data)
            finally:
                ws_queue.task_done()

    # Start server and pump task
    async with ws_server:
        pump_task = asyncio.create_task(pump_queue_to_ws())

        # Run simulavr in a dedicated thread to avoid blocking the loop
        loop = asyncio.get_running_loop()
        sim_thread = threading.Thread(
            target=run_simulavr, args=(args, args.elffile, loop, ws_queue), daemon=True
        )
        sim_thread.start()

        # Wait for signal to exit
        stop = asyncio.Future()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                asyncio.get_running_loop().add_signal_handler(sig, stop.set_result, None)
            except NotImplementedError:
                pass
        await stop

        pump_task.cancel()
        try:
            await pump_task
        except asyncio.CancelledError:
            pass


def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

