#!/usr/bin/env python3
"""
Klipper AVR Bridge (SimulAVR + WebSocket)
-----------------------------------------

Purpose
  - Emulate an AVR MCU using pysimulavr, exposing a PTY that Klipper can open.
  - Forward ALL serial-link bytes (both directions) to connected WebSocket
    clients as raw binary frames (incl parsing).
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
import json
from typing import Optional, TextIO

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

# Optional Klipper msgproto (imported later after args allow sys.path tweaks)
msgproto = None


SERIALBITS = 10  # 8N1
SIMULAVR_FREQ = 10 ** 9


class WSRawBroadcaster:
    def __init__(self, history_messages: int = 5000):
        self.clients: set[WebSocketServerProtocol] = set()
        # Store recent parsed-message frames to replay to late-joining clients
        self.history: list[str] = []
        self.history_messages = max(0, history_messages)

    async def register(self, ws: WebSocketServerProtocol):
        # Add client and replay recent history
        self.clients.add(ws)
        try:
            if self.history:
                for payload in self.history:
                    try:
                        await ws.send(payload)
                    except Exception:
                        break
            await ws.wait_closed()
        finally:
            self.clients.discard(ws)

    async def broadcast(self, payload: str, *, add_to_history: bool = True):
        # Optionally append to history
        if add_to_history and self.history_messages > 0:
            self.history.append(payload)
            if len(self.history) > self.history_messages:
                del self.history[:len(self.history) - self.history_messages]
        # Send to all current clients
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


async def ws_handler(ws: WebSocketServerProtocol, broadcaster: WSRawBroadcaster):
    # Accept connection and replay recent history, ignore incoming frames
    await broadcaster.register(ws)


class RawLogger:
    """Thread-safe raw byte logger with direction tagging.

    Directions:
      - 'H2A': Host -> AVR (what Klipper writes)
      - 'A2H': AVR -> Host (what the MCU writes)
    """

    def __init__(self, fp: TextIO):
        self._fp: TextIO = fp
        self._lock = threading.Lock()
        self._t0 = time.monotonic()
        # Emit a small banner to help alignment
        try:
            self._fp.write("# raw-log start: ts=seconds_since_start direction len | hex | ascii\n")
            self._fp.flush()
        except Exception:
            pass

    def _hex(self, data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)

    def _ascii(self, data: bytes) -> str:
        return "".join(chr(b) if 32 <= b < 127 else '.' for b in data)

    def log(self, direction: str, data: bytes):
        if not data:
            return
        ts = time.monotonic() - self._t0
        line = f"[{ts:9.6f}] {direction} {len(data)}: {self._hex(data)} |{self._ascii(data)}|\n"
        with self._lock:
            self._fp.write(line)
            try:
                self._fp.flush()
            except Exception:
                pass

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
        # Anchor wall-clock to simulated clock using monotonic time
        self.start_monotonic = time.monotonic()
        self.delay = SIMULAVR_FREQ // 10000
        self.sc.Add(self)

    def DoStep(self, trueHwStep):
        # Compute how far simulated time is ahead/behind wall-time
        curtime = time.monotonic()
        clock = self.sc.GetCurrentTime()
        target_time = self.start_monotonic + clock * self.pacing_rate
        offset = target_time - curtime
        # If running ahead of real time, sleep to realign; otherwise, don't stall
        if offset > 0:
            # Cap sleep to avoid oversleep on long offsets
            time.sleep(min(offset, 0.005))
        # Periodically re-anchor to eliminate accumulated drift
        if clock >= self.next_check_clock:
            # Make target_time align with current monotonic time
            self.start_monotonic = time.monotonic() - clock * self.pacing_rate
            self.next_check_clock = clock + self.delay * 500
        return self.delay


class SerialRxPin(pysimulavr.PySimulationMember, pysimulavr.Pin):
    """AVR TXD -> Terminal.write(data)

    Captures bits from the device TX pin and emits whole bytes to the
    terminal, which writes to WS.
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

    - write(data): AVR->Host direction (device TX). Writes to PTY master.
    - read(): Host->AVR direction (host TX). Reads from PTY master and
      schedules WS broadcast of the same bytes.
    """

    def __init__(self, loop: asyncio.AbstractEventLoop, ws_queue: asyncio.Queue[bytes], *,
                 ws_raw_batch_ms: int = 5, raw_logger=None):
        self.fd = -1
        self.loop = loop
        self.ws_queue = ws_queue
        self._raw_logger = raw_logger
        # Raw WS batching to reduce overhead
        self._batch_ms = max(0, ws_raw_batch_ms) / 1000.0
        self._accum = bytearray()
        self._accum_lock = threading.Lock()
        self._flush_scheduled = False

    def run(self, fd: int):
        self.fd = fd

    def _flush_now(self):
        # Runs in event loop
        with self._accum_lock:
            payload = bytes(self._accum)
            self._accum.clear()
            self._flush_scheduled = False
        if payload:
            self.ws_queue.put_nowait(payload)

    def _schedule_flush(self):
        # Runs in event loop
        if self._flush_scheduled:
            return
        self._flush_scheduled = True
        if self._batch_ms > 0:
            self.loop.call_later(self._batch_ms, self._flush_now)
        else:
            self.loop.call_soon(self._flush_now)

    def _mirror_ws(self, data: bytes):
        if not data:
            return
        # Accumulate in thread-safe buffer and schedule a single flush soon
        with self._accum_lock:
            self._accum.extend(data)
        self.loop.call_soon_threadsafe(self._schedule_flush)

    def write(self, data: bytes):
        # Bytes from AVR TX go to PTY
        if self.fd >= 0 and data:
            os.write(self.fd, data)
            self._mirror_ws(data)
            if self._raw_logger is not None:
                try:
                    self._raw_logger.log('A2H', data)
                except Exception:
                    pass

    def read(self) -> bytes:
        try:
            data = os.read(self.fd, 64)
            if data:
                self._mirror_ws(data)
                if self._raw_logger is not None:
                    try:
                        self._raw_logger.log('H2A', data)
                    except Exception:
                        pass
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

    # Optional raw logger
    raw_logger = None
    if getattr(options, 'raw_log', None):
        path = options.raw_log
        try:
            if path == '-' or path.lower() == 'stdout':
                raw_logger = RawLogger(sys.stdout)
            else:
                # Open in append text mode to allow multiple runs
                fp = open(path, 'a', buffering=1)
                raw_logger = RawLogger(fp)
        except Exception as e:
            print(f"Warning: failed to open raw log '{path}': {e}")
            raw_logger = None

    io = TerminalIO(loop, ws_queue, ws_raw_batch_ms=options.ws_raw_batch_ms, raw_logger=raw_logger)

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
        # Close raw logger file if not stdout
        try:
            if raw_logger is not None and raw_logger._fp not in (sys.stdout, sys.stderr):
                raw_logger._fp.close()
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
    parser.add_argument("--ws-raw-batch-ms", type=int, default=2,
                        help="Batch raw-byte WS frames for N ms before sending (reduces overhead)")
    parser.add_argument("--parse-debug", action="store_true",
                        help="Print parser resync and packet summaries to stdout")
    parser.add_argument("--ws-history-messages", type=int, default=5000,
                        help="Buffer and replay the most recent N parsed frames to new WS clients (0 disables)")
    parser.add_argument("--raw-log", dest="raw_log", default=None,
                        help="Log raw serial bytes with direction to this file (use '-' for stdout)")
    parser.add_argument("--dict", dest="dict_path", default=str(Path(__file__).resolve().parents[1] / "avr/klipper.dict"),
                        help="Path to Klipper MCU dictionary (klipper.dict)")
    parser.add_argument("--klipper-py", dest="klipper_py_path", default=None,
                        help="Path to Klipper repo (to import msgproto). Optional.")

    args = parser.parse_args(argv)

    broadcaster = WSRawBroadcaster(history_messages=args.ws_history_messages)

    # Attempt to import Klipper's msgproto if path provided / available
    global msgproto
    if args.klipper_py_path:
        sys.path.insert(0, args.klipper_py_path)
    try:
        import msgproto as _mp  # type: ignore
        msgproto = _mp
    except Exception:
        msgproto = None
        print("Note: Could not import klipper 'msgproto'. Parsed message output disabled.")

    # WS server
    ws_server = websockets.serve(
        lambda ws: ws_handler(ws, broadcaster),
        host=args.ws_host,
        port=args.ws_port,
        max_size=None,
        max_queue=None,
    )

    # Queue for mirroring serial bytes from simulavr thread -> WS task
    ws_queue: asyncio.Queue[bytes] = asyncio.Queue()

    async def pump_queue_to_ws():
        # Optional parsed message setup
        mp = None
        parse_buf = bytearray()
        if msgproto is not None and args.dict_path and os.path.exists(args.dict_path):
            try:
                with open(args.dict_path, 'rb') as dfile:
                    dictionary = dfile.read()
                mp = msgproto.MessageParser()
                mp.process_identify(dictionary, decompress=False)
                print(f"Loaded Klipper dictionary from: {args.dict_path} ({len(dictionary)} bytes)")
            except Exception as e:
                print(f"Warning: Failed to initialize msgproto parser: {e}")
                mp = None
        else:
            print("Note: Parser not initialized (no msgproto or missing dict). Only raw bytes will be forwarded.")

        while True:
            data = await ws_queue.get()
            try:
                if data:
                    # If parser available, broadcast parsed text as JSON
                    if mp is not None:
                        parse_buf += data
                        while True:
                            l = mp.check_packet(parse_buf)
                            if l == 0:
                                break
                            if l < 0:
                                # Invalid data: keep the indicated tail
                                tail = -l
                                if args.parse_debug:
                                    print(f"Parser resync: keeping tail {tail} bytes (buffer={len(parse_buf)})")
                                parse_buf[:] = parse_buf[-tail:]
                                continue
                            try:
                                msgs = mp.dump(parse_buf[:l])
                                # Normalize to a list of human-readable lines
                                lines: list[str] = []
                                def _collect(obj):
                                    if obj is None:
                                        return
                                    if isinstance(obj, (bytes, bytearray)):
                                        for ln in obj.decode('utf-8', errors='ignore').splitlines():
                                            if ln.strip():
                                                lines.append(ln)
                                    elif isinstance(obj, str):
                                        for ln in obj.splitlines():
                                            if ln.strip():
                                                lines.append(ln)
                                    elif isinstance(obj, (list, tuple)):
                                        for it in obj:
                                            _collect(it)
                                    else:
                                        s = str(obj)
                                        for ln in s.splitlines():
                                            if ln.strip():
                                                lines.append(ln)
                                _collect(msgs)

                                if lines:
                                    if args.parse_debug:
                                        print(f"Parsed packet: {l} bytes -> {len(lines)} line(s). First: {lines[0][:120]}")
                                    # Inject deterministic stepper names (A-D) based on fixed OID mapping.
                                    # Ignore extruder and any other steppers beyond oid=3.
                                    oid_to_name = {0: 'stepper_a', 1: 'stepper_b', 2: 'stepper_c', 3: 'stepper_d'}
                                    out_lines: list[str] = []
                                    for ln in lines:
                                        s = ln
                                        try:
                                            s = str(s)
                                        except Exception:
                                            pass
                                        if 'config_stepper ' in s:
                                            try:
                                                rest = s.split('config_stepper ', 1)[1]
                                                oid = None
                                                for part in rest.split():
                                                    if part.startswith('oid='):
                                                        oid = int(part[4:])
                                                        break
                                                if oid is not None and oid in oid_to_name and ' name=' not in s:
                                                    s = f"{s} name={oid_to_name[oid]}"
                                            except Exception:
                                                # On any parsing error, leave the line unchanged
                                                pass
                                        out_lines.append(s)
                                    # Filter out noisy handshake chatter that floods the WS and isn't needed for visualization
                                    def _is_noise_line(s: str) -> bool:
                                        s = s.strip()
                                        if not s:
                                            return True
                                        cmd = s.split()[0]
                                        return cmd in ('identify', 'identify_response', 'get_clock', 'clock', 'allocate_oids', 'emergency_stop')
                                    filtered_lines = [s for s in out_lines if not _is_noise_line(s)]
                                    if filtered_lines:
                                        payload = {
                                            'action': 'klipper_parsed',
                                            'lines': filtered_lines,
                                        }
                                        await broadcaster.broadcast(json.dumps(payload), add_to_history=True)
                            except Exception as e:
                                if args.parse_debug:
                                    print(f"Parser error: {e}")
                            parse_buf = parse_buf[l:]
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
