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
import collections
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
        self.lock = asyncio.Lock()
        # Prevent concurrent ws.send() on the same connection which can corrupt
        # ordering or raise runtime errors in the websockets library.
        self._send_locks: dict[WebSocketServerProtocol, asyncio.Lock] = {}

    async def register(self, ws: WebSocketServerProtocol):
        # Snapshot history and register client with its own send lock
        async with self.lock:
            history_snapshot = list(self.history)
            self.clients.add(ws)
            self._send_locks[ws] = asyncio.Lock()

        try:
            # Replay history using the per-client send lock to avoid concurrent sends
            if history_snapshot:
                lock = self._send_locks.get(ws)
                if lock is not None:
                    async with lock:
                        for payload in history_snapshot:
                            await ws.send(payload)
                else:
                    # Very unlikely: client removed mid-registration
                    return

            # Wait for client to close
            await ws.wait_closed()
        except Exception:
            # e.g., connection closed during history replay
            pass
        finally:
            # Remove client and its lock
            async with self.lock:
                self.clients.discard(ws)
                self._send_locks.pop(ws, None)

    async def broadcast(self, payload: str, *, add_to_history: bool = True):
        async with self.lock:
            # Add to history buffer
            if add_to_history and self.history_messages > 0:
                self.history.append(payload)
                if len(self.history) > self.history_messages:
                    del self.history[:len(self.history) - self.history_messages]
            # Snapshot current clients and their locks
            clients_to_send = [(ws, self._send_locks.get(ws)) for ws in self.clients]

        if not clients_to_send:
            return

        async def _send(ws: WebSocketServerProtocol, lock: Optional[asyncio.Lock]):
            try:
                if lock is None:
                    # Fallback: send without lock (shouldn't normally happen)
                    await ws.send(payload)
                else:
                    async with lock:
                        await ws.send(payload)
                return None
            except Exception as e:
                return e

        results = await asyncio.gather(*[_send(ws, lock) for ws, lock in clients_to_send], return_exceptions=False)

        # Clean up clients that failed to send
        failed_clients = [ws for (ws, _), result in zip(clients_to_send, results) if isinstance(result, Exception)]
        if failed_clients:
            async with self.lock:
                for client in failed_clients:
                    self.clients.discard(client)
                    self._send_locks.pop(client, None)


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
    """PTY-backed terminal.

    - write(data): AVR->Host direction (device TX). Writes to PTY master.
    - read(): Host->AVR direction (host TX). Reads from PTY master and
      schedules queueing of the same bytes for parsing.
    """

    def __init__(self, loop: asyncio.AbstractEventLoop, parser_queue: asyncio.Queue[bytes], *,
                 raw_logger=None):
        self.fd = -1
        self.loop = loop
        self.parser_queue = parser_queue
        self._raw_logger = raw_logger
        self._eof_sent = False

    def run(self, fd: int):
        self.fd = fd

    def write(self, data: bytes):
        # Bytes from AVR TX go to PTY
        if self.fd >= 0 and data:
            os.write(self.fd, data)
            if self._raw_logger is not None:
                try:
                    self._raw_logger.log('A2H', data)
                except Exception:
                    pass

    def read(self) -> bytes:
        try:
            # Read a reasonably large chunk to reduce tiny fragments
            data = os.read(self.fd, 1024)
            if data:
                self._eof_sent = False  # Reset on new data
                self.loop.call_soon_threadsafe(self.parser_queue.put_nowait, data)
                if self._raw_logger is not None:
                    try:
                        self._raw_logger.log('H2A', data)
                    except Exception:
                        pass
            elif not self._eof_sent:
                # EOF: forward a sentinel to unblock parser
                self.loop.call_soon_threadsafe(self.parser_queue.put_nowait, None)
                self._eof_sent = True
            return data
        except os.error as e:
            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                pysimulavr.SystemClock.Instance().stop()
        return b""


def create_pty(ptyname: str) -> int:
    """Create a PTY pair and set RAW mode on both ends.

    We explicitly configure the slave side to raw before Klipper opens it to
    avoid any initial line-discipline transformations that could corrupt the
    binary protocol (which would cause early parser resync and lost packets).
    """
    mfd, sfd = pty.openpty()
    slave_path = os.ttyname(sfd)
    # Refresh symlink
    try:
        os.unlink(ptyname)
    except os.error:
        pass
    os.symlink(slave_path, ptyname)
    try:
        os.chmod(slave_path, 0o666)
    except Exception as e:
        print(f"Warning: could not chmod PTY slave {slave_path}: {e}")

    # Helper to put an FD into 8N1 raw mode, non-canonical, no echo
    def _set_raw(fd: int):
        try:
            attr = termios.tcgetattr(fd)
        except termios.error:
            return
        # iflag
        attr[0] &= ~(
            termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
            termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
        # oflag
        attr[1] &= ~termios.OPOST
        # cflag
        attr[2] &= ~(termios.CSIZE | termios.PARENB)
        attr[2] |= termios.CS8
        # lflag
        attr[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
        # cc
        attr[6][termios.VMIN] = 0
        attr[6][termios.VTIME] = 0
        try:
            termios.tcsetattr(fd, termios.TCSAFLUSH, attr)
        except termios.error:
            pass

    # Set RAW on both ends; master also non-blocking for our reader
    _set_raw(sfd)  # important: configure the end Klipper opens
    _set_raw(mfd)
    try:
        flags = fcntl.fcntl(mfd, fcntl.F_GETFL)
        fcntl.fcntl(mfd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    except Exception:
        pass
    return mfd


def run_simulavr(options, elffile: str, loop: asyncio.AbstractEventLoop, parser_queue: asyncio.Queue[bytes]):
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

    io = TerminalIO(loop, parser_queue, raw_logger=raw_logger)

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
    parser.add_argument("elf_path", nargs="?", default=None, help="Path to AVR ELF (e.g., klipper.elf)")
    parser.add_argument("--elf", "--mcu-bin", dest="elffile", default=None,
                        help="Alias for positional ELF path (for drop-in compat)")
    parser.add_argument("-m", "--machine", default="atmega644", help="AVR device type")
    parser.add_argument("-s", "--speed", type=int, default=16000000, help="CPU speed (Hz)")
    parser.add_argument("-r", "--rate", dest="pacing_rate", type=float, default=0.0, help="Real-time pacing rate")
    parser.add_argument("-b", "--baud", type=int, default=250000, help="UART baud rate")
    parser.add_argument("-t", "--trace", default=None, help="Signals to trace (? for list)")
    parser.add_argument("-p", "--port", default="/tmp/klipper_host_mcu", help="PTY symlink path for Klipper")
    parser.add_argument("--host-path", dest="host_path", default=None,
                        help="Alias for --port (matches linux mcu bridge)")
    parser.add_argument("--raw-path", dest="raw_path", default=None,
                        help="Unused compatibility flag (linux bridge passthrough)")
    deffile = Path(sys.argv[0]).with_suffix(".vcd").name
    parser.add_argument("-f", "--tracefile", default=deffile, help="Trace VCD filename")
    parser.add_argument("--ws-host", default="127.0.0.1", help="WebSocket bind host")
    parser.add_argument("--ws-port", type=int, default=8770, help="WebSocket port")
    parser.add_argument("--ws-raw-batch-ms", type=int, default=2,
                        help="Batch raw-byte WS frames for N ms before sending (reduces overhead)")
    parser.add_argument("--parse-debug", action="store_true",
                        help="Print parser resync and packet summaries to stdout")
    parser.add_argument("--keep-noise", action="store_true",
                        help="Do not filter handshake/noise lines (identify, clock, allocate_oids, etc)")
    parser.add_argument("--no-dedup", action="store_true",
                        help="Disable packet deduplication (keep retransmitted frames)")
    parser.add_argument("--ws-history-messages", type=int, default=0,
                        help="Buffer and replay the most recent N parsed frames to new WS clients (0 disables)")
    parser.add_argument("--raw-log", dest="raw_log", default=None,
                        help="Log raw serial bytes with direction to this file (use '-' for stdout)")
    parser.add_argument("--dict", dest="dict_path", default=str(Path(__file__).resolve().parents[1] / "avr/klipper.dict"),
                        help="Path to Klipper MCU dictionary (klipper.dict)")
    parser.add_argument("--klipper-py", dest="klipper_py_path", default=None,
                        help="Path to Klipper repo (to import msgproto). Optional.")
    parser.add_argument("--klippy-log", dest="klippy_log", default=None,
                        help="Ignored placeholder for drop-in CLI compatibility.")

    args = parser.parse_args(argv)

    if args.host_path:
        args.port = args.host_path
    if args.raw_path:
        print("Note: --raw-path is ignored when using the simulavr bridge.")
    if args.klippy_log:
        print("Note: --klippy-log is ignored; clock updates come from parsed traffic.")

    elffile = args.elffile or args.elf_path
    if not elffile:
        candidates = [
            Path("out/klipper.elf"),
            Path(__file__).resolve().parents[1] / "avr/klipper.elf",
        ]
        for cand in candidates:
            if cand.exists():
                elffile = str(cand)
                break
    if not elffile:
        parser.error("No AVR ELF provided; pass path via positional argument or --elf.")
    args.elffile = elffile

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

    # Queue for serial bytes from simulavr thread -> parser task
    parser_queue: asyncio.Queue[bytes] = asyncio.Queue()

    async def pump_parser_queue(broadcaster: WSRawBroadcaster):
        # Optional parsed message setup
        mp = None
        parse_buf = bytearray()
        line_seq = 0  # Monotonic sequence across all emitted lines (for debugging)
        last_clock_hz: Optional[float] = None
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

        # Helper: try to locate a valid packet anywhere in the buffer
        def _scan_for_packet(buf: bytearray) -> tuple[int, int] | None:
            if not buf:
                return None
            # Slide across the buffer and run check_packet on each suffix
            # to find the earliest valid frame.
            for i in range(max(0, len(buf) - 1024), len(buf)):
                l = mp.check_packet(buf[i:]) if mp is not None else 0
                if l > 0 and i + l <= len(buf):
                    return (i, l)
            return None

        synced_once = False
        # Deduplicate recent packets to avoid processing host retransmissions
        # (common when early bytes are dropped and ACKs are missed). We keep a
        # small rolling window of raw packet bytes.
        recent_packets = collections.deque(maxlen=64)
        recent_set = set()

        while True:
            data = await parser_queue.get()
            try:
                if data is None:  # Got EOF sentinel
                    if parse_buf:
                        if args.parse_debug:
                            print(f"Parser EOF, discarding partial buffer ({len(parse_buf)} bytes)")
                        parse_buf.clear()
                    continue

                if data:
                    # If parser available, broadcast parsed text as JSON
                    if mp is not None:
                        parse_buf += data
                        while True:
                            l = mp.check_packet(parse_buf)
                            if l == 0:
                                break
                            if l < 0:
                                # On initial sync attempts, search the buffer for
                                # the first valid frame instead of dropping down to
                                # a 1-byte tail repeatedly.
                                found = _scan_for_packet(parse_buf)
                                if found is not None:
                                    start, l2 = found
                                    if start:
                                        if args.parse_debug:
                                            print(f"Parser resync: found frame at offset {start} len={l2} (buffer={len(parse_buf)})")
                                        # Drop bytes before the frame
                                        parse_buf[:] = parse_buf[start:]
                                    l = l2
                                else:
                                    if args.parse_debug:
                                        print(f"Parser resync: no frame found in buffer={len(parse_buf)}; keeping buffer for more data")
                                    # Avoid discarding; wait for more data
                                    break
                            # We have a valid packet of length l at start 0
                            try:
                                raw_pkt = bytes(parse_buf[:l])
                                if False and raw_pkt in recent_set:
                                    if args.parse_debug:
                                        print(f"Dedup: dropped retransmitted frame len={l}")
                                    # Consume and skip duplicate
                                    parse_buf = parse_buf[l:]
                                    continue
                                # Track packet in dedup set with bounded history
                                evicted = None
                                if recent_packets.maxlen is not None and len(recent_packets) == recent_packets.maxlen:
                                    # About to evict leftmost on append
                                    evicted = recent_packets[0]
                                recent_packets.append(raw_pkt)
                                recent_set.add(raw_pkt)
                                if evicted is not None:
                                    recent_set.discard(evicted)

                                msgs = mp.dump(raw_pkt)
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
                                    # Output lines as-is to match linux mcu bridge behavior (no renaming or filtering)
                                    out_lines = [str(s) for s in lines]
                                    filtered_lines = out_lines
                                    if filtered_lines:
                                        start_idx = line_seq
                                        line_seq += len(filtered_lines)
                                        payload = {
                                            'action': 'klipper_parsed',
                                            'lines': filtered_lines,
                                            'seq': start_idx,
                                            'count': len(filtered_lines),
                                        }
                                        await broadcaster.broadcast(json.dumps(payload), add_to_history=True)
                                        # Emit clock updates when we see set_clock packets
                                        clock_val = None
                                        for line in filtered_lines:
                                            if 'set_clock' not in line:
                                                continue
                                            for part in line.split():
                                                if part.startswith('clock='):
                                                    try:
                                                        val = float(part.split('=', 1)[1])
                                                    except ValueError:
                                                        continue
                                                    if val > 0:
                                                        clock_val = val
                                                    break
                                            if clock_val is not None:
                                                break
                                        if clock_val is not None and (last_clock_hz is None or abs(clock_val - last_clock_hz) > 1e-6):
                                            last_clock_hz = clock_val
                                            await broadcaster.broadcast(json.dumps({'action': 'klipper_clock', 'clock_hz': clock_val}), add_to_history=True)
                            except Exception as e:
                                if args.parse_debug:
                                    print(f"Parser error: {e}")
                            parse_buf = parse_buf[l:]
            finally:
                parser_queue.task_done()

    # Start server and pump task
    async with ws_server:
        pump_task = asyncio.create_task(pump_parser_queue(broadcaster))

        # Run simulavr in a dedicated thread to avoid blocking the loop
        loop = asyncio.get_running_loop()
        sim_thread = threading.Thread(
            target=run_simulavr, args=(args, args.elffile, loop, parser_queue), daemon=True
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
