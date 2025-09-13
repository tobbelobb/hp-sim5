#!/usr/bin/env python3
"""
Klipper Linux MCU PTY Bridge (+ WebSocket mirror)
-------------------------------------------------

Runs the Linux "process MCU" (klipper_mcu) with its PTY exposed at
`/tmp/klipper_host_mcu_raw`, presents a second PTY at `/tmp/klipper_host_mcu`
for Klippy to open, and forwards bytes bidirectionally between them.

Additionally, it parses host->MCU (Klippy->firmware) packets using a
Klipper firmware dictionary (klipper.dict) and mirrors a filtered subset
of commands to a WebSocket at ws://<host>:8770/.

Intended as a bare-bones, low-latency drop-in for simulavr-based bridges.
"""

from __future__ import annotations

import argparse
import asyncio
import errno
import fcntl
import json
import os
import pathlib
import pty
import shlex
import shutil
import signal
import sys
import termios
import time
import re
from typing import List, Optional, Set

# Optional Klipper msgproto import (supports parsing commands using klipper.dict)
_msgproto = None


# ----------------------------- WebSocket ------------------------------------

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except Exception:
    websockets = None
    WebSocketServerProtocol = object  # type: ignore


class WSManager:
    def __init__(self):
        self._clients: Set[WebSocketServerProtocol] = set()
        self._lock = asyncio.Lock()

    async def register(self, ws: WebSocketServerProtocol):
        async with self._lock:
            self._clients.add(ws)
        try:
            await ws.wait_closed()
        finally:
            async with self._lock:
                self._clients.discard(ws)

    async def broadcast_json(self, payload_obj: dict):
        data = json.dumps(payload_obj)
        async with self._lock:
            clients = list(self._clients)
        if not clients:
            return
        # Send sequentially to reduce locking/CPU overhead (message volumes are small)
        for ws in clients:
            try:
                await ws.send(data)
            except Exception:
                # Drop failed clients
                try:
                    await ws.close()
                except Exception:
                    pass
                async with self._lock:
                    self._clients.discard(ws)


# -------------------------- PTY + termios utils -----------------------------

def _set_raw(fd: int) -> None:
    try:
        attr = termios.tcgetattr(fd)
    except termios.error:
        return
    # iflag
    attr[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                 termios.ISTRIP | termios.INLCR | termios.IGNCR |
                 termios.ICRNL | termios.IXON)
    # oflag
    attr[1] &= ~termios.OPOST
    # cflag
    attr[2] &= ~(termios.CSIZE | termios.PARENB)
    attr[2] |= termios.CS8
    # lflag
    attr[3] &= ~(termios.ECHONL | termios.ECHO | termios.ICANON |
                 termios.ISIG | termios.IEXTEN)
    # cc: VMIN/VTIME
    attr[6][termios.VMIN] = 0
    attr[6][termios.VTIME] = 0
    try:
        termios.tcsetattr(fd, termios.TCSAFLUSH, attr)
    except termios.error:
        pass


def _set_nonblocking(fd: int) -> None:
    try:
        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    except Exception:
        pass


def create_pty_symlink(symlink_path: str) -> int:
    """Create a new PTY pair and symlink its slave to symlink_path.

    Returns the master fd (int) which Klippy traffic will be read/written on.
    """
    mfd, sfd = pty.openpty()
    # Configure both ends for raw binary traffic
    _set_raw(sfd)
    _set_raw(mfd)
    _set_nonblocking(mfd)
    # Link slave to the desired symlink path (replacing any existing link)
    try:
        if os.path.islink(symlink_path) or os.path.exists(symlink_path):
            os.unlink(symlink_path)
    except Exception:
        pass
    tname = os.ttyname(sfd)
    os.symlink(tname, symlink_path)
    return mfd


def open_tty_rw(path: str) -> int:
    # Open the slave end the Linux MCU exposes (read/write, non-blocking, raw)
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    _set_raw(fd)
    _set_nonblocking(fd)
    return fd


# ----------------------------- Parser / Mirror ------------------------------

class ParsedMirror:
    """Parses host->MCU packets, filters commands, and mirrors to WS.

    Flush policy: flush immediately at packet boundary; additionally, a
    coalescing timer of 1ms ensures bursts are grouped; if buffered payload
    exceeds 16KiB, flush immediately.
    """

    def __init__(self, ws: WSManager, mp, allowed_cmds: Set[str], *,
                 batch_ms: float = 0.001, max_bytes: int = 16 * 1024):
        self.ws = ws
        self.mp = mp
        self.allowed = allowed_cmds
        self.batch_ms = batch_ms
        self.max_bytes = max_bytes
        self.buf = bytearray()
        self.lines: List[str] = []
        self.bytes_acc = 0
        self._flush_task: Optional[asyncio.Task] = None
        self._seq = 0

    async def feed(self, data: bytes):
        if not data:
            return
        self.buf.extend(data)
        # Parse all complete packets currently in buffer
        while True:
            l = self.mp.check_packet(self.buf)
            if l <= 0:
                break
            pkt = bytes(self.buf[:l])
            # Decode to lines (may be multiple commands in a block)
            try:
                decoded_lines = self.mp.dump(pkt)
            except Exception:
                decoded_lines = []
            # Filter subset
            for ln in decoded_lines:
                name = ln.split()[0] if ln else ''
                if name in self.allowed:
                    self.lines.append(ln)
                    self.bytes_acc += len(ln) + 1
            # Consume packet
            del self.buf[:l]
            # Flush immediately at packet boundary if anything collected
            if self.lines:
                await self._maybe_flush(immediate=True)
        # If no full packet yet, ensure a near-term flush is scheduled
        if self.lines:
            await self._maybe_flush(immediate=False)

    async def _maybe_flush(self, *, immediate: bool):
        if immediate or self.bytes_acc >= self.max_bytes:
            await self._flush()
            return
        if self._flush_task is None or self._flush_task.done():
            self._flush_task = asyncio.create_task(self._delayed_flush())

    async def _delayed_flush(self):
        try:
            await asyncio.sleep(self.batch_ms)
            await self._flush()
        except asyncio.CancelledError:
            pass

    async def _flush(self):
        if not self.lines:
            return
        payload = {
            'action': 'klipper_parsed',
            'lines': self.lines,
            'seq': self._seq,
            'count': len(self.lines),
        }
        self._seq += len(self.lines)
        self.lines = []
        self.bytes_acc = 0
        await self.ws.broadcast_json(payload)


# ----------------------------- Bridge core ----------------------------------

class FDBridge:
    """Bidirectional non-blocking fd forwarder with parser on A->B path.

    A: PTY master (Klippy side)
    B: Linux MCU raw symlink (klipper_mcu PTY slave)
    """

    def __init__(self, loop: asyncio.AbstractEventLoop, fd_a: int, fd_b: int,
                 parser: Optional[ParsedMirror] = None):
        self.loop = loop
        self.fd_a = fd_a
        self.fd_b = fd_b
        self.parser = parser
        # Pending output buffers
        self.ab_buf = bytearray()  # A -> B
        self.ba_buf = bytearray()  # B -> A
        self._a_writer = False
        self._b_writer = False

    def start(self):
        self.loop.add_reader(self.fd_a, self._on_read_a)
        self.loop.add_reader(self.fd_b, self._on_read_b)

    def close(self):
        try:
            self.loop.remove_reader(self.fd_a)
        except Exception:
            pass
        try:
            self.loop.remove_reader(self.fd_b)
        except Exception:
            pass
        try:
            os.close(self.fd_a)
        except Exception:
            pass
        try:
            os.close(self.fd_b)
        except Exception:
            pass

    def _on_read_a(self):
        try:
            data = os.read(self.fd_a, 4096)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            # Treat other errors as hangup
            self.close()
            return
        if not data:
            # EOF/hangup
            self.close()
            return
        # Queue write to B
        self.ab_buf += data
        self._ensure_writer_b()
        # Feed parser (host->MCU path)
        if self.parser is not None:
            # Schedule feed without blocking reader
            asyncio.ensure_future(self.parser.feed(data))

    def _on_read_b(self):
        try:
            data = os.read(self.fd_b, 4096)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            self.close()
            return
        if not data:
            self.close()
            return
        self.ba_buf += data
        self._ensure_writer_a()

    def _ensure_writer_a(self):
        if self._a_writer:
            return
        self._a_writer = True
        self.loop.add_writer(self.fd_a, self._on_write_a)

    def _ensure_writer_b(self):
        if self._b_writer:
            return
        self._b_writer = True
        self.loop.add_writer(self.fd_b, self._on_write_b)

    def _on_write_a(self):
        if not self.ba_buf:
            self._a_writer = False
            try:
                self.loop.remove_writer(self.fd_a)
            except Exception:
                pass
            return
        try:
            n = os.write(self.fd_a, self.ba_buf)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            self.close()
            return
        if n:
            del self.ba_buf[:n]

    def _on_write_b(self):
        if not self.ab_buf:
            self._b_writer = False
            try:
                self.loop.remove_writer(self.fd_b)
            except Exception:
                pass
            return
        try:
            n = os.write(self.fd_b, self.ab_buf)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            self.close()
            return
        if n:
            del self.ab_buf[:n]


# ------------------------------- Main ---------------------------------------

ALLOWED_COMMANDS = {
    'allocate_oids',
    'config_analog_in',
    'config_digital_out',
    'set_digital_out_pwm_cycle',
    'config_stepper',
    'finalize_config',
    'query_analog_in',
    'queue_digital_out',
    'set_next_step_dir',
    'set_position',
    'queue_step',
}


async def main_async(argv=None):
    parser = argparse.ArgumentParser(description="Linux MCU PTY bridge + WebSocket mirror")
    parser.add_argument('--raw-path', default='/tmp/klipper_host_mcu_raw',
                        help='Path klipper_mcu exposes (via -I).')
    parser.add_argument('--host-path', default='/tmp/klipper_host_mcu',
                        help='Path presented to Klippy (symlink to our PTY).')
    parser.add_argument('--mcu-bin', default=None,
                        help='Path to klipper_mcu binary (default: out/klipper.elf if present, else klipper_mcu in PATH).')
    parser.add_argument('--dict', dest='dict_path', default='out/klipper.dict',
                        help='Path to Klipper firmware dictionary (klipper.dict).')
    parser.add_argument('--klipper-py', dest='klipper_py_path', default=None,
                        help='Path to Klipper repo directory (to import msgproto). Optional.')
    parser.add_argument('--ws-host', default='127.0.0.1', help='WebSocket bind host')
    parser.add_argument('--ws-port', type=int, default=8770, help='WebSocket port')
    parser.add_argument('--no-ws', action='store_true', help='Disable WebSocket server')
    # Keep the default coalescing window tiny so parsed lines are visible
    # almost immediately even if packets arrive in bursts.
    parser.add_argument('--batch-ms', type=float, default=0.001,
                        help='Coalescing window for WS flush (seconds).')
    parser.add_argument('--max-bytes', type=int, default=16*1024,
                        help='Flush to WS when buffered payload exceeds this size.')
    parser.add_argument('--klippy-log', default=None,
                        help='Optional path to Klippy log (klippy.log). If set, the bridge tails it and broadcasts measured freq as {action:"klipper_clock", clock_hz:<number>} to WS clients.')
    args = parser.parse_args(argv)

    if websockets is None and not args.no_ws:
        print("Error: The 'websockets' package is required. pip install websockets")
        return 2

    # Import msgproto if available
    global _msgproto
    if args.klipper_py_path:
        sys.path.insert(0, args.klipper_py_path)
    try:
        try:
            import msgproto as _mp  # type: ignore
        except Exception:
            from klippy import msgproto as _mp  # type: ignore
        _msgproto = _mp
    except Exception:
        _msgproto = None
        print("Warning: Could not import Klipper msgproto; parsed mirror disabled.")

    # Start klipper_mcu process
    mcu_bin = args.mcu_bin
    if not mcu_bin:
        # Prefer out/klipper.elf if it exists in current repo
        elf = pathlib.Path('out/klipper.elf')
        if elf.exists():
            mcu_bin = str(elf)
        else:
            mcu_bin = shutil.which('klipper_mcu') or 'klipper_mcu'
    cmd = [mcu_bin, '-I', args.raw_path]
    print(f"Launching: {' '.join(shlex.quote(c) for c in cmd)}")
    mcu_proc = await asyncio.create_subprocess_exec(*cmd,
                                                    stdout=asyncio.subprocess.PIPE,
                                                    stderr=asyncio.subprocess.PIPE)

    # Create host-facing PTY and symlink
    master_fd = create_pty_symlink(args.host_path)

    # Wait for raw path to appear (created by klipper_mcu)
    deadline = time.monotonic() + 10.0
    while not os.path.exists(args.raw_path):
        if time.monotonic() > deadline:
            print(f"Error: Timed out waiting for {args.raw_path} to appear.")
            try:
                mcu_proc.terminate()
            except Exception:
                pass
            return 3
        await asyncio.sleep(0.05)

    raw_fd = open_tty_rw(args.raw_path)

    # WebSocket server + parser setup
    ws_mgr = WSManager()
    if not args.no_ws:
        ws_server = websockets.serve(lambda ws: ws_mgr.register(ws),
                                     host=args.ws_host, port=args.ws_port,
                                     max_size=None, max_queue=None)
    else:
        ws_server = None

    # Initialize msgproto parser from dictionary
    parser_obj: Optional[ParsedMirror] = None
    if _msgproto is not None and args.dict_path and os.path.exists(args.dict_path):
        try:
            with open(args.dict_path, 'rb') as f:
                dictionary = f.read()
            mp = _msgproto.MessageParser()
            mp.process_identify(dictionary, decompress=False)
            parser_obj = ParsedMirror(ws_mgr, mp, ALLOWED_COMMANDS,
                                      batch_ms=args.batch_ms,
                                      max_bytes=args.max_bytes)
            print(f"Loaded Klipper dictionary: {args.dict_path} ({len(dictionary)} bytes)")
        except Exception as e:
            print(f"Warning: Failed to initialize msgproto parser: {e}")
            parser_obj = None
    else:
        if _msgproto is None:
            print("Note: msgproto unavailable; WS mirror will still run but without parsed lines.")
        else:
            print(f"Note: dictionary not found: {args.dict_path}")

    # Start forwarding
    loop = asyncio.get_running_loop()
    bridge = FDBridge(loop, master_fd, raw_fd, parser=parser_obj)
    bridge.start()

    # Consume klipper_mcu stdout/stderr (avoid filling pipes)
    async def _drain_stream(prefix: str, stream: Optional[asyncio.StreamReader]):
        if stream is None:
            return
        try:
            while True:
                line = await stream.readline()
                if not line:
                    break
                sys.stdout.write(f"[{prefix}] {line.decode(errors='replace')}")
        except asyncio.CancelledError:
            pass

    drain_out = asyncio.create_task(_drain_stream('mcu:out', mcu_proc.stdout))
    drain_err = asyncio.create_task(_drain_stream('mcu:err', mcu_proc.stderr))

    # Handle shutdown signals
    stop = asyncio.Future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop.set_result, None)
        except NotImplementedError:
            pass

    print(f"Ready: Klippy can connect to {args.host_path}. WS on {args.ws_host}:{args.ws_port}.")

    # Optional: tail Klippy log for measured freq updates
    tail_task = None
    if args.klippy_log and not args.no_ws:
        log_path = args.klippy_log

        async def tail_klippy_log(path: str, ws: WSManager):
            last_clock = None
            file_inode = None
            f = None
            try:
                while True:
                    try:
                        st = os.stat(path)
                        if file_inode is None or st.st_ino != file_inode:
                            if f:
                                try:
                                    f.close()
                                except Exception:
                                    pass
                            f = open(path, 'r', errors='ignore')
                            file_inode = st.st_ino
                            # Read from beginning to catch an earlier freq line
                            f.seek(0, os.SEEK_SET)
                    except FileNotFoundError:
                        # File not ready yet – wait and retry
                        await asyncio.sleep(0.5)
                        continue

                    line = f.readline()
                    if not line:
                        await asyncio.sleep(0.25)
                        continue
                    m = re.search(r"freq=(\d+(?:\.\d+)?)", line)
                    if m:
                        try:
                            val = float(m.group(1))
                            # Broadcast only on change
                            if val > 0 and val != last_clock:
                                last_clock = val
                                await ws.broadcast_json({'action': 'klipper_clock', 'clock_hz': val})
                        except Exception:
                            pass
            finally:
                if f:
                    try:
                        f.close()
                    except Exception:
                        pass

        tail_task = asyncio.create_task(tail_klippy_log(log_path, ws_mgr))

    # Run servers and wait for stop
    if ws_server is not None:
        async with ws_server:
            await stop
    else:
        await stop

    # Cleanup
    try:
        bridge.close()
    except Exception:
        pass
    for t in (drain_out, drain_err):
        t.cancel()
        try:
            await t
        except Exception:
            pass
    if tail_task is not None:
        tail_task.cancel()
        try:
            await tail_task
        except Exception:
            pass
    try:
        mcu_proc.terminate()
    except Exception:
        pass
    try:
        os.unlink(args.host_path)
    except Exception:
        pass


def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
