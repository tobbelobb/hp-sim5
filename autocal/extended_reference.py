"""Live Python events for the shared extended-reference flight recorder."""
import atexit
import os
from pathlib import Path
import time
import json

_socket = None


def enabled():
    return bool(os.environ.get('AUTOCAL_REFERENCE_WS'))


def emit(kind, **payload):
    global _socket
    if not enabled():
        return
    event = dict(version=1, type='autocal_event', source='python',
                 wall_time_ms=time.time_ns() // 1_000_000, wall_time_source='python.time.time_ns',
                 monotonic_ms=time.monotonic_ns() / 1_000_000, pid=os.getpid(), kind=kind,
                 sim_time_s=None, sim_time_source='unavailable', payload=payload)
    if _socket is None:
        from websockets.sync.client import connect
        _socket = connect(os.environ['AUTOCAL_REFERENCE_WS'], open_timeout=5, close_timeout=2)
        atexit.register(_socket.close)
    _socket.send(json.dumps(event))
    reply = json.loads(_socket.recv(timeout=10))
    if reply.get('type') != 'event_ack':
        raise RuntimeError('Extended reference recorder did not acknowledge Python event')


def artifact(path):
    path = Path(path)
    if enabled() and path.is_file():
        emit('artifact', path=str(path.resolve()), content=path.read_text(encoding='utf-8'))


class EventLog:
    """Mirror exact text writes, including optimizer stdout redirected to the log."""
    def __init__(self, handle, path):
        self.handle = handle
        self.path = str(Path(path).resolve())

    def write(self, text):
        result = self.handle.write(text)
        self.handle.flush()
        if text:
            emit('text_log', text=text, path=self.path)
        return result

    def __getattr__(self, name):
        return getattr(self.handle, name)
