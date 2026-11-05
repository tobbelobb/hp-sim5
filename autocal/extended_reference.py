"""Asynchronous, best-effort Python events for the extended-reference recorder.

Recording must never stop or delay the scientific run. Events are queued and a
background thread owns the WebSocket and acknowledgement wait. A failed capture
degrades to a recorded error; it is not raised into the caller.
"""
import atexit
import json
import os
from pathlib import Path
import queue
import threading
import time


ACK_TIMEOUT_SECONDS = 10.0
CLOSE_TIMEOUT_SECONDS = 12.0
MAX_EVENTS = 4096
MAX_BYTES = 32 * 1024 * 1024
_STOP = object()
_lock = threading.RLock()
_queue = None
_queued_bytes = 0
_thread = None
_socket = None
_error = None
_last_error = None
_registered = False


def _degrade(reason):
    global _error, _last_error, _socket
    with _lock:
        message = str(reason)
        if _error is None:
            _error = message
        if _last_error is None:
            _last_error = message
        socket, _socket = _socket, None
    if socket is not None:
        try:
            socket.close()
        except Exception:
            pass


def _run(url):
    global _socket, _queued_bytes
    socket = None
    try:
        from websockets.sync.client import connect
        socket = connect(url, open_timeout=5, close_timeout=2)
        with _lock:
            if _error is not None:
                return
            _socket = socket
        while True:
            item = _queue.get()
            if item is _STOP:
                return
            data, size = item
            try:
                if _error is not None:
                    continue
                socket.send(data)
                reply = json.loads(socket.recv(timeout=ACK_TIMEOUT_SECONDS))
                if reply.get('type') != 'event_ack':
                    raise RuntimeError('Extended reference recorder did not acknowledge Python event')
            finally:
                with _lock:
                    _queued_bytes = max(0, _queued_bytes - size)
    except BaseException as error:
        _degrade(f'{type(error).__name__}: {error}')
    finally:
        if socket is not None:
            try:
                socket.close()
            except Exception:
                pass
        with _lock:
            if _socket is socket:
                _socket = None


def _start(url):
    global _last_error, _queue, _queued_bytes, _thread, _registered
    with _lock:
        if _error is not None:
            return False
        if _thread is not None and _thread.is_alive():
            return True
        try:
            _last_error = None
            _queue = queue.Queue(maxsize=MAX_EVENTS)
            _queued_bytes = 0
            _thread = threading.Thread(target=_run, args=(url,), name='autocal-reference-recorder', daemon=True)
            _thread.start()
        except Exception as error:
            _queue = _thread = None
            _queued_bytes = 0
            _degrade(f'{type(error).__name__}: {error}')
            return False
        if not _registered:
            atexit.register(close)
            _registered = True
        return True


def close():
    """Drain best effort without turning a recorder failure into a run failure."""
    global _queue, _queued_bytes, _thread, _socket, _error
    with _lock:
        queue_ = _queue
        thread = _thread
        failed = _error is not None
    if queue_ is None or thread is None:
        with _lock:
            if _thread is None:
                _socket = None
                if _error is not None:
                    _last_error = _error
                _error = None
        return
    if thread.is_alive():
        try:
            queue_.put(_STOP, timeout=CLOSE_TIMEOUT_SECONDS if not failed else 2)
        except queue.Full:
            _degrade('Extended reference recorder queue did not drain before close')
            while True:
                try:
                    queue_.get_nowait()
                except queue.Empty:
                    break
            with _lock:
                _queued_bytes = 0
            try:
                queue_.put_nowait(_STOP)
            except queue.Full:
                pass
    thread.join(timeout=CLOSE_TIMEOUT_SECONDS if not failed else 1)
    if thread.is_alive():
        _degrade('Extended reference recorder did not stop before close timeout')
        thread.join(timeout=1)
    with _lock:
        if not thread.is_alive():
            if _queue is queue_:
                _queue = None
            if _thread is thread:
                _thread = None
            _socket = None
            if _error is not None:
                _last_error = _error
            _error = None


def failure():
    """Return the first non-fatal recording failure, if any."""
    with _lock:
        return _error if _error is not None else _last_error


def enabled():
    return bool(os.environ.get('AUTOCAL_REFERENCE_WS'))


def emit(kind, **payload):
    global _queued_bytes
    url = os.environ.get('AUTOCAL_REFERENCE_WS')
    if not url:
        return
    event = dict(version=1, type='autocal_event', source='python',
                 wall_time_ms=time.time_ns() // 1_000_000, wall_time_source='python.time.time_ns',
                 monotonic_ms=time.monotonic_ns() / 1_000_000, pid=os.getpid(), kind=kind,
                 sim_time_s=None, sim_time_source='unavailable', payload=payload)
    try:
        data = json.dumps(event)
        size = len(data.encode('utf-8'))
    except Exception as error:
        _degrade(f'{type(error).__name__}: {error}')
        return
    if not _start(url):
        return
    overflow = None
    with _lock:
        if _error is not None or _queue is None:
            return
        if _queued_bytes + size > MAX_BYTES:
            overflow = 'byte limit'
        else:
            _queued_bytes += size
            try:
                _queue.put_nowait((data, size))
            except queue.Full:
                _queued_bytes = max(0, _queued_bytes - size)
                overflow = 'event limit'
    if overflow:
        _degrade(f'Extended reference recorder queue exceeded its {overflow}; capture stopped')


def artifact(path):
    if not enabled():
        return
    try:
        path = Path(path)
        if path.is_file():
            content = path.read_text(encoding='utf-8')
        else:
            return
    except Exception as error:
        _degrade(f'{type(error).__name__}: {error}')
        return
    emit('artifact', path=str(path.resolve()), content=content)


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
