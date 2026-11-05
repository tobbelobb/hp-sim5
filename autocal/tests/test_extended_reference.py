import io
import json
import time

from autocal import extended_reference


def test_event_log_preserves_exact_fragments_and_mirrors_redirected_output(monkeypatch, tmp_path):
    events = []
    monkeypatch.setattr(extended_reference, 'emit', lambda kind, **payload: events.append((kind, payload)))
    handle = io.StringIO()
    log = extended_reference.EventLog(handle, tmp_path / 'run.log')
    print('cost:', 1.25, file=log)
    assert handle.getvalue() == 'cost: 1.25\n'
    assert ''.join(payload['text'] for _, payload in events) == handle.getvalue()
    assert all(kind == 'text_log' for kind, _ in events)
    assert all(payload['path'] == str(tmp_path / 'run.log') for _, payload in events)


def test_artifact_embeds_original_content_only_when_enabled(monkeypatch, tmp_path):
    events = []
    monkeypatch.setattr(extended_reference, 'emit', lambda kind, **payload: events.append((kind, payload)))
    path = tmp_path / 'dataset.json'
    path.write_text('{"sweeps": []}\n')
    monkeypatch.delenv('AUTOCAL_REFERENCE_WS', raising=False)
    extended_reference.artifact(path)
    assert not events
    monkeypatch.setenv('AUTOCAL_REFERENCE_WS', 'ws://localhost:9877')
    extended_reference.artifact(path)
    assert events == [('artifact', dict(path=str(path), content=path.read_text()))]


def test_emit_queues_and_drains_acknowledged_events(monkeypatch):
    class Socket:
        def __init__(self):
            self.events = []

        def send(self, data):
            self.events.append(json.loads(data))

        def recv(self, timeout):
            return json.dumps({'type': 'event_ack'})

        def close(self):
            pass

    socket = Socket()
    monkeypatch.setenv('AUTOCAL_REFERENCE_WS', 'ws://fixture')
    monkeypatch.setattr('websockets.sync.client.connect', lambda *args, **kwargs: socket)
    extended_reference.emit('first', value=1)
    extended_reference.emit('second', value=2)
    extended_reference.close()
    assert [event['kind'] for event in socket.events] == ['first', 'second']


def test_ack_timeout_disables_capture_without_raising(monkeypatch):
    class SilentSocket:
        def send(self, data):
            pass

        def recv(self, timeout):
            raise TimeoutError('no acknowledgement')

        def close(self):
            pass

    monkeypatch.setenv('AUTOCAL_REFERENCE_WS', 'ws://fixture')
    monkeypatch.setattr('websockets.sync.client.connect', lambda *args, **kwargs: SilentSocket())
    started = time.monotonic()
    extended_reference.emit('first')
    assert time.monotonic() - started < 1
    deadline = time.monotonic() + 2
    while extended_reference.failure() is None and time.monotonic() < deadline:
        time.sleep(.01)
    assert 'TimeoutError' in extended_reference.failure()
    extended_reference.emit('later')
    extended_reference.close()
    assert 'TimeoutError' in extended_reference.failure()


def test_queue_byte_limit_disables_capture_without_raising(monkeypatch):
    class Socket:
        def send(self, data):
            pass

        def recv(self, timeout):
            return json.dumps({'type': 'event_ack'})

        def close(self):
            pass

    monkeypatch.setenv('AUTOCAL_REFERENCE_WS', 'ws://fixture')
    monkeypatch.setattr(extended_reference, 'MAX_BYTES', 1)
    monkeypatch.setattr('websockets.sync.client.connect', lambda *args, **kwargs: Socket())
    extended_reference.emit('first')
    assert 'byte limit' in extended_reference.failure()
    extended_reference.close()
