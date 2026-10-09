import io

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
