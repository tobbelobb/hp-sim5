import json
import os
from contextlib import contextmanager
from concurrent.futures import ThreadPoolExecutor
from threading import Event

import pytest
from websockets.sync.client import connect

from autocal import autocal as ac, extended_reference
from autocal.browser_recording import browser_recording


@pytest.mark.parametrize("flag", ["--extended-recording", "--recording", "--record"])
def test_recording_alias_owns_browser_recorder(tmp_path, monkeypatch, flag):
    import autocal.browser_recording as module
    monkeypatch.delenv("AUTOCAL_REFERENCE_WS", raising=False)
    dataset = tmp_path / "sweeps.json"
    calls = []

    @contextmanager
    def recording(path):
        calls.append(("start", path))
        yield tmp_path
        calls.append(("stop", path))

    monkeypatch.setattr(module, "browser_recording", recording)
    monkeypatch.setattr(ac, "_run_full_auto", lambda *args: 0)
    assert ac.main(["--sim", "--machine-type", "hp4", "--dataset", str(dataset), flag]) == 0
    assert calls == [("start", dataset), ("stop", dataset)]


@pytest.mark.parametrize("fail", [False, True])
def test_owned_recorder_drains_browser_and_finalizes(tmp_path, monkeypatch, fail):
    monkeypatch.delenv("AUTOCAL_REFERENCE_WS", raising=False)
    dataset = tmp_path / "sweeps.json"

    registered = Event()

    def browser(url):
        with connect(url) as socket:
            socket.send(json.dumps(dict(version=1, type="event_source", source="browser")))
            assert json.loads(socket.recv(timeout=10))["type"] == "ack"
            registered.set()
            assert json.loads(socket.recv(timeout=30))["type"] == "recording_stop"
            # Last queued batch still receives its acknowledgement during shutdown.
            socket.send(json.dumps(dict(version=1, type="autocal_event", source="browser",
                wall_time_ms=1, kind="shutdown_test", payload={})))
            assert json.loads(socket.recv(timeout=10))["type"] == "event_ack"

    with ThreadPoolExecutor() as executor:
        try:
            with browser_recording(dataset) as directory:
                extended_reference.emit("test_start", message="owned recording")
                future = executor.submit(browser, os.environ["AUTOCAL_REFERENCE_WS"])
                # Ensure the source is registered before requesting shutdown.
                assert registered.wait(timeout=10)
                if fail:
                    raise ValueError("collection failed")
        except ValueError:
            assert fail
        future.result(timeout=10)
    assert "AUTOCAL_REFERENCE_WS" not in os.environ
    manifests = list(directory.glob("hangprinter-*.json"))
    assert len(manifests) == 1
    manifest = json.loads(manifests[0].read_text())
    assert manifest["status"] == "finalized"
    assert not manifest["rejected_messages"]
    assert list(directory.glob("*.rrd"))


def test_headless_recording_alias_uses_managed_headless_recorder(tmp_path, monkeypatch):
    import autocal.headless_sim as module
    monkeypatch.delenv("AUTOCAL_REFERENCE_WS", raising=False)
    seen = []

    class Headless:
        directory = tmp_path
        manifest = {}

        def __init__(self, args, collector_args):
            seen.append(args.extended_reference)

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

    monkeypatch.setattr(module, "HeadlessSimulation", Headless)
    monkeypatch.setattr(ac, "_run_full_auto", lambda *args: 0)
    assert ac.main(["--headless-sim", "--machine-type", "hp4", "--record"]) == 0
    assert seen == [True]
