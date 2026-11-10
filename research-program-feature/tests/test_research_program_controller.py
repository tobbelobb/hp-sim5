"""Lifecycle checks with fixture clients; no live research or model calls."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import sys
import time

from program.manager import ProgramManager
from test_research_program_manager import ROOT
from test_research_program_review import context, submit, reviewer


def local_manager(c):
    m = ProgramManager(c['store'], c['root'], 'p', 'fixture-manager', c['tmp'] / 'manager', reviews=c['controller'])
    m.controller.recover('p', handoff_attempt_id=c['attempt']['id'], reason='Fixture worker had no world; missing exit manifest retained')
    return m


def ready(m, identity, minutes=.01):
    s = m.store
    s.add_assignment('p', 'route', 'Distinct bounded observation ' + identity, minutes, record_id=identity)
    s.transition('p', identity, 'ready', 'Fixture selection')


def test_launcher_deadline_survives_absent_host_and_second_cancel_during_cleanup(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('deadline_launcher', ROOT / 'scripts/research_agent.py')
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    services = []
    class Service:
        def __init__(self, *args, **kwargs):
            services.append(self)
            self.closed = False
        def start(self):
            time.sleep(5)
        def close(self):
            # The host can send cancellation as the launcher's alarm expires.
            # That second interrupt must not abort this existing cleanup path.
            os.kill(os.getpid(), signal.SIGTERM)
            time.sleep(.01)
            self.closed = True
    monkeypatch.setattr(launcher, 'RuntimeService', Service)
    monkeypatch.setattr(launcher, 'ensure_native_physics', lambda: None)
    monkeypatch.setattr(launcher, 'write_index', lambda *_: None)
    session = tmp_path / 'deadline-session'
    monkeypatch.setattr(sys, 'argv', ['research_agent.py', '--machine', 'hp4', '--codex', '--batch', '--viewer', 'none', '--sidebar-section', 'none',
                                    '--prompt', 'Bound startup independently of tool host', '--session-dir', str(session),
                                    '--wall-deadline', str(time.time() + .3)])
    assert launcher.main() == 130
    assert services[0].closed
    assert json.loads((session / 'exit.json').read_text())['returncode'] == 130
    assert signal.getitimer(signal.ITIMER_REAL)[0] == 0


def test_review_guard_deadline_survives_absent_host_polling(context):
    c = context
    submission = submit(c)
    run = reviewer(c, submission, mode='hold', wall_minutes=.005)
    process = c['controller'].children[run['id']]
    process.wait(timeout=3)
    ended = c['controller'].status('p', run['id'])
    assert ended['details']['state'] == 'timed_out'
    assert Path(ended['details']['directory'], 'wall-expired').is_file()
    assert ended['details']['guard_argv'][0] == sys.executable
    assert ended['details']['argv'][0] == 'codex'
