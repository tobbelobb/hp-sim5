"""Exercise service lifecycle against a real terminal without launching Codex."""
import contextlib
import importlib.util
import os
from pathlib import Path
import pty
import select
import shutil
import subprocess
import termios
import time
import tty

import pytest

from hp_sim5_research.services import RuntimeService, stop_process

ROOT = Path(__file__).resolve().parents[2]


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('service', ['viewer', 'runtime', 'collector'])
def test_service_shutdown_preserves_codex_terminal(tmp_path, monkeypatch, service):
    node = shutil.which('node')
    assert node, 'The research collector requires Node.js'
    idle = 'console.log("ready"); setInterval(() => {}, 1000)'
    wrapper = f'import os\nos.execv({node!r}, [{node!r}, "-e", {idle!r}])\n'
    master, terminal = pty.openpty()
    popen = subprocess.Popen
    processes = []

    def launch(command, **options):
        # Simulate the launcher's inherited Codex terminal, unless explicitly isolated.
        options.setdefault('stdin', terminal)
        child = popen(command, **options)
        processes.append(child)
        return child

    monkeypatch.setattr(subprocess, 'Popen', launch)
    try:
        if service == 'viewer':
            launcher = load_script('research_agent')
            monkeypatch.setattr(launcher, 'ROOT', tmp_path)
            monkeypatch.setattr(launcher, 'free_port', lambda: 12345)
            executable = tmp_path / '.venv/bin/rerun'
            executable.parent.mkdir(parents=True)
            executable.write_text(f'#!{shutil.which("python3")}\n' + wrapper)
            executable.chmod(0o755)
            monkeypatch.setattr(launcher.socket, 'create_connection', lambda *a, **k: contextlib.nullcontext())
            process, _ = launcher.start_viewer('headless', tmp_path)
            log = tmp_path / 'viewer.log'
        elif service == 'runtime':
            import hp_sim5_research.services as services
            monkeypatch.setattr(services, 'free_port', lambda: 12345)
            script = tmp_path / 'scripts/research_runtime.py'
            script.parent.mkdir()
            script.write_text(wrapper)
            monkeypatch.setattr(services, 'wait_ready', lambda *a, **k: None)
            runtime = RuntimeService(tmp_path, tmp_path / 'native').start()
            process, log = runtime.process, runtime.directory / 'runtime.log'
        else:
            module = load_script('research_runtime')
            runtime = module.Runtime.__new__(module.Runtime)
            runtime.directory, runtime.processes = tmp_path, {}
            process, log = runtime.spawn('collector', [node, '-e', idle])

        deadline = time.monotonic() + 5
        while 'ready' not in log.read_text():
            assert process.poll() is None, log.read_text()
            assert time.monotonic() < deadline, 'Node did not start'
            time.sleep(.01)

        # Services start first; Codex subsequently enables raw input.
        tty.setraw(terminal)
        before = termios.tcgetattr(terminal)
        detached = os.getsid(process.pid) == process.pid
        stop_process(process)
        assert termios.tcgetattr(terminal) == before, 'Service shutdown restored Codex to cooked input'
        assert detached, 'Terminal Ctrl-C must not signal background services'

        # Mouse reports remain intact and Esc arrives without needing Enter.
        keys = b'\x1b[<35;55;62M\x1b'
        os.write(master, keys)
        assert select.select([terminal], [], [], 1)[0], 'Esc is stuck in line-buffered input'
        assert os.read(terminal, len(keys)) == keys
        assert not select.select([master], [], [], .05)[0], 'Mouse bytes were echoed into the prompt'
    finally:
        for process in processes:
            stop_process(process)
        os.close(master)
        os.close(terminal)
