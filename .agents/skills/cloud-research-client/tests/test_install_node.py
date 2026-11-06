import hashlib
import json
import os
from pathlib import Path
import subprocess
import tarfile


ROOT = Path(__file__).resolve().parents[4]
INSTALLER = ROOT / '.agents/skills/cloud-research-client/scripts/install-node.sh'


def test_installer_selects_latest_lts_and_updates_existing_install(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    commands = tmp_path / 'bin'
    commands.mkdir()
    curl = commands / 'curl'
    curl.write_text('''#!/bin/bash
set -eu
if [[ "$2" == *index.json ]]; then cp "$FAKE_NODE_DIST/index.json" "$4"; exit; fi
url="$2"
if [[ "$1" == -fsSLO ]]; then cp "$FAKE_NODE_DIST/${url##*/}" .; else cp "$FAKE_NODE_DIST/SHASUMS256.txt" "$4"; fi
''')
    curl.chmod(0o755)
    env = {**os.environ, 'PATH': f'{commands}:{os.environ["PATH"]}',
           'FAKE_NODE_DIST': str(source), 'HP_SIM5_CLOUD_TOOLS': str(tmp_path / 'tools')}

    def release(version):
        directory = f'node-v{version}-linux-x64'
        node = source / directory / 'bin/node'
        node.parent.mkdir(parents=True)
        node.write_text('#!/bin/sh\n')
        node.chmod(0o755)
        archive = source / f'{directory}.tar.xz'
        with tarfile.open(archive, 'w:xz') as tar:
            tar.add(source / directory, arcname=directory)
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        (source / 'SHASUMS256.txt').write_text(f'{digest}  {archive.name}\n')
        (source / 'index.json').write_text(json.dumps([
            {'version': 'v99.0.0', 'lts': False},
            {'version': f'v{version}', 'lts': 'Test'},
            {'version': 'v1.0.0', 'lts': 'Old'},
        ]))
        subprocess.run(['bash', str(INSTALLER)], env=env, check=True, capture_output=True)
        assert os.readlink(tmp_path / 'tools/node/current') == directory

    release('2.1.0')
    release('3.0.0')
