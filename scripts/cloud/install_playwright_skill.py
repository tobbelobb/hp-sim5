#!/usr/bin/env python3
"""Restore only the Microsoft Playwright skill; preserve other project skills."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import urllib.request


def install(destination):
    manifest = json.loads(Path(__file__).with_name('playwright-skill.json').read_text())
    expected = manifest['files']

    def matches(directory):
        if directory.is_symlink() or any(p.is_symlink() for p in directory.rglob('*')):
            return False
        actual = {
            p.relative_to(directory).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.rglob('*') if p.is_file() and '__pycache__' not in p.parts
        }
        return actual == expected

    if destination.exists() or destination.is_symlink():
        if not matches(destination):
            raise SystemExit(f'Preserving modified skill; reconcile it manually: {destination}')
        print(f'{destination.name}: verified')
        return

    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.playwright-download-', dir=destination.parent) as temp:
        stage = Path(temp) / 'playwright-cli'
        stage.mkdir()
        for relative, checksum in expected.items():
            target = stage / relative
            if not target.resolve().is_relative_to(stage.resolve()):
                raise SystemExit(f'Invalid skill path: {relative}')
            url = (f"https://raw.githubusercontent.com/{manifest['source']}/"
                   f"{manifest['commit']}/{manifest['skillPath']}/{relative}")
            with urllib.request.urlopen(url, timeout=60) as response:
                content = response.read()
            if hashlib.sha256(content).hexdigest() != checksum:
                raise SystemExit(f'Upstream checksum mismatch: {relative}')
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
        if not matches(stage):
            raise SystemExit('Playwright skill verification failed')
        stage.rename(destination)
    print(f'{destination.name}: installed and verified')


if __name__ == '__main__':
    default = Path(__file__).resolve().parents[2] / '.agents/skills/playwright-cli'
    install(Path(sys.argv[1]) if len(sys.argv) > 1 else default)
