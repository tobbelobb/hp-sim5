"""Fixed, checksummed review inputs; no model calls or automatic merge."""
import hashlib
from pathlib import Path
import subprocess
import uuid

from hp_sim5_research.workspace import read_json, role, write_json


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def seal(root, program, selection_path, *, dry_run=False):
    selection = read_json(selection_path)
    if not isinstance(selection, dict):
        raise ValueError('Review selection must be an object')
    if not isinstance(selection.get('claim'), str) or not selection['claim'].strip():
        raise ValueError('Review selection needs a claim')
    files = selection.get('files')
    if not isinstance(files, list) or not files or len(files) > 64:
        raise ValueError('Select 1–64 report/evidence files; reference large recordings by checksum')
    candidate = git(root, 'rev-parse', '--verify', selection.get('candidate_revision', 'HEAD') + '^{commit}')
    base = git(root, 'rev-parse', '--verify', selection.get('base_revision', candidate) + '^{commit}')
    sources = []
    total = 0
    for value in files:
        relative = Path(value)
        if relative.is_absolute() or '..' in relative.parts or relative == Path('.'):
            raise ValueError('Evidence paths must be relative to the repository')
        if any(part.startswith('.env') or part in ('.git', '.research-state') for part in relative.parts) or relative.name in (
                'connection.json', 'attachment.json', 'attachment.toml', 'launch.json'):
            raise ValueError('Credentials and private launcher state are not evidence')
        entry = git(root, 'ls-tree', candidate, '--', relative.as_posix())
        if not entry or entry.split()[0] not in ('100644', '100755'):
            raise ValueError(f'Evidence must be a regular file committed in the candidate: {value}')
        blob = git(root, 'rev-parse', candidate + ':' + relative.as_posix())
        total += int(git(root, 'cat-file', '-s', blob))
        if total > 32 * 1024 * 1024:
            raise ValueError('Selected review inputs exceed 32 MiB; use portable recording references')
        sources.append((relative, blob))
    directory = program / 'reviews' / ('review-' + uuid.uuid4().hex[:12])
    manifest = {'version': 1, 'claim': selection['claim'], 'scope': selection.get('scope', ''),
                'limitations': selection.get('limitations', []), 'candidate_revision': candidate,
                'candidate_tree': git(root, 'rev-parse', candidate + '^{tree}'), 'base_revision': base, 'files': []}
    patch = subprocess.check_output(['git', '-C', str(root), 'diff', '--binary', base, candidate, '--'])
    for index, (relative, blob) in enumerate(sources):
        raw = subprocess.check_output(['git', '-C', str(root), 'cat-file', 'blob', blob])
        target = f'files/{index:02d}-{relative.name}'
        manifest['files'].append({'path': target, 'source': relative.as_posix(), 'blob': blob, 'sha256': hashlib.sha256(raw).hexdigest()})
        if not dry_run:
            (directory / 'files').mkdir(parents=True, exist_ok=True)
            (directory / target).write_bytes(raw)
    manifest['patch_sha256'] = hashlib.sha256(patch).hexdigest()
    if not dry_run:
        (directory / 'candidate.diff').write_bytes(patch)
        write_json(directory / 'manifest.json', manifest)
        (directory / 'manifest.sha256').write_text(digest(directory / 'manifest.json') + '\n')
    return {'directory': str(directory), 'instructions': role(root, 'reviewer'),
            'task': f'Review the fixed submission at {directory}. Verify its hashes using the program CLI. '
                    'The manifest identifies a committed candidate; uncommitted work is not included. '
                    'Inspect that revision and the selected files. Write review.md here with your assessment, '
                    'checks performed, limitations, and any decisive missing test.', 'manifest': manifest}


def verify(directory):
    directory = Path(directory).resolve()
    manifest_path = directory / 'manifest.json'
    if digest(manifest_path) != (directory / 'manifest.sha256').read_text().strip():
        raise ValueError('Review manifest changed')
    manifest = read_json(manifest_path)
    for item in manifest['files']:
        path = (directory / item['path']).resolve()
        if not path.is_relative_to(directory) or digest(path) != item['sha256']:
            raise ValueError('Selected review evidence changed: ' + item['path'])
    if digest(directory / 'candidate.diff') != manifest['patch_sha256']:
        raise ValueError('Candidate diff changed')
    return {'verified': True, 'directory': str(directory), 'candidate_revision': manifest['candidate_revision'],
            'candidate_tree': manifest['candidate_tree'], 'scientific_validity': 'not_assessed'}
