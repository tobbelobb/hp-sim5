"""Human-readable session names and a catalog built from short research abstracts."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import unicodedata
from urllib.parse import quote
import uuid


def session_name(topic):
    ascii_topic = unicodedata.normalize('NFKD', topic).encode('ascii', 'ignore').decode()
    slug = re.sub(r'[^a-z0-9]+', '-', ascii_topic.lower()).strip('-')[:64].rstrip('-') or 'research'
    date = datetime.now(timezone.utc).strftime('%Y-%m-%d')
    return f'{date}-{slug}-{uuid.uuid4().hex[:8]}'


def initial_abstract(topic):
    title = ' '.join(topic.split())[:120]
    return (f'# {title}\n\n'
            'Status: not yet summarized.\n\n'
            '## Abstract\n\n'
            'The session has started. No research result is available yet.\n\n'
            '## Topics\n\n'
            f'{title}\n\n'
            '## Tested hypotheses\n\n'
            '| Hypothesis | Experiment IDs | Outcome | Evidence |\n'
            '| --- | --- | --- | --- |\n'
            '| None recorded yet | None | Not tested | None |\n\n'
            '## Decision handoff\n\n'
            'No research decision yet. Scope, review and the next observation '
            'have not been recorded.\n\n'
            '## References\n\n'
            '- [Original task](prompt.txt)\n'
            '- [Research notebook](research.md)\n')


def catalog_entries(root):
    root = Path(root)
    candidates = [p for parent in (root, root / 'sessions') if parent.is_dir()
                  for p in parent.iterdir() if p.is_dir()]
    # Prefer a descriptive alias over its old hash, and include each session once.
    candidates.sort(key=lambda p: (bool(re.fullmatch(r'[0-9a-f]{32}', p.name)), p.name))
    seen = set()
    for directory in candidates:
        resolved = directory.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if not any((directory / name).is_file() for name in
                   ('abstract.md', 'report.md', 'research.md', 'prompt.txt', 'launch.json', 'exit.json')):
            continue
        abstract = directory / 'abstract.md'
        summary = (abstract.read_text() if abstract.is_file() else
                   f'# {directory.name}\n\nStatus: abstract missing. '
                   'Consult the original artifacts; tested hypotheses have not been cataloged.\n')
        yield directory, summary


def render_index(root, search=''):
    root = Path(root)
    parts = ['# Research session catalog\n\n'
             'Short abstracts and tested hypotheses from local sessions. '
             'Missing or pending summaries are not evidence of completed experiments.\n\n'
             'Regenerate with `.venv/bin/python scripts/research_index.py`.\n']
    for directory, summary in catalog_entries(root):
        if search.casefold() not in (directory.name + '\n' + summary).casefold():
            continue
        relative = directory.relative_to(root).as_posix()
        parts.append(f'## [{directory.name}]({quote(relative)}/)\n')
        links = [f'[{name}]({quote(relative)}/{name})' for name in
                 ('abstract.md', 'report.md', 'research.md') if (directory / name).is_file()]
        if links:
            parts.append(' · '.join(links) + '\n')
        # Anchor relative evidence links at their session while embedding its abstract.
        summary = re.sub(r'\]\(([^)]+)\)', lambda m: '](' + (
            quote(relative) + '/' + m[1] if not m[1].startswith(('/', '#')) and ':' not in m[1]
            else m[1]) + ')', summary)
        summary = re.sub(r'^(#+) ', r'##\1 ', summary, flags=re.MULTILINE)
        parts.append(summary)
    return '\n'.join(parts).rstrip() + '\n'


def write_index(root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    content = render_index(root)
    # Concurrent launchers must not leave a partly written catalog.
    with tempfile.NamedTemporaryFile(mode='w', dir=root, prefix='.index-', delete=False) as temporary:
        temporary.write(content)
    os.replace(temporary.name, root / 'index.md')


def saved_autocal_status(path):
    """Read a historical observation without consulting or signalling any PID."""
    path = Path(path)
    result = dict(record=str(path.resolve()), saved_status=None, observation='unavailable',
                  live_status='unknown', scientific_success='not_assessed', diagnostics=[])
    try:
        job = json.loads(path.read_text())
        if not isinstance(job, dict):
            raise ValueError('expected a job object')
    except (OSError, ValueError) as error:
        result['diagnostics'].append(f'{path}: {error}')
        return result
    status = job.get('status')
    result['saved_status'] = status
    if status in ('accepted', 'failed', 'cancelled', 'budget_exhausted'):
        result['observation'] = 'terminal'
    elif status in ('running', 'cancelling'):
        result['observation'] = 'historical_nonterminal'
    else:
        result['observation'] = 'unknown_status'
        result['diagnostics'].append(f'{path}: unrecognized saved autocal status {status!r}')
    for key in ('job_id', 'kind', 'started_unix_s', 'ended_unix_s', 'exit_code', 'stop_reason',
                'settings', 'progress', 'error', 'cleanup_error', 'supervisor_source_sha256'):
        if key in job:
            result[key] = job[key]
    return result


def _handoff_path(value):
    if not isinstance(value, str):
        raise ValueError('Handoff file paths must be strings')
    path = Path(value)
    if not value or path.is_absolute() or '..' in path.parts or path == Path('.'):
        raise ValueError(f'Handoff paths must be relative files without ..: {value!r}')
    if path == Path('handoff.json'):
        raise ValueError('handoff.json is reserved for the export manifest')
    return path


def _sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def export_handoff(session, selection, destination):
    """Copy an explicit evidence selection; preserve missing files in the manifest."""
    session, destination = Path(session).resolve(), Path(destination).resolve()
    if not isinstance(selection, dict):
        raise ValueError('Selection must be a JSON object')
    for key in ('claim', 'scope', 'decision', 'source_revision'):
        if not isinstance(selection.get(key), str) or not selection[key].strip():
            raise ValueError(f'Selection requires a nonempty {key}')
    if not isinstance(selection.get('files', []), list):
        raise ValueError('Selected files must be a list')
    if not isinstance(selection.get('limitations', []), list) or any(
            not isinstance(item, str) for item in selection.get('limitations', [])):
        raise ValueError('Limitations must be a list of strings')
    files = [dict(path=name, role=role) for name, role in
             [('abstract.md', 'abstract'), ('report.md', 'report'), ('research.md', 'notebook')]]
    files += selection.get('files', [])
    paths = set()
    for entry in files:
        if not isinstance(entry, dict):
            raise ValueError('Each selected file must be an object with a relative path')
        path = _handoff_path(entry['path'])
        if path in paths or any(parent in paths for parent in path.parents) or any(path in p.parents for p in paths):
            raise ValueError(f'Duplicate or overlapping handoff file: {path}')
        paths.add(path)
        if not isinstance(entry.get('source', entry['path']), str) or not entry.get('source', entry['path']):
            raise ValueError(f'Invalid source for {path}')
        if not isinstance(entry.get('role', 'evidence'), str):
            raise ValueError(f'Invalid role for {path}')
        if 'sha256' in entry and (not isinstance(entry['sha256'], str) or not re.fullmatch('[0-9a-f]{64}', entry['sha256'])):
            raise ValueError(f'Invalid expected SHA-256 for {path}')
    # Refuse to overwrite an existing bundle or merge it with stale evidence.
    destination.mkdir(parents=True, exist_ok=False)
    manifest = {key: selection[key] for key in ('claim', 'scope', 'decision', 'source_revision')}
    manifest.update(format_version=1, source_session=str(session),
                    limitations=selection.get('limitations', []), files=[])
    for entry in files:
        source = session / entry.get('source', entry['path'])
        target = destination / entry['path']
        record = dict(path=entry['path'], source=str(source), role=entry.get('role', 'evidence'),
                      expected_sha256=entry.get('sha256'), sha256=None, status='missing')
        manifest['files'].append(record)
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            record.update(sha256=_sha256(target), status='copied')
            if record['expected_sha256'] and record['sha256'] != record['expected_sha256']:
                record.update(status='hash_mismatch', diagnostic='Source differs from selected SHA-256')
        except OSError as error:
            record.update(status='unavailable', diagnostic=str(error))
    (destination / 'handoff.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return read_handoff(destination)


def read_handoff(directory):
    """Locate portable records and verify selected bytes without the source tree."""
    directory = Path(directory).resolve()
    result = dict(directory=str(directory), integrity='partial', diagnostics=[], files=[], jobs=[])
    try:
        manifest = json.loads((directory / 'handoff.json').read_text())
        if not isinstance(manifest, dict) or manifest.get('format_version') != 1:
            raise ValueError('expected a version-1 handoff manifest')
        for key in ('claim', 'scope', 'decision', 'source_revision', 'source_session', 'limitations'):
            result[key] = manifest[key]
        if not isinstance(manifest['files'], list) or not manifest['files']:
            raise ValueError('expected a nonempty files list')
        for entry in manifest['files']:
            if not isinstance(entry, dict):
                raise ValueError('expected a file object')
            path = _handoff_path(entry['path'])
            target = directory / path
            record = dict(entry, current_path=str(target))
            result['files'].append(record)
            if entry['status'] != 'copied':
                result['diagnostics'].append(f'{path}: {entry.get("diagnostic", entry["status"])}')
            try:
                if not target.resolve().is_relative_to(directory):
                    raise ValueError('file resolves outside the handoff')
                actual = _sha256(target)
                record['integrity'] = 'ok' if actual == entry['sha256'] else 'hash_mismatch'
                if record['integrity'] != 'ok':
                    result['diagnostics'].append(f'{path}: SHA-256 mismatch; evidence changed after export')
            except (OSError, ValueError) as error:
                record['integrity'] = 'unavailable'
                result['diagnostics'].append(f'{path}: {error}')
            if entry['role'] == 'job' and record['integrity'] == 'ok':
                job = saved_autocal_status(target)
                result['jobs'].append(job)
                result['diagnostics'].extend(job['diagnostics'])
    except (OSError, ValueError, KeyError, TypeError) as error:
        result['diagnostics'].append(f'Invalid handoff manifest: {error}')
    if not result['diagnostics']:
        result['integrity'] = 'verified_selected_files'
    return result
