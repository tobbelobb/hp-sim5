"""Save small research results in Git, separately from live controller state."""
from pathlib import Path
import shutil
import subprocess

from .program import ProgramError, _json, render_markdown

MAX_FILE = 1024 * 1024
MAX_TOTAL = 8 * MAX_FILE
PRIVATE = {'.env', 'auth.json', 'credentials.json', 'connection.json', 'control-token.json',
           'attachment.json', 'attachment.toml', 'instructions.md', 'prompt.txt', 'events.jsonl'}


def save_program(store, root, program_id, *, files=(), commit=False):
    """Copy notes, selected evidence and memory; commit only this result directory."""
    root = Path(root).resolve()
    if type(commit) is not bool or not isinstance(files, (list, tuple)) or not all(isinstance(p, str) for p in files):
        raise ProgramError('save requires a boolean commit and a list of file paths')
    snapshot = store.snapshot(program_id)
    records = snapshot['records']
    for r in records:
        if r['kind'] in ('attempt', 'review_run', 'integration_job') and r['details'].get('state') in (
                'starting', 'running', 'cancelling', 'ownership_lost'):
            raise ProgramError('Stop owned work and finish cleanup before saving research results')
    destination = root / 'research/programs' / program_id
    selected, skipped = {}, []
    size = 0

    def select(source, relative, required=False):
        nonlocal size
        source = Path(source)
        if not source.is_absolute():
            source = root / source
        reason = None
        if source.name in PRIVATE or source.name.startswith('.env') or '.git' in source.parts or source.is_symlink():
            reason = 'private/runtime file or symlink'
        elif not source.resolve().is_relative_to(root) or not source.is_file():
            reason = 'missing or outside this checkout'
        elif source.stat().st_size > MAX_FILE or size + source.stat().st_size > MAX_TOTAL:
            reason = 'exceeds small-file budget'
        if reason:
            if required:
                raise ProgramError(f'Cannot save selected file {source}: {reason}')
            skipped.append({'source': str(source), 'reason': reason})
        else:
            selected[relative] = source
            size += source.stat().st_size

    for r in records:
        if r['kind'] == 'attempt':
            for name in ('research.md', 'report.md', 'abstract.md', 'final-message.md'):
                select(Path(r['details']['session_dir']) / name, f'workers/{r["id"]}/{name}')
        elif r['kind'] == 'artifact':
            path = Path(r['details']['reference'])
            select(path, f'evidence/{r["id"]}/{path.name}')
        elif r['kind'] == 'submission':
            directory = store.directory / 'submissions' / r['id']
            for name in ('manifest.json', 'candidate.diff', *(a['id'] for a in r['details']['manifest']['assets'])):
                select(directory / name, f'submissions/{r["id"]}/{name}')
    for source in files:
        path = Path(source)
        absolute = (root / path).resolve()
        if not absolute.is_relative_to(root):
            raise ProgramError('Selected research files must be inside this checkout')
        select(path, 'selected/' + str(absolute.relative_to(root)), required=True)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / 'program.json').write_text(_json(snapshot) + '\n')
    (destination / 'report.md').write_text(render_markdown(snapshot))
    for relative, source in selected.items():
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
    (destination / 'files.json').write_text(_json({'copied': sorted(selected), 'skipped': skipped}) + '\n')
    revision = None
    if commit:
        relative = str(destination.relative_to(root))
        def git(*args, allowed=(0,)):
            result = subprocess.run(['git', '-C', str(root), *args], capture_output=True, text=True)
            if result.returncode not in allowed:
                detail = result.stderr.strip() or result.stdout.strip() or 'no diagnostic output'
                raise ProgramError(f'git {args[0]} failed ({result.returncode}): {detail}')
            return result
        git('add', '--force', '--', relative)
        changed = git('diff', '--cached', '--quiet', '--', relative, allowed=(0, 1))
        if changed.returncode:
            git('commit', '--only', '-m', 'Save research results: ' + snapshot['program']['goal'].splitlines()[0][:100], '--', relative)
        revision = git('rev-parse', 'HEAD').stdout.strip()
    return {'directory': str(destination), 'copied': len(selected), 'skipped': skipped, 'commit': revision}
