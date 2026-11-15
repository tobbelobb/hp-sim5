"""Preserve selected replay stages and inventory local-only recordings.

Run after all owned cases close. Large raw files are retained, never deleted.
"""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent
REPO = next(p for p in ROOT.parents if (p/'autocal/pose_fit.py').exists())


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            result.update(block)
    return {'path': str(path.relative_to(REPO)), 'bytes': path.stat().st_size, 'sha256': result.hexdigest()}


def main():
    rules = ['**/events.jsonl', '**/*.rrd', '**/sweeps.stages/*.json', '**/same-data-fitters/fit-*.json']
    cases = []
    for folder in sorted(p for p in ROOT.iterdir() if p.is_dir() and (p/'input.json').exists()):
        manifest_path = folder/'sweeps.headless/manifest.json'
        manifest = json.loads(manifest_path.read_text())
        if not (folder/'summary.json').exists():
            raise ValueError(f'Case has not closed: {folder.name}')
        stages = sorted((folder/'sweeps.stages').glob('fit-*-default.json'))
        if stages:
            rules.append('!'+str(stages[-1].relative_to(ROOT)))
        histories = sorted((folder/'sweeps.stages').glob('history-*.json'))
        if histories:
            with histories[-1].open('rb') as source, histories[-1].with_suffix('.json.gz').open('wb') as target:
                with gzip.GzipFile(fileobj=target, mode='wb', mtime=0) as archive:
                    for block in iter(lambda: source.read(1024*1024), b''):
                        archive.write(block)
        cases.append({'case': folder.name,
            'git_revision': manifest.get('git_revision'), 'source_sha256': manifest.get('source_sha256'),
            'autocal_source_sha256': manifest.get('autocal_source_sha256'),
            'status': manifest.get('status'), 'normal_completion': manifest.get('normal_completion'),
            'input': digest(folder/'input.json'), 'scene': digest(folder/'input-scene.usda'),
            'manifest': digest(manifest_path), 'summary': digest(folder/'summary.json'),
            'dataset': digest(folder/'sweeps.json') if (folder/'sweeps.json').exists() else None})
    (ROOT/'.gitignore').write_text('\n'.join(rules)+'\n')
    files = [p for p in ROOT.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    ignored = subprocess.run(['git', 'check-ignore', '--stdin'], cwd=REPO, text=True, capture_output=True,
        input='\n'.join(str(p.relative_to(REPO)) for p in files)+'\n', check=False)
    if ignored.returncode not in (0,1):
        raise RuntimeError(ignored.stderr)
    local = [digest(REPO/name) for name in ignored.stdout.splitlines()]
    output = {'scope': 'Completed and informative partial production simulations. Nominal dfc0e3e3 trials precede the 5251f749 return-enforcement fix; variants exercise that fix.',
        'storage': 'Selected replay stages, gzip history archives, datasets, logs and summaries are committed. Listed raw RRD/event journals and redundant stages are retained only at these local workspace paths; there is no off-host backup or portable URI. Do not delete the only copies.',
        'cases': cases, 'retained_local_only': local}
    (ROOT/'inventory.json').write_text(json.dumps(output, indent=2)+'\n')
    print(f'{len(cases)} cases, {len(local)} local-only files, {sum(f["bytes"] for f in local):,} raw bytes')


if __name__ == '__main__':
    main()
