"""Preserve selected exact fixture receipts and Git objects after the passing run."""
import hashlib
import gzip
import json
from pathlib import Path
import shutil
import sqlite3
import subprocess

ROOT = Path(__file__).resolve().parents[4]
SESSION = Path(__file__).resolve().parent
FIXTURES = Path('/tmp/pytest-of-agent/pytest-current').resolve()
CASES = {
    'compatible': 'test_compatible_binary_candida0',
    'failing-combination': 'test_individual_passing_candid0',
    'conflict': 'test_rejected_preparations_pre0',
    'stale-base': 'test_rejected_preparations_pre1',
    'mutated-source': 'test_rejected_preparations_pre2',
    'missing-physics-gate': 'test_required_physics_gate_is_0',
    'runner-loss': 'test_runner_loss_cannot_releas0',
    'detached-child': 'test_detected_detached_child_h0',
    'file-host': 'test_actual_file_host_returns_0',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    summary = {}
    for name, fixture in CASES.items():
        source = FIXTURES / fixture
        target = SESSION / 'fixture-evidence' / name
        target.mkdir(parents=True, exist_ok=True)
        with sqlite3.connect((source / 'memory/programs.sqlite3').as_uri() + '?mode=ro', uri=True) as connection:
            connection.row_factory = sqlite3.Row
            records = []
            for row in connection.execute('SELECT * FROM records ORDER BY created_at,id'):
                r = dict(row)
                r['details'] = json.loads(r.pop('details_json'))
                records.append(r)
            (target / 'records.json').write_text(json.dumps(records, indent=2) + '\n')
        for directory in ('integrations', 'submissions'):
            for path in (source / 'memory' / directory).glob('*/*'):
                if path.is_file():
                    relative = path.relative_to(source / 'memory')
                    copied = target / relative
                    copied.parent.mkdir(parents=True, exist_ok=True)
                    if path.suffix == '.diff':
                        copied.with_name(copied.name + '.gz').write_bytes(gzip.compress(path.read_bytes(), mtime=0))
                    else:
                        shutil.copyfile(path, copied)
        bundle = target / 'fixture.bundle'
        subprocess.run(['git', '-C', str(source / 'repo'), 'bundle', 'create', str(bundle), '--all'], check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        summary[name] = {'source': str(source), 'git_bundle': str(bundle.relative_to(SESSION)),
                         'preparations': [{'id': r['id'], 'outcome': r['details']['manifest']['outcome'],
                                           'sha256': r['details']['sha256']} for r in records if r['kind'] == 'integration_preparation'],
                         'checks': [{'id': r['id'], 'outcome': r['details']['manifest']['outcome'],
                                     'sha256': r['details']['sha256']} for r in records if r['kind'] == 'integration_check'],
                         'admissions': [{'id': r['id'], 'finding': r['details']['finding'],
                                         'knowledge_state': r['details']['knowledge_state']} for r in records if r['kind'] == 'admission']}
    (SESSION / 'fixture-evidence.json').write_text(json.dumps(summary, indent=2) + '\n')
    hashes = {str(p.relative_to(SESSION)): sha(p) for p in sorted((SESSION / 'fixture-evidence').rglob('*')) if p.is_file()}
    (SESSION / 'fixture-hashes.json').write_text(json.dumps(hashes, indent=2) + '\n')


if __name__ == '__main__':
    main()
