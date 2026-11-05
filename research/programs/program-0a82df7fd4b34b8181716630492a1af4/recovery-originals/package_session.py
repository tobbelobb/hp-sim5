"""Archive this completed session, excluding credentials and live control files."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil
import tarfile

SESSION = Path('/workspace/hp-sim5/output/research/sessions/attempt-939c073f49004eb6949910600200be51')
OUTPUT = Path(__file__).resolve().parent
PRIVATE = {'connection.json', 'tool-connection.json', 'attachment.json', 'attachment.toml',
           'connection.lease', 'auth.json', 'launch.json', 'session.json'}
SECRET = re.compile(rb'(?i)(?:Bearer\s+[A-Za-z0-9_\-.]{20,}|sk-proj-[A-Za-z0-9_-]+|'
                    rb'"(?:api_key|access_token|auth_token|bearer_token|authorization|password)"\s*:\s*"[^"\r\n]+")')

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        while chunk := stream.read(1024 * 1024):
            h.update(chunk)
    return h.hexdigest()

groups = {}
omitted = []
for path in sorted(SESSION.rglob('*')):
    rel = path.relative_to(SESSION)
    if not path.is_file():
        continue
    if path.is_symlink() or path.name in PRIVATE or any(
            p.startswith('.env') or p in {'.git', '.venv', 'node_modules'} for p in rel.parts):
        omitted.append({'path': str(rel), 'reason': 'private control/dependency file or symlink'})
        continue
    if path.suffix.lower() in {'.json', '.jsonl', '.md', '.txt', '.log', '.py', '.mjs', '.js', '.toml'}:
        matched = False
        with path.open('rb') as stream:
            for line in stream:
                if SECRET.search(line):
                    matched = True
                    break
        if matched:
            omitted.append({'path': str(rel), 'reason': 'credential-like text; excluded without printing values'})
            continue
    if rel.parts[0] in {'runs', 'native'} and len(rel.parts) > 1:
        group = '--'.join(rel.parts[:2])
    elif len(rel.parts) > 1:
        group = rel.parts[0]
    else:
        group = 'session-record'
    groups.setdefault(group, []).append(path)

manifest = {'session': str(SESSION), 'source_revision': '5bb8eb955fa680ddc631b6fb10f988ec3efb497a',
            'omitted': omitted, 'groups': [], 'verified_by_readback': False}
for name, paths in sorted(groups.items()):
    archive = OUTPUT / (name + '.tar.gz')
    records = []
    with archive.open('wb') as raw, gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0, compresslevel=3) as zipped:
        with tarfile.open(fileobj=zipped, mode='w') as tar:
            for path in paths:
                rel = str(path.relative_to(SESSION))
                records.append({'path': rel, 'bytes': path.stat().st_size, 'sha256': digest(path)})
                info = tar.gettarinfo(str(path), arcname=rel)
                info.mtime = info.uid = info.gid = 0
                info.uname = info.gname = ''
                with path.open('rb') as stream:
                    tar.addfile(info, stream)
    expected = {r['path']: r for r in records}
    with tarfile.open(archive, 'r:gz') as tar:
        for member in tar:
            h = hashlib.sha256()
            with tar.extractfile(member) as stream:
                while chunk := stream.read(1024 * 1024):
                    h.update(chunk)
            assert h.hexdigest() == expected.pop(member.name)['sha256'], member.name
    assert not expected
    record = {'name': name, 'uncompressed_bytes': sum(r['bytes'] for r in records),
              'gzip_sha256': digest(archive), 'files': records, 'parts': []}
    # Keep every Git object below GitHub's individual-file size limit.
    if archive.stat().st_size > 90 * 1024 * 1024:
        with archive.open('rb') as stream:
            part = 0
            while chunk := stream.read(48 * 1024 * 1024):
                target = OUTPUT / (archive.name + f'.part{part:03d}')
                target.write_bytes(chunk)
                record['parts'].append({'path': target.name, 'bytes': len(chunk), 'sha256': digest(target)})
                part += 1
        archive.unlink()
    else:
        record['parts'].append({'path': archive.name, 'bytes': archive.stat().st_size, 'sha256': digest(archive)})
    manifest['groups'].append(record)
    print(json.dumps({'group': name, 'files': len(records), 'compressed_bytes': sum(p['bytes'] for p in record['parts'])}), flush=True)

manifest['verified_by_readback'] = True
(OUTPUT / 'raw-artifact-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
review = Path('/workspace/hp-sim5/.research-programs/reviews/review-run-a87f2b7c6e6a4b5c889e45cf866cc22f/final-message.md')
shutil.copyfile(review, OUTPUT / 'original-review-response.json')
print(json.dumps({'groups': len(groups), 'files': sum(len(g['files']) for g in manifest['groups']),
                  'uncompressed_bytes': sum(g['uncompressed_bytes'] for g in manifest['groups']),
                  'compressed_bytes': sum(p['bytes'] for g in manifest['groups'] for p in g['parts']),
                  'omissions': len(omitted), 'verified_by_readback': True}), flush=True)
