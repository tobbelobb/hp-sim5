"""Verify or restore the original research artifacts without overwriting changes."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--destination', type=Path)
parser.add_argument('--verify-only', action='store_true')
args = parser.parse_args()
if not args.verify_only and args.destination is None:
    parser.error('provide --destination or --verify-only')
root = Path(__file__).resolve().parent
manifest = json.loads((root / 'raw-artifact-manifest.json').read_text())
count = 0
with tempfile.TemporaryDirectory() as temporary:
    for group in manifest['groups']:
        archive = Path(temporary) / 'joined.tar.gz'
        full = hashlib.sha256()
        with archive.open('wb') as output:
            for part in group['parts']:
                h = hashlib.sha256()
                with (root / part['path']).open('rb') as stream:
                    while chunk := stream.read(1024 * 1024):
                        h.update(chunk)
                        full.update(chunk)
                        output.write(chunk)
                assert h.hexdigest() == part['sha256'], part['path']
        assert full.hexdigest() == group['gzip_sha256'], group['name']
        expected = {entry['path']: entry for entry in group['files']}
        with tarfile.open(archive, 'r:gz') as tar:
            for member in tar:
                assert member.isfile() and member.name in expected, member.name
                record = expected.pop(member.name)
                data = tar.extractfile(member).read()
                assert len(data) == record['bytes'], member.name
                assert hashlib.sha256(data).hexdigest() == record['sha256'], member.name
                if not args.verify_only:
                    destination = args.destination.resolve()
                    target = (destination / member.name).resolve()
                    assert target.is_relative_to(destination), member.name
                    target.parent.mkdir(parents=True, exist_ok=True)
                    if target.exists():
                        assert hashlib.sha256(target.read_bytes()).hexdigest() == record['sha256'], target
                    else:
                        target.write_bytes(data)
                        target.chmod(member.mode & 0o777)
                count += 1
        assert not expected
print(json.dumps({'verified_files': count, 'restored': not args.verify_only}))
