#!/usr/bin/env python3
"""Negative control: the only export/import path (research handoff) round trips.

Recordings themselves have no export/import or resume-from-recording API: `.rrd`
files are written by the recorder and can only be read by a generic Rerun reader.
The repository's export/import feature is `archive.export_handoff` /
`read_handoff`; it must copy the selected bytes and verify them.

Exit 0: verified round trip and tamper detection both work. Exit 1 otherwise.
"""
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parents[3]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.archive import export_handoff, read_handoff


def main():
    scratch = Path(tempfile.mkdtemp(dir='/tmp/opencode'))
    session, destination = scratch / 'session', scratch / 'handoff'
    session.mkdir()
    (session / 'abstract.md').write_text('# Abstract\n\nOne sentence.\n')
    (session / 'report.md').write_text('# Report\n\nEvidence.\n')
    (session / 'research.md').write_text('# Notebook\n\nNotes.\n')
    (session / 'trace.json').write_text(json.dumps({'step': [0, 1, 2]}))
    selection = {'claim': 'test claim', 'scope': 'test scope', 'decision': 'test decision',
                 'source_revision': 'test-revision', 'limitations': [],
                 'files': [{'path': 'trace.json', 'role': 'evidence'}]}
    exported = export_handoff(session, selection, destination)
    imported = read_handoff(destination)
    copied = {record['path']: record['sha256'] for record in imported['files']}
    source = {path.name: __import__('hashlib').sha256(path.read_bytes()).hexdigest()
              for path in session.iterdir()}
    (destination / 'report.md').write_text('# Report\n\nTampered.\n')
    tampered = read_handoff(destination)

    checks = {
        'export_integrity': exported['integrity'],
        'import_integrity': imported['integrity'],
        'files_copied': sorted(copied),
        'hashes_match_sources': all(copied[name] == source[name] for name in copied),
        'tampered_integrity': tampered['integrity'],
        'tamper_detected': any('SHA-256 mismatch' in item for item in tampered['diagnostics']),
    }
    print(json.dumps(checks, indent=2))
    failures = [name for name, ok in (
        ('round trip not verified', imported['integrity'] != 'verified_selected_files'),
        ('hash mismatch on a clean copy', not checks['hashes_match_sources']),
        ('tamper not detected', not checks['tamper_detected']),
    ) if ok]
    if failures:
        print('FAIL (handoff export/import): ' + '; '.join(failures), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
