"""Saved-evidence capture + real reviewer command plans; no actual client dispatch.

The producer process is a zero-work fixture, not a managed science investigation.
Source measurements are inspected/copied, not recomputed or newly admitted.
"""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.program import ProgramStore

SESSION = Path(__file__).resolve().parent
SOURCE = ROOT / 'output/research/sessions/2026-10-23-consensus-membership'


def main():
    with tempfile.TemporaryDirectory(prefix='hp-review-plan-') as temporary:
        directory = Path(temporary)
        worker = directory / 'zero-work-producer'
        worker.write_text(f'#!{sys.executable}\n# A lifecycle fixture only.\n')
        worker.chmod(0o755)
        with ProgramStore(directory / 'memory', create=True) as store:
            store.create_program('Can saved evidence close a blanket default-change route?', program_id='plan-check', question_id='question')
            store.add_approach('plan-check', 'question', 'Inspect existing CM-E1 evidence', 'No duplicate science run', record_id='route')
            store.add_assignment('plan-check', 'route', 'Inspect saved evidence only',
                                 {'wall_minutes': 1, 'fresh_fits': 0, 'collections': 0, 'physics_steps': 0}, record_id='assignment')
            store.transition('plan-check', 'assignment', 'ready', 'Zero-work fixture can demonstrate capture and planning')
            dispatcher = store.workers(ROOT, executable=worker)
            attempt = dispatcher.start('plan-check', 'assignment')
            dispatcher.children[attempt['id']].wait(timeout=5)
            attempt = dispatcher.status('plan-check', attempt['id'])
            claim = store.graph().add_claim('plan-check',
                'On eight additional frozen simulated histories, blanket finite-prediction membership did not reliably improve summed anchor error',
                {'evidence_type': 'simulated', 'stage': 'frozen final consensus', 'hardware': False}, record_id='claim')
            selected = []
            for identity, role, filename in [('report', 'report', 'report.md'), ('abstract', 'abstract', 'abstract.md'),
                                               ('measurements', 'evidence', 'results.json')]:
                path = SOURCE / filename
                sha = hashlib.sha256(path.read_bytes()).hexdigest()
                store.add_artifact('plan-check', str(path), sha, 'simulated',
                                   'historical CM-E1 source provenance in selected report/results', record_id=identity+'-artifact',
                                   provenance={'input_identities': ['CM-E1-frozen-histories'], 'observation': 'inspected, not recomputed'})
                selected.append({'id': identity, 'role': role, 'path': str(path), 'artifact_id': identity+'-artifact'})
            controller = store.reviews(ROOT)
            submission = controller.submit('plan-check', attempt['id'], claim['id'],
                base_revision=attempt['details']['source_revision'], config={'comparison': 'saved CM-E1 only'},
                input_identities=['CM-E1-frozen-histories'], selected=selected, producer_checks=[])
            submission_path = store.directory / 'submissions' / submission['id']
            saved = SESSION / 'sealed-example'
            if saved.exists():
                shutil.rmtree(saved)
            shutil.copytree(submission_path, saved)
            plans = [controller.plan('plan-check', submission['id'], backend=backend) for backend in ('codex', 'claude')]
            for plan in plans:
                plan['prompt_sha256'] = hashlib.sha256(plan.pop('prompt').encode()).hexdigest()
            (SESSION / 'command-plans.json').write_text(json.dumps(plans, indent=2)+'\n')
            (SESSION / 'pending-query.json').write_text(json.dumps(store.graph().query('plan-check', 'support', 'claim'), indent=2)+'\n')
            (SESSION / 'program-snapshot.json').write_text(json.dumps(store.snapshot('plan-check'), indent=2)+'\n')
            result = {'boundary': __doc__.strip(), 'submission_id': submission['id'],
                      'submission_sha256': submission['details']['sha256'], 'selected_files': len(selected),
                      'reviewer_commands_planned': [p['argv'][0] for p in plans],
                      'actual_client_processes': 0, 'reviews': 0, 'admissions': 0,
                      'source_head': attempt['details']['source_revision'], 'source_dirty': attempt['details']['source_dirty']}
            (SESSION / 'exercise.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
