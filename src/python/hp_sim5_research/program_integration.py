"""Frozen reviewed patches in explicit-base worktrees, with polled exact-revision checks.

This cooperative same-UID controller never merges, pushes, resets or stashes the
human checkout. Its durable lease includes startup, waiting for checks and unknown
ownership. Check output and failed worktrees remain available for inspection.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from .program import ProgramError, _identity, _json, _now, _object, _text
from .program_graph import identities
from .program_review import commit, digest, git, json_digest, physics_changed
from .program_workers import process_identity, process_live

CONTRACT = 'hp-sim5-combined-integration-v1'
LEASED = ('preparing', 'prepared', 'checking', 'ownership_lost')
CLEANUP_BOUNDARY = ('Observed cleanup covers the owned Linux process group. Supported checks keep descendants '
                    'in that group and finish external services before exit. Detected detached children hold '
                    'the lease; external or escaped services after abrupt runner loss remain unobserved.')


def group_members(identity):
    """Observe our Linux process group, including descendants after leader loss."""
    if not isinstance(identity, dict) or identity.get('group') != identity.get('pid'):
        return None
    current = process_identity(os.getpid())
    if not current or any(identity.get(k) != current.get(k) for k in ('boot_id', 'uid', 'host')):
        return None
    leader = process_identity(identity['pid'])
    if leader is not None and leader != identity:
        return None  # Reused leader identity never confers ownership.
    members = []
    try:
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():
                continue
            try:
                fields = (proc / 'stat').read_text().rsplit(') ', 1)[1].split()
                if int(fields[2]) == identity['group'] and fields[0] not in ('Z', 'X'):
                    members.append(int(proc.name))
            except FileNotFoundError:
                continue
    except (OSError, ValueError, IndexError):
        return None
    return members


def validate_integrations(records):
    def require(identity, kind):
        r = records.get(identity)
        if r is None or r['kind'] != kind:
            raise ProgramError('Missing integration ' + kind + ' reference')
        return r
    for r in records.values():
        d = r['details']
        if r['kind'] == 'integration_job':
            if d.get('contract') != CONTRACT or d.get('state') not in (*LEASED, 'finished', 'released'):
                raise ProgramError('Invalid integration job contract/state')
            if d.get('preparation_id'):
                p = require(d['preparation_id'], 'integration_preparation')
                if p['details']['manifest']['job_id'] != r['id']:
                    raise ProgramError('Integration preparation belongs to another job')
        elif r['kind'] in ('integration_preparation', 'integration_check'):
            m = d.get('manifest', {})
            if d.get('sha256') != json_digest(m) or m.get('contract') != CONTRACT:
                raise ProgramError('Integration receipt hash/contract mismatch')
            require(m.get('job_id'), 'integration_job')
            for source in m.get('sources', []):
                sub = require(source['submission_id'], 'submission')
                admission = require(source['admission_id'], 'admission')
                if (source['submission_sha256'] != sub['details']['sha256'] or
                        source['review_id'] != admission['details']['review_id'] or
                        source['submission_id'] != admission['details']['submission_id'] or
                        admission['details']['decision'] != 'keep'):
                    raise ProgramError('Integration source differs from reviewed admission')
            if r['kind'] == 'integration_check':
                p = require(m.get('preparation_id'), 'integration_preparation')
                if m.get('preparation_sha256') != p['details']['sha256']:
                    raise ProgramError('Integration check preparation hash mismatch')
                prepared = p['details']['manifest']
                for key in ('sources', 'combined_revision', 'combined_tree', 'config', 'config_sha256',
                            'input_identities', 'input_identities_sha256'):
                    if m.get(key) != prepared.get(key):
                        raise ProgramError('Integration check differs from prepared ' + key)
                if m.get('outcome') == 'passed':
                    before, after = m.get('pre_identity'), m.get('post_identity')
                    plan, execution = m.get('plan', {}), m.get('execution')
                    if (prepared['outcome'] != 'prepared' or not before or before != after or
                            before['revision'] != prepared['combined_revision'] or before['status'] or
                            before['tree'] != before['index_tree'] or before['tree'] != prepared['combined_tree'] or
                            plan.get('config') != prepared['config'] or
                            plan.get('input_identities') != prepared['input_identities'] or
                            not isinstance(execution, list) or not execution or
                            [x['check'] for x in execution] != plan.get('checks') or
                            any(x['returncode'] != 0 for x in execution) or prepared.get('reviewer_blockers') or
                            (prepared['physics_gate_required'] and not any(
                                c['argv'] == ['npm', 'run', 'test:physics'] for c in plan['checks'])) or
                            any(not any(c['argv'] == argv for c in plan['checks']) for argv in prepared['required_checks'])):
                        raise ProgramError('Passing integration receipt lacks exact clean checks and required gates')


class IntegrationController:
    def __init__(self, store, root, reviews):
        self.store, self.root, self.reviews = store, Path(root).resolve(), reviews
        self.owner = {'id': _identity('integration-controller'), 'process': process_identity(os.getpid())}
        self.children = {}

    def _save(self, program_id, identity, details):
        with self.store.transaction():
            self.store._touch(program_id)
            self.store.connection.execute('UPDATE records SET details_json=?, revision=revision+1, updated_at=? WHERE id=?',
                                          (_json(details), _now(), identity))

    def _receipt(self, program_id, kind, manifest):
        identity = _identity(kind)
        path = self.store.directory / 'integrations' / manifest['job_id'] / (identity + '.json')
        raw = (_json(manifest) + '\n').encode()
        path.write_bytes(raw)
        details = {'manifest': manifest, 'sha256': json_digest(manifest), 'path': str(path), 'file_sha256': digest(raw)}
        return self.store._insert(program_id, kind, manifest['outcome'], details, identity)

    def _verify_receipt(self, receipt):
        d = receipt['details']
        try:
            raw = Path(d['path']).read_bytes()
            if (json_digest(d['manifest']) != d['sha256'] or digest(raw) != d['file_sha256'] or
                    json.loads(raw) != d['manifest']):
                raise ProgramError('Integration receipt integrity mismatch')
        except (OSError, ValueError) as e:
            raise ProgramError('Integration receipt integrity check failed: ' + str(e)) from e
        return d['manifest']

    def _sources(self, program_id, submission_ids):
        sources = []
        for identity in submission_ids:
            submission, gaps = self.reviews.verify(program_id, identity)
            if gaps:
                raise ProgramError('Submission has missing evidence: ' + '; '.join(gaps))
            m = submission['details']['manifest']
            states = self.store.graph().query(program_id, 'support', m['claim']['id'])['states']
            if states[identity]['admission_state'] != 'reviewed':
                raise ProgramError('Integration requires a current accepted reviewed finding: ' + identity)
            admission = self.store.get_record(program_id, states[identity]['admission_ids'][-1], 'admission')
            review = self.store.get_record(program_id, admission['details']['review_id'], 'review')
            sources.append({'submission_id': identity, 'submission_sha256': submission['details']['sha256'],
                            'review_id': admission['details']['review_id'], 'admission_id': admission['id'],
                            'base_revision': m['base_revision'], 'candidate_revision': m['candidate_revision'],
                            'diff_sha256': m['diff_sha256'], 'config': m['config'],
                            'input_identities': m['input_identities'], 'physics_gate_required': m['physics_gate_required']})
            sources[-1].update(code_readiness=admission['details']['code_readiness'],
                               engineering_reasons=admission['details'].get('engineering_reasons', []),
                               review_code_readiness=review['details']['response']['code_readiness'],
                               required_checks=review['details']['response'].get('required_checks', []))
        return sources

    def prepare(self, program_id, submission_ids, *, target_base, config, input_identities):
        """Combine ordered sealed submissions at an explicit base. Retain all outcomes."""
        identities(submission_ids, 'submission_ids', nonempty=True)
        _object(config, 'combined configuration')
        identities(input_identities, 'combined input identities', nonempty=True)
        base = commit(self.root, target_base)
        if self.store.get_program(program_id)['state'] != 'active':
            raise ProgramError('Integration preparation requires an active program')
        identity = _identity('integration')
        directory = self.store.directory / 'integrations' / identity
        d = {'contract': CONTRACT, 'state': 'preparing', 'owner': self.owner, 'process': None,
             'directory': str(directory), 'worktree': str(directory / 'worktree'),
             'branch': 'research-integration/' + identity, 'preparation_id': None, 'check_id': None}
        with self.store.transaction():
            from .program_pool import check_exclusive
            check_exclusive(self.store)
            self.store._touch(program_id)
            self.store._insert(program_id, 'integration_job', 'Combined candidate lease', d, identity)
        directory.mkdir(mode=0o700, parents=True)
        m = {'contract': CONTRACT, 'job_id': identity, 'target_base': base, 'submission_ids': submission_ids,
             'sources': [], 'config': config, 'config_sha256': json_digest(config),
             'input_identities': input_identities, 'input_identities_sha256': json_digest(input_identities),
             'worktree': d['worktree'], 'branch': d['branch'], 'combined_revision': None,
             'combined_tree': None, 'diff_sha256': None, 'changed_paths': [], 'physics_gate_required': False,
             'required_checks': [], 'reviewer_blockers': [],
             'outcome': 'preparation_failed', 'reason': None, 'at': _now()}
        try:
            try:
                m['sources'] = self._sources(program_id, submission_ids)
            except ProgramError as error:
                m.update(outcome='changed_or_unreviewed', reason=str(error))
                return self._finish_prepare(program_id, identity, d, m)
            patches = [s for s in m['sources'] if s['candidate_revision']]
            m['required_checks'] = [argv for source in patches for argv in source['required_checks']]
            m['reviewer_blockers'] = [source['submission_id'] + ': reviewer code readiness is ' + source['review_code_readiness']
                                      for source in patches if source['review_code_readiness'] != 'ready']
            if not patches:
                m.update(outcome='report_only', reason='Accepted findings have no code patch; no integration needed')
                return self._finish_prepare(program_id, identity, d, m)
            if any(s['base_revision'] != base for s in patches):
                m.update(outcome='stale_base', reason='Sealed base differs from explicit target; submit and review a new candidate')
                return self._finish_prepare(program_id, identity, d, m)
            for source in patches:
                result = subprocess.run(['git', '-C', str(self.root), 'merge-base', '--is-ancestor',
                                         base, source['candidate_revision']], capture_output=True)
                if result.returncode:
                    m.update(outcome='stale_base', reason='Sealed candidate is not descended from explicit base')
                    return self._finish_prepare(program_id, identity, d, m)
            # No worker helper: it uses ambient HEAD and shares mutable output/build links.
            git(self.root, '-c', 'core.hooksPath=/dev/null', 'worktree', 'add', '-b', d['branch'], d['worktree'], base)
            for index, source in enumerate(patches):
                raw = (self.store.directory / 'submissions' / source['submission_id'] / 'candidate.diff').read_bytes()
                if digest(raw) != source['diff_sha256']:
                    raise ProgramError('Frozen patch changed during preparation')
                path = directory / f'{index:02d}.diff'
                path.write_bytes(raw)
                result = subprocess.run(['git', '-C', d['worktree'], 'apply', '--index', '--binary', str(path)], capture_output=True)
                (directory / f'apply-{index:02d}.log').write_bytes(result.stdout + result.stderr)
                if result.returncode:
                    m.update(outcome='conflict', reason='Frozen patches do not apply together: ' + source['submission_id'])
                    return self._finish_prepare(program_id, identity, d, m)
            git(d['worktree'], '-c', 'core.hooksPath=/dev/null', '-c', 'commit.gpgsign=false',
                '-c', 'user.name=Research integration', '-c', 'user.email=research-integration@localhost',
                'commit', '--allow-empty', '-m', 'Prepare reviewed combined candidate ' + identity)
            revision = commit(d['worktree'], 'HEAD')
            paths = git(d['worktree'], 'diff', '--name-only', '-z', base, revision, '--').decode().split('\0')[:-1]
            raw = git(d['worktree'], 'diff', '--binary', base, revision, '--')
            (directory / 'combined.diff').write_bytes(raw)
            if self._sources(program_id, submission_ids) != m['sources']:
                raise ProgramError('Reviewed source identities changed during preparation')
            m.update(outcome='prepared', combined_revision=revision,
                     combined_tree=git(d['worktree'], 'rev-parse', 'HEAD^{tree}').decode().strip(),
                     diff_sha256=digest(raw), changed_paths=paths,
                     physics_gate_required=physics_changed(paths) or any(s['physics_gate_required'] for s in patches))
            m['physics_gate_required'] |= ['npm', 'run', 'test:physics'] in m['required_checks']
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            m.update(outcome='preparation_failed', reason=str(error))
        return self._finish_prepare(program_id, identity, d, m)

    def _finish_prepare(self, program_id, identity, d, m):
        with self.store.transaction():
            self.store._touch(program_id)
            receipt = self._receipt(program_id, 'integration_preparation', m)
            d.update(state='prepared' if m['outcome'] == 'prepared' else 'finished', preparation_id=receipt['id'])
            self.store.connection.execute('UPDATE records SET details_json=?, revision=revision+1, updated_at=? WHERE id=?',
                                          (_json(d), _now(), identity))
        return self.status(program_id, identity)

    def _identity(self, worktree):
        return {'revision': commit(worktree, 'HEAD'),
                'tree': git(worktree, 'rev-parse', 'HEAD^{tree}').decode().strip(),
                'index_tree': git(worktree, 'write-tree').decode().strip(),
                'status': git(worktree, 'status', '--porcelain=v1', '--untracked-files=all').decode()}

    def _fixed(self, program_id, d):
        receipt = self.store.get_record(program_id, d['preparation_id'], 'integration_preparation')
        m = self._verify_receipt(receipt)
        if (m['outcome'] != 'prepared' or self._sources(program_id, m['submission_ids']) != m['sources'] or
                digest((Path(d['directory']) / 'combined.diff').read_bytes()) != m['diff_sha256']):
            raise ProgramError('Combined candidate or reviewed sources changed')
        identity = self._identity(d['worktree'])
        if (identity['revision'] != m['combined_revision'] or identity['tree'] != m['combined_tree'] or
                identity['index_tree'] != identity['tree'] or identity['status']):
            raise ProgramError('Combined candidate identity changed or worktree is dirty')
        return receipt, m, identity

    def _runner_identity(self, d):
        if d.get('process'):
            return d['process']
        try:
            return json.loads((Path(d['directory']) / 'runner-identity.json').read_text())
        except (OSError, ValueError):
            return None

    def _evaluator(self):
        runner = self.root / 'scripts/research_program_check_runner.py'
        return {'contract': CONTRACT, 'source_revision': commit(self.root, 'HEAD'),
                'controller_sha256': digest(Path(__file__).read_bytes()), 'runner_sha256': digest(runner.read_bytes())}

    def check(self, program_id, integration_id, *, checks, config, input_identities, wall_minutes=5):
        """Start finite argv checks; return immediately and poll integration_status."""
        if self.store.get_program(program_id)['state'] != 'active':
            raise ProgramError('Integration checks require an active program')
        if (isinstance(wall_minutes, bool) or not isinstance(wall_minutes, (int, float)) or
                not math.isfinite(wall_minutes) or wall_minutes <= 0 or wall_minutes > 90):
            raise ProgramError('Checks require finite wall_minutes in (0, 90]')
        if not isinstance(checks, list) or not checks or len(checks) > 16:
            raise ProgramError('Select 1–16 relevant checks')
        for check in checks:
            if (not isinstance(check, dict) or set(check) != {'name', 'argv'} or
                    not isinstance(check['argv'], list) or not check['argv'] or
                    not all(isinstance(arg, str) and arg and '\0' not in arg for arg in check['argv'])):
                raise ProgramError('Each check requires name and a nonempty argv string list')
            _text(check['name'], 'check name')
        r = self.store.get_record(program_id, integration_id, 'integration_job')
        d = r['details']
        if d['state'] != 'prepared' or d['owner'] != self.owner:
            raise ProgramError('Checks require this controller\'s prepared integration lease')
        receipt = self.store.get_record(program_id, d['preparation_id'], 'integration_preparation')
        m = self._verify_receipt(receipt)
        if config != m['config'] or input_identities != m['input_identities']:
            raise ProgramError('Check configuration/input identities differ from prepared candidate')
        d['check_plan'] = {'checks': checks, 'config': config, 'input_identities': input_identities,
                           'deadline': time.time() + wall_minutes * 60, 'started_at': _now(), 'evaluator': self._evaluator(),
                           'cleanup_boundary': CLEANUP_BOUNDARY,
                           'configuration_boundary': 'Configuration and input IDs are supplied declarations; Git identities and command outcomes are observed'}
        d['pre_identity'] = None
        try:
            _, _, d['pre_identity'] = self._fixed(program_id, d)
        except (OSError, ValueError, subprocess.SubprocessError) as e:
            return self._finish_check(program_id, integration_id, d, 'changed_candidate', str(e))
        if m['reviewer_blockers']:
            return self._finish_check(program_id, integration_id, d, 'engineering_blocked', '; '.join(m['reviewer_blockers']))
        if m['physics_gate_required'] and not any(c['argv'] == ['npm', 'run', 'test:physics'] for c in checks):
            return self._finish_check(program_id, integration_id, d, 'missing_physics_gate',
                                      'Final combined revision requires real npm run test:physics')
        if any(not any(c['argv'] == argv for c in checks) for argv in m['required_checks']):
            return self._finish_check(program_id, integration_id, d, 'missing_required_check',
                                      'Combined plan omits an explicit reviewer required check')
        d.update(state='checking', process=None)
        self._save(program_id, integration_id, d)  # Charge survives the Popen gap.
        directory = Path(d['directory'])
        (directory / 'check-plan.json').write_text(_json(d['check_plan']))
        try:
            with (directory / 'runner.log').open('wb') as stdout:
                process = subprocess.Popen([sys.executable, str(self.root / 'scripts/research_program_check_runner.py'),
                                            str(directory), d['worktree']], cwd=d['worktree'],
                                           stdin=subprocess.DEVNULL, stdout=stdout, stderr=stdout, start_new_session=True)
            self.children[integration_id] = process
            d['process'] = process_identity(process.pid)
            self._save(program_id, integration_id, d)
        except OSError as e:
            return self._finish_check(program_id, integration_id, d, 'check_failed', str(e))
        return self.status(program_id, integration_id)

    def _finish_check(self, program_id, identity, d, outcome, reason, *, execution=None, post=None):
        prep = self.store.get_record(program_id, d['preparation_id'], 'integration_preparation')
        m = prep['details']['manifest']
        logs = []
        for path in sorted(Path(d['directory']).glob('check-*.log')):
            logs.append({'path': str(path), 'sha256': digest(path.read_bytes())})
        manifest = {'contract': CONTRACT, 'job_id': identity, 'preparation_id': prep['id'],
                    'preparation_sha256': prep['details']['sha256'], 'sources': m['sources'],
                    'combined_revision': m['combined_revision'], 'combined_tree': m['combined_tree'],
                    'config': m['config'], 'config_sha256': m['config_sha256'],
                    'input_identities': m['input_identities'], 'input_identities_sha256': m['input_identities_sha256'],
                    'plan': d['check_plan'], 'pre_identity': d['pre_identity'], 'post_identity': post,
                    'execution': execution, 'logs': logs, 'outcome': outcome, 'reason': reason, 'at': _now()}
        with self.store.transaction():
            self.store._touch(program_id)
            receipt = self._receipt(program_id, 'integration_check', manifest)
            d.update(state='finished', check_id=receipt['id'])
            self.store.connection.execute('UPDATE records SET details_json=?, revision=revision+1, updated_at=? WHERE id=?',
                                          (_json(d), _now(), identity))
        return self.status(program_id, identity)

    def status(self, program_id, integration_id):
        """Explain accepted findings and engineering readiness with immutable receipts."""
        r = self.store.get_record(program_id, integration_id, 'integration_job')
        d = r['details']
        process = self.children.get(integration_id)
        if d['state'] == 'checking' and process is not None and process.poll() is not None:
            detached = Path(d['directory']) / 'detached-observation.json'
            if detached.exists():
                d.update(state='ownership_lost', cleanup='unsupported detached work; exclusive lease retained',
                         detached_observation={'path': str(detached), 'sha256': digest(detached.read_bytes())})
                self._save(program_id, integration_id, d)
                return self._status_record(program_id, integration_id)
            members = group_members(self._runner_identity(d))
            if members is None:
                d.update(state='ownership_lost', cleanup='unknown process-group boundary; exclusive lease retained')
                self._save(program_id, integration_id, d)
                return self.status(program_id, integration_id)
            if members:
                # An owned runner can crash before its own cleanup. Kill only its
                # birth-verified group and retain the lease until exit is observed.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                return self._status_record(program_id, integration_id)
            execution, post, reason, outcome = None, None, None, 'check_failed'
            try:
                _, m, post = self._fixed(program_id, d)
                if post != d['pre_identity']:
                    raise ProgramError('Source identity changed during checks')
                if self._evaluator() != d['check_plan']['evaluator']:
                    raise ProgramError('Evaluator source identity changed during checks')
                directory = Path(d['directory'])
                if d.get('cancel_requested'):
                    outcome, reason = 'cancelled', d['cancel_requested']
                elif (directory / 'wall-expired').exists():
                    outcome, reason = 'timed_out', 'Check wall allocation expired'
                elif not (directory / 'execution.json').exists():
                    reason = 'Check runner exited ' + str(process.returncode)
                else:
                    execution = json.loads((directory / 'execution.json').read_text())
                    if (not isinstance(execution, list) or len(execution) != len(d['check_plan']['checks']) or
                            any(x['check'] != c for x, c in zip(execution, d['check_plan']['checks']))):
                        raise ProgramError('Check execution differs from fixed plan')
                    if all(x['returncode'] == 0 for x in execution):
                        outcome = 'passed'
                    else:
                        reason = 'Combined checks failed; producer passes cannot establish combined readiness'
            except (OSError, ValueError, KeyError, subprocess.SubprocessError) as e:
                outcome, reason = 'changed_candidate', str(e)
            return self._finish_check(program_id, integration_id, d, outcome, reason, execution=execution, post=post)
        return self._status_record(program_id, integration_id)

    def _status_record(self, program_id, integration_id):
        r = self.store.get_record(program_id, integration_id, 'integration_job')
        d = r['details']
        preparations = [self.store.get_record(program_id, d['preparation_id'])] if d['preparation_id'] else []
        checks = [self.store.get_record(program_id, d['check_id'])] if d['check_id'] else []
        return {'job': r, 'preparation': preparations[0] if preparations else None,
                'check': checks[0] if checks else None,
                'patch_readiness': checks[0]['details']['manifest']['outcome'] if checks else
                                  preparations[0]['details']['manifest']['outcome'] if preparations and d['state'] == 'finished'
                                  else d['state'],
                'claim_acceptance': 'Separate reviewed admissions remain unchanged by integration outcomes',
                'cleanup_boundary': CLEANUP_BOUNDARY,
                'lease_held': d['state'] in LEASED, 'automatic_merge_or_push': False}

    def release(self, program_id, integration_id, reason):
        """Cancel owned checks or release a prepared lease; preserve candidate and receipts."""
        _text(reason, 'release reason')
        r = self.store.get_record(program_id, integration_id, 'integration_job')
        d = r['details']
        if d['state'] not in LEASED:
            return self.status(program_id, integration_id)
        if d['owner'] != self.owner:
            raise ProgramError('Historical integration lease requires explicit recovery after process shutdown')
        process = self.children.get(integration_id)
        if process is not None and 'check_plan' in d:
            identity = self._runner_identity(d)
            members = group_members(identity)
            if members is None:
                raise ProgramError('Refusing to signal unverified check process')
            d['cancel_requested'] = reason
            d['state'] = 'checking'
            self._save(program_id, integration_id, d)
            try:
                if members:
                    os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            # Keep the lease until a later poll observes exit.
            return self.status(program_id, integration_id)
        if d['state'] == 'checking':
            return self.status(program_id, integration_id)
        d.update(state='released', release={'reason': reason, 'at': _now()})
        self._save(program_id, integration_id, d)
        return self.status(program_id, integration_id)

    def recover(self, program_id, integration_id, *, reason=None):
        """Never adopt historical PIDs. Retain exclusion until owner and child are gone."""
        r = self.store.get_record(program_id, integration_id, 'integration_job')
        d = r['details']
        if d['state'] not in LEASED or (d['owner'] == self.owner and d['state'] != 'ownership_lost'):
            return self.status(program_id, integration_id)
        if process_live(d['owner'].get('process')) and d['owner']['process'] != self.owner['process']:
            raise ProgramError('Live foreign controller retains integration lease')
        if d['state'] != 'ownership_lost':
            d.update(state='ownership_lost', recovery={'at': _now(), 'adopted': False, 'signalled': False})
            self._save(program_id, integration_id, d)
        if reason:
            _text(reason, 'recovery reason')
            if 'check_plan' in d:
                members = group_members(self._runner_identity(d))
                if members is None:
                    raise ProgramError('Check startup/cleanup identity unknown; exclusive lease retained')
                if members:
                    raise ProgramError('Historical check process group remains live; exclusive lease retained')
                observation = Path(d['directory']) / 'detached-observation.json'
                if observation.exists():
                    try:
                        detached = json.loads(observation.read_text())['processes']
                    except (OSError, ValueError, KeyError) as error:
                        raise ProgramError('Detached cleanup unknown; exclusive lease retained') from error
                    if any(process_live(p) for p in detached):
                        raise ProgramError('Observed detached process remains live; exclusive lease retained')
            elif not d.get('preparation_id'):
                raise ProgramError('Preparation startup/cleanup unknown; exclusive lease retained')
            d.update(state='released', release={'at': _now(), 'reason': reason,
                                               'checks_admitted': False, 'adopted': False, 'signalled': False})
            self._save(program_id, integration_id, d)
        return self.status(program_id, integration_id)
