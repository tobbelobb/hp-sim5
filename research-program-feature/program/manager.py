"""Program-bound chat tools. Physics stays in the existing worker launcher."""
import inspect
from pathlib import Path
import sqlite3
import subprocess
import traceback

from .store import ProgramError, _object
from .prompts import PromptPortfolio
from .workers import ACTIVE, runtime_check
from .controller import ProgramController
from .integration import IntegrationController
from hp_sim5_research.coding_client import research_client
from hp_sim5_research.client_transport import client_adapter
from . import pool


def failure(code, message):
    return {'ok': False, 'error': {'code': code, 'message': message}}


class ProgramManager:
    """Retained dispatcher and separate reviewer for exactly one program.

    Calls run on its host thread; SQLite connections never cross threads.
    """

    def __init__(self, store, root, program_id, controller_id, session, *, workers=None, reviews=None):
        store.get_program(program_id)
        self.store, self.root, self.program_id = store, Path(root).resolve(), program_id
        self.controller_id, self.session = controller_id, Path(session)
        self.session.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.workers = workers or store.workers(self.root)
        self.reviews = reviews or store.reviews(self.root)
        self.integrations = IntegrationController(store, self.root, self.reviews)
        self.prompts = PromptPortfolio(self.root)
        self.stopping = False
        self.cleanup_errors = []
        self.controller = ProgramController(self)
        self.operations = {
            'overview': self.overview, 'status': self.overview, 'capabilities': self.capabilities,
            'runtime_check': self.runtime_check,
            'query': store.graph().query, 'question': store.add_question, 'approach': store.add_approach,
            'assignment': store.add_assignment, 'artifact': store.add_artifact,
            'claim': store.graph().add_claim, 'derive': store.graph().add_derivation,
            'relate': store.graph().add_relation, 'assess': store.graph().add_assessment,
            'review_event': store.graph().review_event, 'transition': self.transition,
            'steering': self.steering, 'decision': self.decision,
            'prompt_portfolio': self.prompt_portfolio,
            'commission': self.commission,
            'worker_start': self.worker_start, 'worker_status': self.workers.status,
            'worker_answer': self.workers.answer,
            'worker_cancel': self.workers.cancel,
            'worker_sessions': self.workers.sessions,
            'configure_pool': self.controller.configure_pool,
            'submit': self.submit, 'submission_verify': self.reviews.verify,
            'submission_invalidate': self.reviews.invalidate, 'review_plan': self.reviews.plan,
            'review_start': self.reviews.start, 'review_status': self.reviews.status,
            'review_cancel': self.reviews.cancel, 'review_apply': self.review_apply,
            'pause': self.pause, 'resume': self.resume, 'stop': self.stop,
            'shutdown': self.shutdown,
            'recover': self.controller.recover,
            'integration_prepare': self.integrations.prepare, 'integration_check': self.integrations.check,
            'integration_status': self.integrations.status, 'integration_release': self.integrations.release,
            'integration_recover': self.integrations.recover, 'integration_finish': self.integrations.finish,
        }

    def runtime_check(self, program_id, *, physics_backend='headless-js'):
        """Check dependencies and launcher socket permissions before work; no motion."""
        if physics_backend not in ('headless-js', 'native-python', 'native-warp', 'native-warp-cuda'):
            raise ProgramError(f'Unsupported physics_backend: {physics_backend}')
        return runtime_check(self.root, physics_backend=physics_backend)

    def prompt_portfolio(self, program_id, *, prompt_id=None, include_text=False):
        """List mission prompts, or load one selected prompt for worker assignment."""
        if prompt_id is None:
            if include_text:
                raise ProgramError('include_text requires a selected prompt_id')
            return self.prompts.inventory()
        return self.prompts.resolve(prompt_id, include_text=include_text)

    def capabilities(self, program_id):
        operations = {}
        for name, method in self.operations.items():
            signature = inspect.signature(method)
            operations[name] = {
                'parameters': [str(p) for k, p in signature.parameters.items() if k != 'program_id'],
                'description': (inspect.getdoc(method) or '').split('\n')[0],
            }
        return {'operations': operations, 'client_transport': research_client(),
                'worker_machine': 'Required hp3/hp4: settings.machine in commission or worker_start; conflicting choices are rejected',
                'physics_world': False, 'default_worker_concurrency': 1,
                'boundary': 'Cooperative same-UID local tools; private files are not adversarial isolation',
                'restart': 'Leases and birth identities reconcile to interrupted handoffs; historical PIDs never authorize signals'}

    def call(self, operation, parameters=None):
        """One structured result for CLI and MCP, including invalid API arguments."""
        try:
            _object({} if parameters is None else parameters, 'parameters')
            parameters = {} if parameters is None else parameters
            if not isinstance(operation, str) or operation not in self.operations:
                return failure('unknown_operation', 'Unknown manager operation; call capabilities')
            if 'program_id' in parameters:
                raise ProgramError('Program identity is bound by the attachment; it cannot be overridden')
            if self.stopping and operation not in ('overview', 'status', 'capabilities', 'prompt_portfolio', 'worker_status',
                                                   'worker_sessions', 'review_status', 'integration_status',
                                                   'integration_recover', 'integration_release', 'shutdown'):
                raise ProgramError('Controller is shutting down; inspect status until cleanup completes')
            method = self.operations[operation]
            inspect.signature(method).bind(self.program_id, **parameters)
            if operation == 'review_start' and self.store.get_program(self.program_id)['state'] != 'active':
                raise ProgramError('Review dispatch requires an active program')
            result = method(self.program_id, **parameters)
            return {'ok': True, 'result': result}
        except (ValueError, TypeError, OSError, sqlite3.Error, subprocess.SubprocessError) as error:
            return failure('invalid_request' if isinstance(error, (ValueError, TypeError)) else 'operation_failed', str(error))
        except Exception:
            traceback.print_exc()
            return failure('internal_error', 'Manager operation failed; inspect controller.log')

    def poll(self):
        """Observe owned children between requests, including cancellation cleanup."""
        for identity in self.child_ids(self.workers, 'attempt'):
            self.workers.status(self.program_id, identity)
        for identity in self.child_ids(self.reviews, 'review_run'):
            self.reviews.status(self.program_id, identity)
        for identity in self.child_ids(self.integrations, 'integration_job'):
            self.integrations.status(self.program_id, identity)

    def child_ids(self, dispatcher, kind):
        return [r[0] for r in self.store.connection.execute(
            'SELECT id FROM records WHERE program_id=? AND kind=?', (self.program_id, kind)) if r[0] in dispatcher.children]

    def overview(self, program_id):
        """Return short progress and result paths; inspect individual records for details."""
        self.poll()
        snapshot = self.store.snapshot(program_id)
        records = snapshot['records']
        program = snapshot['program']
        return {'program': program,
                'work': [{'id': r['id'], 'question': r['title'],
                          'state': r['details']['execution_state']} for r in records if r['kind'] == 'assignment'],
                'researchers': [{'id': r['id'], 'state': r['details']['state'],
                                 'report': str(Path(r['details']['session_dir']) / 'report.md'),
                                 'question': self.workers.status(program_id, r['id']).get('manager_question')}
                                for r in records if r['kind'] == 'attempt'],
                'reviews': [{'id': r['id'], 'state': r['details']['state'],
                             'review_id': r['details'].get('review_id')} for r in records if r['kind'] == 'review_run'],
                'integrations': [{'id': r['id'], 'state': r['details']['state'],
                                  'delivered_revision': r['details'].get('delivered_revision')}
                                 for r in records if r['kind'] == 'integration_job'],
                'summary': str(self.root / 'research/output' / program_id / 'summary.html'),
                'manager': {'program_id': program_id, 'controller_id': self.controller_id,
                            'client_transport': research_client(), 'stopping': self.stopping,
                            'physics_world': False, 'cleanup_errors': self.cleanup_errors,
                            'links': {'store': str(self.store.path), 'session': str(self.session)}}}

    def commission(self, program_id, objective, *, settings=None):
        """Give a researcher a question and optional launcher settings."""
        settings = {'worktree': True, **_object({} if settings is None else settings, 'settings')}
        program = self.store.get_program(program_id)
        approach = self.store.add_approach(program_id, program['question_id'], objective, objective)
        assignment = self.store.add_assignment(program_id, approach['id'], objective, settings=settings)
        return self.store.transition(program_id, assignment['id'], 'ready', 'Commissioned researcher')

    def submit(self, program_id, attempt_id, *, base_revision, candidate_revision=None,
               selected, title=None, claim_id=None, config=None, input_identities=None, producer_checks=None):
        """Select canonical report/evidence files and an optional committed change for review."""
        attempt = self.store.get_record(program_id, attempt_id, 'attempt')
        if claim_id is None:
            claim = self.store.graph().add_claim(program_id, title or attempt['title'],
                                                 {'session': attempt['details']['session_dir']})
            claim_id = claim['id']
        return self.reviews.submit(program_id, attempt_id, claim_id, base_revision=base_revision,
                                   candidate_revision=candidate_revision, selected=selected,
                                   config={} if config is None else config,
                                   input_identities=input_identities or [attempt_id],
                                   producer_checks=[] if producer_checks is None else producer_checks)

    def transition(self, program_id, record_id, target, reason, *, expected_revision=None):
        if record_id == program_id:
            raise ProgramError('Use pause, resume or stop for program lifecycle controls')
        return self.store.transition(program_id, record_id, target, reason, expected_revision=expected_revision)

    def worker_start(self, program_id, assignment_id, *, settings=None, instructions=''):
        """Start the commissioned researcher; retain its context and canonical session."""
        settings = self.workers.assignment_settings(program_id, assignment_id, settings)
        if pool.limit(self.store) > 1:
            settings['worktree'] = True
        adapter = client_adapter()
        if adapter:
            adapter.ensure_runtime_ready(self.root, settings, runtime_check)
        steering = [r['title'] for r in self.store.snapshot(program_id)['records'] if r['kind'] == 'steering'
                    and r['details']['subject_id'] in (None, program_id, assignment_id)]
        if steering:
            instructions = 'Human direction:\n' + '\n'.join(steering) + '\n\n' + instructions
        return self.workers.start(program_id, assignment_id, settings=settings, instructions=instructions)

    def review_apply(self, program_id, review_id):
        """Record a review while the program is active or paused."""
        if self.store.get_program(program_id)['state'] in ('stopped', 'completed'):
            raise ProgramError('A stopped or completed program cannot apply a new review')
        return self.reviews.apply(program_id, review_id)

    def steering(self, program_id, text, *, subject_id=None, actor='human'):
        """Save human direction without prescribing experiments or allocations."""
        return self.store.record_note(program_id, 'steering', text, subject_id=subject_id, actor=actor)

    def decision(self, program_id, text, *, subject_id=None, evidence_ids=None, actor='manager',
                 record_id=None, expected_revision=None):
        return self.store.record_note(program_id, 'decision', text, subject_id=subject_id, evidence_ids=evidence_ids,
                                      actor=actor, record_id=record_id, expected_revision=expected_revision)

    def _cancel_owned(self):
        for identity in self.child_ids(self.workers, 'attempt'):
            record = self.workers.status(self.program_id, identity)
            if record['details']['state'] in ACTIVE:
                try:
                    self.workers.cancel(self.program_id, identity)
                except ProgramError as error:
                    self.cleanup_errors.append({'attempt_id': identity, 'error': str(error), 'physical_shutdown': 'unknown'})
        for identity in self.child_ids(self.reviews, 'review_run'):
            try:
                self.reviews.cancel(self.program_id, identity)
            except ProgramError as error:
                self.cleanup_errors.append({'review_id': identity, 'error': str(error), 'reviewer_shutdown': 'unknown'})
        for r in self.store.snapshot(self.program_id)['records']:
            if r['kind'] == 'integration_job' and r['details']['owner'] == self.integrations.owner:
                try:
                    self.integrations.release(self.program_id, r['id'], 'Manager stop/shutdown')
                except ProgramError as error:
                    self.cleanup_errors.append({'integration_id': r['id'], 'error': str(error), 'check_shutdown': 'unknown'})

    def pause(self, program_id, reason):
        """Pause new dispatch. Running workers and reviewers continue until explicitly stopped."""
        if self.store.get_program(program_id)['state'] != 'paused':
            self.store.transition(program_id, program_id, 'paused', reason)
        return self.overview(program_id)

    def resume(self, program_id, reason):
        """Allow explicit dispatch again; unfinished conversations are never restarted."""
        self.store.transition(program_id, program_id, 'active', reason)
        return self.overview(program_id)

    def stop(self, program_id, reason):
        """Stop new dispatch and cancel owned children; retain evidence and poll cleanup."""
        if self.store.get_program(program_id)['state'] != 'stopped':
            self.store.transition(program_id, program_id, 'stopped', reason)
        self._cancel_owned()
        return self.overview(program_id)

    def shutdown(self, program_id):
        """Retire this tool host after owned children exit; program history remains."""
        self.stopping = True
        self._cancel_owned()
        return self.overview(program_id)

    def drained(self):
        return all(r['details']['state'] not in ('preparing', 'prepared', 'checking', 'ownership_lost')
                   for r in self.store.snapshot(self.program_id)['records']
                   if r['kind'] == 'integration_job' and r['details']['owner'] == self.integrations.owner) and all(
                   dispatcher.children[i].poll() is not None for dispatcher, kind in
                   ((self.workers, 'attempt'), (self.reviews, 'review_run'),
                    (self.integrations, 'integration_job')) for i in self.child_ids(dispatcher, kind))
