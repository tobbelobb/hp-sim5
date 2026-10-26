"""Program-bound chat tools. Physics stays in the existing worker launcher."""
import inspect
from pathlib import Path
import sqlite3
import subprocess
import traceback

from .program import ProgramError, _json, _object, render_markdown
from .program_workers import ACTIVE
from .program_strategy import ProgramStrategy
from .program_controller import ProgramController
from .program_integration import IntegrationController
from .program_save import save_program
from .coding_client import research_client


UNAVAILABLE = {}


def failure(code, message):
    return {'ok': False, 'error': {'code': code, 'message': message}}


class ProgramManager:
    """Retained dispatcher and separate reviewer for exactly one program.

    This retains a finite allocation/recovery controller. All
    calls run on its host thread; SQLite connections never cross threads.
    """

    def __init__(self, store, root, program_id, controller_id, session, *, workers=None, reviews=None):
        store.get_program(program_id)
        self.store, self.root, self.program_id = store, Path(root).resolve(), program_id
        self.controller_id, self.session = controller_id, Path(session)
        self.session.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.workers = workers or store.workers(self.root)
        self.reviews = reviews or store.reviews(self.root)
        self.integrations = IntegrationController(store, self.root, self.reviews)
        self.strategy = ProgramStrategy(store)
        self.stopping = False
        self.cleanup_errors = []
        self.controller = ProgramController(self)
        self.operations = {
            'overview': self.overview, 'status': self.overview, 'capabilities': self.capabilities,
            'save': self.save,
            'query': store.graph().query, 'question': store.add_question, 'approach': store.add_approach,
            'assignment': store.add_assignment, 'artifact': store.add_artifact,
            'claim': store.graph().add_claim, 'derive': store.graph().add_derivation,
            'relate': store.graph().add_relation, 'assess': store.graph().add_assessment,
            'review_event': store.graph().review_event, 'transition': self.transition,
            'steering': self.steering, 'decision': self.decision,
            'strategy': self.strategy.reconcile, 'route_plan': self.strategy.route,
            'route_decide': self.strategy.decide, 'commission': self.strategy.commission,
            'strategy_result': self.strategy.result,
            'worker_start': self.worker_start, 'worker_status': self.workers.status,
            'worker_cancel': self.controller.cancel_attempt, 'cancel_attempt': self.controller.cancel_attempt,
            'worker_sessions': self.workers.sessions,
            'configure_pool': self.controller.configure_pool,
            'submit': self.reviews.submit, 'submission_verify': self.reviews.verify,
            'submission_invalidate': self.reviews.invalidate, 'review_plan': self.reviews.plan,
            'review_start': self.controller.review_start, 'review_status': self.reviews.status,
            'review_cancel': self.reviews.cancel, 'review_apply': self.controller.review_apply,
            'pause': self.pause, 'resume': self.resume, 'stop': self.stop,
            'shutdown': self.shutdown,
            'allocate': self.controller.allocate, 'run_once': self.controller.run_once,
            'run_within_allocation': self.controller.run_within_allocation,
            'runtime_status': self.controller.status, 'recover': self.controller.recover,
            'fund_review': self.controller.fund_review, 'worker_usage': self.controller.usage,
            'stop_allocation': self.controller.stop_allocation,
            'stop_program': self.stop,
            'integration_prepare': self.integrations.prepare, 'integration_check': self.integrations.check,
            'integration_status': self.integrations.status, 'integration_release': self.integrations.release,
            'integration_recover': self.integrations.recover,
        }

    def capabilities(self, program_id):
        operations = {}
        for name, method in self.operations.items():
            signature = inspect.signature(method)
            operations[name] = {
                'parameters': [str(p) for k, p in signature.parameters.items() if k != 'program_id'],
                'description': (inspect.getdoc(method) or '').split('\n')[0],
            }
        return {'operations': operations, 'unavailable': UNAVAILABLE, 'client_transport': research_client(),
                'physics_world': False, 'default_worker_concurrency': 1,
                'boundary': 'Cooperative same-UID local tools; private files are not adversarial isolation',
                'worker_allocation': 'Finite task/wall/review reservations; numerical counters remain explicit contracts',
                'restart': 'Leases and birth identities reconcile to interrupted handoffs; historical PIDs never authorize signals'}

    def call(self, operation, parameters=None):
        """One structured result for CLI and MCP, including invalid API arguments."""
        try:
            _object({} if parameters is None else parameters, 'parameters')
            parameters = {} if parameters is None else parameters
            if not isinstance(operation, str) or operation not in self.operations:
                if isinstance(operation, str) and operation in UNAVAILABLE:
                    return failure('unavailable', UNAVAILABLE[operation])
                return failure('unknown_operation', 'Unknown manager operation; call capabilities')
            if 'program_id' in parameters:
                raise ProgramError('Program identity is bound by the attachment; it cannot be overridden')
            if self.stopping and operation not in ('overview', 'status', 'capabilities', 'worker_status',
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
        self.controller.poll()

    def child_ids(self, dispatcher, kind):
        return [r[0] for r in self.store.connection.execute(
            'SELECT id FROM records WHERE program_id=? AND kind=?', (self.program_id, kind)) if r[0] in dispatcher.children]

    def overview(self, program_id):
        self.poll()
        board = self.strategy.reconcile(program_id)
        result = self.store.overview(program_id)
        result['strategy'] = board
        result['runtime'] = self.controller.status(program_id)
        result['next_allocation'] = self.strategy.next_allocation(program_id, board)
        result['brief'] = self.strategy.brief(board, result['next_allocation'])
        result['brief'] += '\n\nWorker pool: ' + str(result['runtime']['pool']) + '\n' + result['runtime']['shared_filesystem']
        result['integrations'] = [self.integrations.status(program_id, r['id']) for r in result['records']
                                  if r['kind'] == 'integration_job']
        result['manager'] = {'program_id': program_id, 'controller_id': self.controller_id,
                             'client_transport': research_client(),
                             'stopping': self.stopping, 'physics_world': False,
                             'cleanup_errors': self.cleanup_errors,
                             'links': {'store': str(self.store.path), 'session': str(self.session),
                                       'overview': str(self.session / 'overview.md'),
                                       'snapshot': str(self.session / 'overview.json'),
                                       'worker_sessions': [r['details']['session_dir'] for r in result['records']
                                                           if r['kind'] == 'attempt']},
                             'unavailable': UNAVAILABLE}
        (self.session / 'overview.json').write_text(_json(result) + '\n')
        (self.session / 'overview.md').write_text(render_markdown(result))
        (self.session / 'brief.md').write_text(result['brief'] + '\n')
        return result

    def save(self, program_id, *, files=(), commit=False):
        """Save/commit only this program's results in the controller's execution context."""
        self.poll()
        return save_program(self.store, self.root, program_id, files=files, commit=commit)

    def transition(self, program_id, record_id, target, reason, *, expected_revision=None):
        if record_id == program_id:
            raise ProgramError('Use pause, resume or stop for program lifecycle controls')
        return self.store.transition(program_id, record_id, target, reason, expected_revision=expected_revision)

    def worker_start(self, program_id, assignment_id, *, settings=None, instructions='', allocation_id=None, retry_of=None):
        """Dispatch the existing worker with its funded strategy and accepted human direction."""
        return self.controller.run_once(program_id, assignment_id, allocation_id=allocation_id,
                                        settings=settings, instructions=instructions, retry_of=retry_of)

    def _worker_contract(self, program_id, assignment_id, instructions):
        board = self.strategy.reconcile(program_id)
        assignment = self.store.get_record(program_id, assignment_id, 'assignment')
        route = next(r for r in board['routes'] if r['id'] == assignment['details']['approach_id'])
        if route['review_required'] or route['futile']:
            raise ProgramError('Review dependencies or reframe the exhausted route before dispatch')
        records = self.strategy.records(program_id)
        investigation = [r for r in records.values() if r['kind'] == 'investigation' and r['details']['assignment_id'] == assignment_id]
        contract = {'investigations': investigation, 'route_plan': route['plan'],
                    'human_inputs': board['human_inputs']}
        if not isinstance(instructions, str):
            raise ProgramError('instructions must be a string')
        if assignment['details']['execution_state'] != 'ready':
            raise ProgramError('Dispatch requires a ready assignment')
        return 'Recorded strategy contract:\n' + _json(contract) + '\n\n' + instructions

    def _launch_worker(self, program_id, assignment_id, *, settings=None, instructions='', runtime=None):
        instructions = self._worker_contract(program_id, assignment_id, instructions)
        return self.workers.start(program_id, assignment_id, settings=settings, instructions=instructions, runtime=runtime)

    def steering(self, program_id, text, *, subject_id=None, evidence_ids=None, actor='human',
                 record_id=None, expected_revision=None, intent='note', links=None, provenance=None,
                 next_assignment_id=None, allocation=None):
        """Capture human provenance; explicit instructions change the next pending allocation."""
        result = self.strategy.steering(program_id, text, subject_id=subject_id, evidence_ids=evidence_ids,
                                     actor=actor, record_id=record_id, expected_revision=expected_revision,
                                     intent=intent, links=links, provenance=provenance,
                                     next_assignment_id=next_assignment_id, allocation=allocation)
        if intent == 'instruction':
            self.controller.steered(result['accepted_input']['id'])
        return result

    def decision(self, program_id, text, *, subject_id=None, evidence_ids=None, actor='manager',
                 record_id=None, expected_revision=None):
        return self.store.record_note(program_id, 'decision', text, subject_id=subject_id, evidence_ids=evidence_ids,
                                      actor=actor, record_id=record_id, expected_revision=expected_revision)

    def _cancel_owned(self):
        for identity in self.child_ids(self.workers, 'attempt'):
            record = self.workers.status(self.program_id, identity)
            if record['details']['state'] in ACTIVE:
                try:
                    self.controller.cancel_attempt(self.program_id, identity)
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
        self.controller.stop()
        self._cancel_owned()
        return self.overview(program_id)

    def shutdown(self, program_id):
        """Retire this tool host after owned children exit; program history remains."""
        self.stopping = True
        self.controller.stop()
        self._cancel_owned()
        return self.overview(program_id)

    def drained(self):
        return all(r['details']['state'] not in ('preparing', 'prepared', 'checking', 'ownership_lost')
                   for r in self.store.snapshot(self.program_id)['records']
                   if r['kind'] == 'integration_job' and r['details']['owner'] == self.integrations.owner) and all(
                   dispatcher.children[i].poll() is not None for dispatcher, kind in
                   ((self.workers, 'attempt'), (self.reviews, 'review_run'),
                    (self.integrations, 'integration_job')) for i in self.child_ids(dispatcher, kind))
