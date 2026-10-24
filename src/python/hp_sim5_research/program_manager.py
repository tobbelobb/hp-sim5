"""Program-bound chat tools. Physics stays in the existing worker launcher."""
import inspect
from pathlib import Path
import sqlite3
import subprocess
import traceback

from .program import ProgramError, _json, _object, render_markdown
from .program_workers import ACTIVE


UNAVAILABLE = {
    'autonomous_continue': 'Task 013: autonomous dispatch, runtime reservations and worker wall deadlines',
    'recover': 'Task 013: restart reconciliation/adoption of unfinished children',
    'strategy': 'Task 012: evidence-based strategy selection and automatic steering application',
    'parallel': 'Task 014: coordinated worker pool',
    'integrate': 'Task 015: combined candidate integration and delivery checks',
}


def failure(code, message):
    return {'ok': False, 'error': {'code': code, 'message': message}}


class ProgramManager:
    """Retained dispatcher and separate reviewer for exactly one program.

    This is a tool owner, not the autonomous allocation/recovery controller. All
    calls run on its host thread; SQLite connections never cross threads.
    """

    def __init__(self, store, root, program_id, controller_id, session, *, workers=None, reviews=None):
        store.get_program(program_id)
        self.store, self.root, self.program_id = store, Path(root).resolve(), program_id
        self.controller_id, self.session = controller_id, Path(session)
        self.session.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.workers = workers or store.workers(self.root)
        self.reviews = reviews or store.reviews(self.root)
        self.stopping = False
        self.operations = {
            'overview': self.overview, 'status': self.overview, 'capabilities': self.capabilities,
            'query': store.graph().query, 'question': store.add_question, 'approach': store.add_approach,
            'assignment': store.add_assignment, 'artifact': store.add_artifact,
            'claim': store.graph().add_claim, 'derive': store.graph().add_derivation,
            'relate': store.graph().add_relation, 'assess': store.graph().add_assessment,
            'review_event': store.graph().review_event, 'transition': self.transition,
            'steering': self.steering, 'decision': self.decision,
            'worker_start': self.workers.start, 'worker_status': self.workers.status,
            'worker_cancel': self.workers.cancel, 'worker_sessions': self.workers.sessions,
            'submit': self.reviews.submit, 'submission_verify': self.reviews.verify,
            'submission_invalidate': self.reviews.invalidate, 'review_plan': self.reviews.plan,
            'review_start': self.reviews.start, 'review_status': self.reviews.status,
            'review_cancel': self.reviews.cancel, 'review_apply': self.reviews.apply,
            'pause': self.pause, 'resume': self.resume, 'stop': self.stop,
            'shutdown': self.shutdown,
        }

    def capabilities(self, program_id):
        operations = {}
        for name, method in self.operations.items():
            signature = inspect.signature(method)
            operations[name] = {
                'parameters': [str(p) for k, p in signature.parameters.items() if k != 'program_id'],
                'description': (inspect.getdoc(method) or '').split('\n')[0],
            }
        return {'operations': operations, 'unavailable': UNAVAILABLE,
                'physics_world': False, 'default_worker_concurrency': 1,
                'boundary': 'Cooperative same-UID local tools; private files are not adversarial isolation',
                'worker_allocation': 'Recorded contract only; independent worker wall enforcement is task 013',
                'restart': 'Ended history is readable; stale active workers are never adopted or signalled'}

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
                                                   'worker_sessions', 'review_status', 'shutdown'):
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
        for identity in tuple(self.workers.children):
            self.workers.status(self.program_id, identity)
        for identity in tuple(self.reviews.children):
            self.reviews.status(self.program_id, identity)

    def overview(self, program_id):
        self.poll()
        result = self.store.overview(program_id)
        pending = [r for r in result['records'] if r['kind'] == 'assignment'
                   and r['details']['execution_state'] in ('planned', 'ready')]
        result['next_allocation'] = ({'assignment_id': pending[0]['id'],
                                      'state': pending[0]['details']['execution_state'],
                                      'allocation': pending[0]['details']['allocation']}
                                     if pending else {'state': 'unallocated', 'allocation': None,
                                                      'reason': 'Manager must select a bounded assignment from the question and evidence'})
        result['manager'] = {'program_id': program_id, 'controller_id': self.controller_id,
                             'stopping': self.stopping, 'physics_world': False,
                             'links': {'store': str(self.store.path), 'session': str(self.session),
                                       'overview': str(self.session / 'overview.md'),
                                       'snapshot': str(self.session / 'overview.json'),
                                       'worker_sessions': [r['details']['session_dir'] for r in result['records']
                                                           if r['kind'] == 'attempt']},
                             'unavailable': UNAVAILABLE}
        (self.session / 'overview.json').write_text(_json(result) + '\n')
        (self.session / 'overview.md').write_text(render_markdown(result))
        return result

    def transition(self, program_id, record_id, target, reason, *, expected_revision=None):
        if record_id == program_id:
            raise ProgramError('Use pause, resume or stop for program lifecycle controls')
        return self.store.transition(program_id, record_id, target, reason, expected_revision=expected_revision)

    def steering(self, program_id, text, *, subject_id=None, evidence_ids=None, actor='human',
                 record_id=None, expected_revision=None):
        """Record human steering; manager reads it and explicitly changes assignments."""
        return self.store.record_note(program_id, 'steering', text, subject_id=subject_id, evidence_ids=evidence_ids,
                                      actor=actor, record_id=record_id, expected_revision=expected_revision)

    def decision(self, program_id, text, *, subject_id=None, evidence_ids=None, actor='manager',
                 record_id=None, expected_revision=None):
        return self.store.record_note(program_id, 'decision', text, subject_id=subject_id, evidence_ids=evidence_ids,
                                      actor=actor, record_id=record_id, expected_revision=expected_revision)

    def _cancel_owned(self):
        for identity in tuple(self.workers.children):
            record = self.workers.status(self.program_id, identity)
            if record['details']['state'] in ACTIVE:
                self.workers.cancel(self.program_id, identity)
        for identity in tuple(self.reviews.children):
            self.reviews.cancel(self.program_id, identity)

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
        return all(p.poll() is not None for p in (*self.workers.children.values(), *self.reviews.children.values()))
