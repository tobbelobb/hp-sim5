"""Finite manager-selected assignments, durable reservations and honest recovery.

The host owns scheduling and clocks. The existing launcher owns physics cleanup.
Same-UID callers can edit the store; historical PIDs confer no process authority.
"""
import json
import math
from pathlib import Path
import time
from urllib.parse import urlparse

from .program import ProgramError, _json, _now, _text
from .program_workers import ACTIVE, WorkerDispatcher, process_identity, process_live

STATES = ('active', 'exhausted', 'expired', 'stopped', 'interrupted')


def number(value, label, *, positive=True, integer=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or
            value < 0 or (positive and value == 0) or (integer and type(value) is not int)):
        raise ProgramError(label + ' must be a finite ' + ('positive' if positive else 'nonnegative') + (' integer' if integer else ' number'))
    return value


def validate_allocations(records):
    for r in records.values():
        if r['kind'] != 'runtime_allocation':
            continue
        d = r['details']
        if d.get('state') not in STATES:
            raise ProgramError('Invalid runtime allocation state')
        for field in ('task_limit', 'retry_limit', 'review_backlog_limit'):
            number(d.get(field), field, integer=True)
        for field in ('wall_seconds', 'review_seconds', 'deadline', 'cleanup_deadline', 'lease_until', 'cleanup_seconds', 'wall_reserved_seconds'):
            number(d.get(field), field, positive=field not in ('cleanup_seconds', 'wall_reserved_seconds', 'lease_until'))
        if (not isinstance(d.get('owner'), dict) or not isinstance(d.get('events'), list) or
                not isinstance(d.get('queue'), list) or len(d['queue']) + len(d.get('reservations', [])) > d['task_limit']):
            raise ProgramError('Invalid runtime owner, queue or event history')
        if abs(d['cleanup_deadline'] - d['deadline'] - d['cleanup_seconds']) > 1e-5:
            raise ProgramError('Runtime cleanup deadline differs from its allowance')
        if not isinstance(d.get('reservations'), list) or len(d['reservations']) > d['task_limit']:
            raise ProgramError('Invalid runtime task reservations')
        seen = set()
        for reservation in d['reservations']:
            assignment = records.get(reservation.get('assignment_id'))
            if assignment is None or assignment['kind'] != 'assignment' or assignment['id'] in seen:
                raise ProgramError('Runtime allocation requires distinct assignments')
            seen.add(assignment['id'])
            number(reservation.get('worker_seconds'), 'worker reservation', positive=False)
            number(reservation.get('review_seconds'), 'review reservation')
            if not isinstance(reservation.get('reviews'), list):
                raise ProgramError('Invalid reserved review history')
            if reservation.get('attempt_id'):
                attempt = records.get(reservation['attempt_id'])
                if attempt is None or attempt['kind'] != 'attempt' or attempt['details']['assignment_id'] != assignment['id']:
                    raise ProgramError('Runtime reservation differs from its attempt assignment')
            for review in reservation['reviews']:
                number(review.get('reserved_seconds'), 'reserved review seconds')
                if review.get('run_id') and (review['run_id'] not in records or records[review['run_id']]['kind'] != 'review_run'):
                    raise ProgramError('Missing runtime review run')
            if sum(r['reserved_seconds'] for r in reservation['reviews']) > reservation['review_seconds'] + 1e-6:
                raise ProgramError('Runtime review reservations exceed capacity')
        total = sum(r['worker_seconds'] + r['review_seconds'] for r in d['reservations'])
        if total > d['wall_seconds'] + 1e-6 or abs(d['wall_reserved_seconds'] - total) > 1e-6:
            raise ProgramError('Runtime wall reservations exceed allocation')


class ProgramController:
    def __init__(self, manager):
        self.manager, self.store, self.program_id = manager, manager.store, manager.program_id
        self.owner = {'id': manager.controller_id, 'process': process_identity(__import__('os').getpid())}
        self.deadlines = {}
        self.recover(self.program_id)

    def records(self):
        return self.store.snapshot(self.program_id)['records']

    def save(self, record):
        with self.store.transaction():
            self.store._touch(self.program_id)
            self.store.connection.execute('UPDATE records SET details_json=?, revision=revision+1, updated_at=? WHERE id=?',
                                          (_json(record['details']), _now(), record['id']))
        return self.store.get_record(self.program_id, record['id'])

    def allocate(self, program_id, task_limit, wall_minutes, *, review_minutes=5, cleanup_seconds=190,
                 retry_limit=1, review_backlog_limit=1):
        """Fund finite tasks and a separate review allowance per task; cleanup is additional wall time."""
        d = self._funding(program_id, task_limit, wall_minutes, review_minutes, cleanup_seconds, retry_limit, review_backlog_limit)
        with self.store.transaction():
            self.store._touch(program_id)
            r = self.store._insert(program_id, 'runtime_allocation', 'Finite manager allocation', d)
        self.deadlines[r['id']] = time.monotonic() + wall_minutes * 60
        return r

    def _funding(self, program_id, task_limit, wall_minutes, review_minutes, cleanup_seconds, retry_limit, review_backlog_limit):
        for label, value in (('task_limit', task_limit), ('retry_limit', retry_limit), ('review_backlog_limit', review_backlog_limit)):
            number(value, label, integer=True)
        number(wall_minutes, 'wall_minutes')
        number(review_minutes, 'review_minutes')
        number(cleanup_seconds, 'cleanup_seconds', positive=False)
        if review_minutes * task_limit >= wall_minutes:
            raise ProgramError('Allocation must leave worker time after all task review reservations')
        if self.store.get_program(program_id)['state'] != 'active':
            raise ProgramError('Funding requires an active program')
        if any(r['kind'] == 'runtime_allocation' and r['details']['state'] == 'active' for r in self.records()):
            raise ProgramError('Use the current finite allocation before funding another')
        now = time.time()
        d = dict(state='active', task_limit=task_limit, wall_seconds=wall_minutes * 60,
                 review_seconds=review_minutes * 60, cleanup_seconds=cleanup_seconds,
                 retry_limit=retry_limit, review_backlog_limit=review_backlog_limit, owner=self.owner,
                 started_at=_now(), deadline=now + wall_minutes * 60,
                 cleanup_deadline=now + wall_minutes * 60 + cleanup_seconds,
                 lease_until=now + 15, reservations=[], queue=[], events=[], wall_reserved_seconds=0)
        return d

    def current(self, allocation_id):
        return self.store.get_record(self.program_id, allocation_id, 'runtime_allocation')

    def locate(self, *, attempt_id=None, submission_id=None, review_id=None):
        records = self.records()
        if submission_id:
            attempt_id = self.store.get_record(self.program_id, submission_id, 'submission')['details']['manifest']['attempt_id']
        for a in reversed(records):
            if a['kind'] == 'runtime_allocation':
                for reservation in a['details']['reservations']:
                    if ((attempt_id and reservation.get('attempt_id') == attempt_id) or
                            (review_id and any(r.get('run_id') == review_id for r in reservation['reviews']))):
                        return a, reservation
        return None, None

    def _ready(self, allocation, assignment_id):
        d = allocation['details']
        if self.store.get_program(self.program_id)['state'] != 'active' or d['state'] != 'active':
            raise ProgramError('Dispatch requires an active program and runtime allocation')
        if d['owner'] != self.owner or time.time() >= d['lease_until']:
            raise ProgramError('Allocation lease is stale or belongs to another controller; recover first')
        if time.time() >= d['deadline'] or time.monotonic() >= self.deadlines.get(allocation['id'], float('inf')):
            raise ProgramError('Runtime wall allocation expired')
        if len(d['reservations']) >= d['task_limit']:
            raise ProgramError('Runtime task allocation exhausted')
        records = self.records()
        if any(r['kind'] == 'runtime_allocation' and any(x['assignment_id'] == assignment_id
               for x in r['details']['reservations']) for r in records):
            raise ProgramError('An assignment is reserved once; create a distinct bounded handoff')
        if any(r['kind'] == 'runtime_allocation' and any(not x.get('attempt_id') and
               x['state'] in ('starting', 'interrupted') and not x.get('handoff') for x in r['details']['reservations'])
               for r in records):
            raise ProgramError('An interrupted startup reservation needs an explicit allocation handoff')
        if any(r['kind'] == 'attempt' and (r['details']['state'] in ACTIVE or
               (self.unknown_cleanup(r['details']) and not r['details'].get('handoff'))) for r in records):
            raise ProgramError('Worker slot is occupied or interrupted ownership needs an explicit bounded handoff')
        if sum(x.get('review_pending', False) and not x.get('superseded_by') for r in records if r['kind'] == 'runtime_allocation'
                                        for x in r['details']['reservations']) >= d['review_backlog_limit']:
            raise ProgramError('Review backlog limit reached; complete the reserved separate review first')
        if any(r['kind'] == 'attempt' and r['details']['assignment_id'] == assignment_id for r in records):
            raise ProgramError('An assignment is dispatched once; create a distinct bounded handoff for a retry')
        assignment = self.store.get_record(self.program_id, assignment_id, 'assignment')
        seconds = assignment['details']['allocation']['wall_minutes'] * 60
        total = seconds + d['review_seconds']
        if (d['wall_reserved_seconds'] + total > d['wall_seconds'] + 1e-6 or
                time.time() + seconds + d['review_seconds'] > d['deadline']):
            raise ProgramError('Insufficient wall allocation after preserving review capacity')
        self.manager._worker_contract(self.program_id, assignment_id, '')
        return seconds

    def run_once(self, program_id, assignment_id, *, allocation_id=None, settings=None, instructions='', retry_of=None):
        """Dispatch one distinct assignment against finite task/wall/review reservations."""
        WorkerDispatcher._settings(settings)
        if not isinstance(instructions, str):
            raise ProgramError('instructions must be a string')
        if allocation_id is None:
            active = [r for r in self.records() if r['kind'] == 'runtime_allocation' and r['details']['state'] == 'active']
            if active:
                allocation_id = active[-1]['id']
            else:
                minutes = self.store.get_record(program_id, assignment_id, 'assignment')['details']['allocation']['wall_minutes']
                allocation_id = self.allocate(program_id, 1, minutes + 5 + .01)['id']
        a = self.current(allocation_id)
        seconds = self._ready(a, assignment_id)
        retry_root = None
        if retry_of:
            prior = self.store.get_record(program_id, retry_of, 'attempt')
            _, previous = self.locate(attempt_id=retry_of)
            if previous is None or prior['details']['state'] not in ('failed', 'launch_failed', 'cancelled', 'interrupted'):
                raise ProgramError('Retry requires an ended unsuccessful runtime attempt')
            retry_root = previous.get('retry_root') or retry_of
            count = sum(x.get('attempt_id') == retry_root or x.get('retry_root') == retry_root
                        for r in self.records() if r['kind'] == 'runtime_allocation' for x in r['details']['reservations'])
            if count >= a['details']['retry_limit']:
                raise ProgramError('Worker retry limit reached; reframe or explicitly fund a larger bounded attempt limit')
        # Reserve before Popen. A crash in this gap retains a charged uncertain start.
        reservation = dict(assignment_id=assignment_id, worker_seconds=seconds, review_seconds=a['details']['review_seconds'],
                           attempt_id=None, state='starting', reviews=[], review_pending=False,
                           charged_at=_now(), usage={'estimates': self.store.get_record(program_id, assignment_id)['details']['allocation'],
                                                    'worker_reports': [], 'supervisor_observations': [],
                                                    'unmetered': ['shell movement', 'shell compute', 'spend']})
        reservation['retry_root'] = retry_root
        a['details']['reservations'].append(reservation)
        a['details']['wall_reserved_seconds'] += seconds + reservation['review_seconds']
        self.save(a)
        runtime = {'allocation_id': a['id'], 'assignment_id': assignment_id,
                   'wall_deadline': min(time.time() + seconds, a['details']['deadline'] - reservation['review_seconds']),
                   'cleanup_seconds': a['details']['cleanup_seconds']}
        try:
            result = self.manager._launch_worker(program_id, assignment_id, settings=settings, instructions=instructions, runtime=runtime)
            reservation.update(attempt_id=result['id'], state=result['details']['state'])
        except BaseException:
            attempts = [r for r in self.records() if r['kind'] == 'attempt' and r['details'].get('runtime', {}).get('allocation_id') == a['id']
                        and r['details']['assignment_id'] == assignment_id]
            reservation.update(attempt_id=attempts[-1]['id'] if attempts else None, state='interrupted')
            self.save(a)
            raise
        self.save(a)
        return result

    def run_within_allocation(self, program_id, allocation_id, assignment_ids, *, settings=None):
        """Queue the manager's finite ordered selection; wait at the review backlog boundary."""
        if not isinstance(assignment_ids, list) or not assignment_ids or len(set(assignment_ids)) != len(assignment_ids):
            raise ProgramError('Select a nonempty list of distinct assignment IDs')
        a = self.current(allocation_id)
        if a['details']['queue']:
            raise ProgramError('An allocation already has a manager-selected queue')
        self._ready(a, assignment_ids[0])
        available = a['details']['task_limit'] - len(a['details']['reservations'])
        if len(assignment_ids) > available:
            raise ProgramError('Selected queue exceeds remaining task allocation')
        total = sum(self.store.get_record(program_id, i, 'assignment')['details']['allocation']['wall_minutes'] * 60 +
                    a['details']['review_seconds'] for i in assignment_ids)
        if a['details']['wall_reserved_seconds'] + total > a['details']['wall_seconds']:
            raise ProgramError('Selected queue exceeds remaining wall/review allocation')
        self.manager.workers._settings(settings)
        a['details']['queue'] = [{'assignment_id': i, 'settings': settings} for i in assignment_ids]
        self.save(a)
        self.poll()
        return self.status(program_id)

    def review_start(self, program_id, submission_id, **settings):
        """Spend reserved review capacity independently of the worker concurrency slot."""
        a, reservation = self.locate(submission_id=submission_id)
        if a is None:
            self.fund_review(program_id, submission_id, wall_minutes=settings.get('wall_minutes', 5))
            a, reservation = self.locate(submission_id=submission_id)
        d = a['details']
        if (self.store.get_program(program_id)['state'] != 'active' or d['state'] in ('stopped', 'expired', 'interrupted') or
                d['owner'] != self.owner or time.time() >= d['deadline']):
            raise ProgramError('Review reservation is stopped, interrupted or expired; retain evidence for a new allocation')
        if any(r['kind'] == 'review_run' and r['details']['state'] in ('running', 'cancelling') for r in self.records()):
            raise ProgramError('The separate review concurrency slot is occupied')
        if len(reservation['reviews']) >= d['retry_limit']:
            raise ProgramError('Review retry limit reached; fund an explicit new bounded handoff')
        remaining = reservation['review_seconds'] - sum(r['reserved_seconds'] for r in reservation['reviews'])
        minutes = settings.get('wall_minutes', min(5, remaining / 60))
        number(minutes, 'review wall_minutes')
        if minutes * 60 > remaining + 1e-6 or time.time() + minutes * 60 > d['deadline']:
            raise ProgramError('Review exceeds its reserved wall capacity')
        self.manager.reviews.plan(program_id, submission_id, **dict(settings, wall_minutes=minutes))
        entry = {'run_id': None, 'reserved_seconds': minutes * 60, 'state': 'starting'}
        reservation['reviews'].append(entry)
        reservation['review_pending'] = True
        self.save(a)
        try:
            result = self.manager.reviews.start(program_id, submission_id, **dict(settings, wall_minutes=minutes))
            entry.update(run_id=result['id'], state=result['details']['state'])
        except BaseException:
            entry['state'] = 'interrupted'
            self.save(a)
            raise
        self.save(a)
        return result

    def fund_review(self, program_id, submission_id, *, wall_minutes=5, retry_limit=1, cleanup_seconds=190):
        """Explicitly fund retained evidence when its previous allocation has ended."""
        submission, _ = self.manager.reviews.verify(program_id, submission_id)
        attempt_id = submission['details']['manifest']['attempt_id']
        old, _ = self.locate(attempt_id=attempt_id)
        if old and old['details']['state'] in ('active', 'exhausted'):
            raise ProgramError('Use the existing review reservation before funding another')
        d = self._funding(program_id, 1, wall_minutes + .01, wall_minutes, cleanup_seconds, retry_limit, 1)
        d.update(state='exhausted', wall_reserved_seconds=wall_minutes * 60)
        d['reservations'].append(dict(assignment_id=submission['details']['manifest']['assignment_id'],
            worker_seconds=0, review_seconds=wall_minutes * 60, attempt_id=attempt_id, state='submitted',
            reviews=[], review_pending=True, charged_at=_now(), usage={'estimates': {}, 'worker_reports': [],
            'supervisor_observations': [], 'unmetered': ['shell movement', 'shell compute', 'spend']}))
        # Funding and every predecessor transfer commit together, including when
        # older interrupted allocations already contain this same obligation.
        with self.store.transaction():
            self.store._touch(program_id)
            result = self.store._insert(program_id, 'runtime_allocation', 'Reserved saved-evidence review', d)
            for previous in self.records():
                if previous['kind'] != 'runtime_allocation' or previous['id'] == result['id']:
                    continue
                changed = False
                for reservation in previous['details']['reservations']:
                    if reservation.get('attempt_id') == attempt_id:
                        reservation['superseded_by'] = result['id']
                        changed = True
                if changed:
                    self.store.connection.execute('UPDATE records SET details_json=?, revision=revision+1, updated_at=? WHERE id=?',
                                                  (_json(previous['details']), _now(), previous['id']))
        self.deadlines[result['id']] = time.monotonic() + wall_minutes * 60
        return result

    def review_apply(self, program_id, review_id):
        review = self.store.get_record(program_id, review_id, 'review')
        a, reservation = self.locate(review_id=review['details']['run_id'])
        if self.store.get_program(program_id)['state'] == 'stopped' or (a and
                (a['details']['state'] in ('stopped', 'expired', 'interrupted') or time.time() >= a['details']['deadline'])):
            raise ProgramError('Late review retained; a stopped allocation cannot admit it')
        result = self.manager.reviews.apply(program_id, review_id)
        if a:
            reservation['review_pending'] = False
            reservation['admission_id'] = result['id']
            self.save(a)
        return result

    def usage(self, program_id, attempt_id, report):
        """Retain a worker-reported usage value without treating it as a measured tool count."""
        from .program import _object
        _object(report, 'worker usage report')
        a, reservation = self.locate(attempt_id=attempt_id)
        if a is None:
            raise ProgramError('Attempt has no runtime reservation')
        reservation['usage']['worker_reports'].append({'at': _now(), 'report': report, 'source': 'worker_report'})
        return self.save(a)

    def cancel_attempt(self, program_id, attempt_id):
        """Cancel an owned worker through its existing launcher's cleanup path."""
        a, _ = self.locate(attempt_id=attempt_id)
        return self.manager.workers.cancel(program_id, attempt_id, grace_seconds=a['details']['cleanup_seconds'] if a else 190)

    def stop(self):
        for a in self.records():
            if a['kind'] == 'runtime_allocation' and a['details']['state'] in ('active', 'exhausted'):
                a['details'].update(state='stopped', queue=[])
                self.save(a)

    def stop_allocation(self, program_id, allocation_id, reason):
        """End one finite grant and cancel its owned work, while the research program remains open."""
        _text(reason, 'reason')
        a = self.current(allocation_id)
        a['details'].update(state='stopped', queue=[])
        a['details']['events'].append({'at': _now(), 'event': 'allocation_stopped', 'reason': reason})
        self.save(a)
        self.poll()
        return self.current(allocation_id)

    def steered(self, input_id):
        for a in self.records():
            if a['kind'] == 'runtime_allocation' and a['details']['queue']:
                a['details']['queue'] = []
                a['details']['events'].append({'at': _now(), 'event': 'queue_held_for_human_instruction', 'input_id': input_id})
                self.save(a)

    def poll(self):
        now = time.time()
        records = {r['id']: r for r in self.records()}
        for a in records.values():
            if a['kind'] != 'runtime_allocation' or a['details']['owner'] != self.owner:
                continue
            d, before = a['details'], _json(a['details'])
            if d['state'] in ('active', 'exhausted') and (now >= d['deadline'] or
                    time.monotonic() >= self.deadlines.get(a['id'], float('inf'))):
                d.update(state='expired', queue=[])
            if d['state'] in ('active', 'exhausted') and now + 10 >= d['lease_until']:
                d['lease_until'] = now + 15
            for reservation in d['reservations']:
                attempt = records.get(reservation.get('attempt_id'))
                if attempt:
                    ad = attempt['details']
                    reservation['state'] = ad['state']
                    reservation['usage']['observed_worker_wall_seconds'] = ad.get('observed_wall_seconds', 0)
                    if ad['state'] == 'submitted' and not reservation.get('admission_id') and not reservation.get('superseded_by'):
                        reservation['review_pending'] = True
                    if d['state'] in ('expired', 'stopped') and ad['state'] in ('running', 'starting') and attempt['id'] in self.manager.workers.children:
                        self.manager.workers.cancel(self.program_id, attempt['id'], grace_seconds=d['cleanup_seconds'])
                for review in reservation['reviews']:
                    run = records.get(review.get('run_id'))
                    if run:
                        review['state'] = run['details']['state']
                        if d['state'] in ('expired', 'stopped') and run['details']['state'] in ('running', 'cancelling') and run['id'] in self.manager.reviews.children:
                            try:
                                self.manager.reviews.cancel(self.program_id, run['id'])
                            except ProgramError as error:
                                run['details'].update(state='interrupted', error=str(error) + '; reviewer shutdown unknown')
                                self.manager.reviews._save(self.program_id, run['id'], run['details'])
                        # Admission commits before the convenience reservation
                        # cache. Recover that cache if the host died in between.
                        admitted = next((r for r in records.values() if r['kind'] == 'admission' and
                            records[r['details']['review_id']]['details']['run_id'] == run['id']), None)
                        if admitted:
                            reservation.update(review_pending=False, admission_id=admitted['id'])
            if len(d['reservations']) == d['task_limit'] and d['state'] == 'active':
                d['state'] = 'exhausted'
            if _json(d) != before:
                # Heartbeats do not invalidate the manager's semantic program revision.
                previous = json.loads(before)
                previous['lease_until'] = d['lease_until']
                if _json(previous) == _json(d):
                    self.store.connection.execute('UPDATE records SET details_json=? WHERE id=?', (_json(d), a['id']))
                else:
                    self.save(a)
            if d['state'] == 'active' and d['queue'] and self.store.get_program(self.program_id)['state'] == 'active':
                next_item = d['queue'][0]
                try:
                    self._ready(a, next_item['assignment_id'])
                except ProgramError:
                    continue
                # Persist dequeue before dispatch. Restart never replays this instruction.
                d['queue'].pop(0)
                self.save(a)
                self.run_once(self.program_id, next_item['assignment_id'], allocation_id=a['id'], settings=next_item['settings'])

    def status(self, program_id):
        return {'allocations': [r for r in self.records() if r['kind'] == 'runtime_allocation'],
                'enforced': ['finite dispatch/task reservations', 'worker concurrency 1', 'review concurrency 1',
                             'review backlog', 'owned launcher/reviewer wall deadlines independent of host polling'],
                'cleanup': 'Launcher cleanup grace is separate; forced exit or missing manifest leaves physical/recording shutdown unverified',
                'workload_limits': 'Production tool per-job limits apply only to supported tool calls. Allocation counters are estimates/contracts, not hard aggregate movement or spend caps',
                'restart': 'No PID adoption. Unverified work is interrupted and requires an explicit new bounded handoff'}

    def recover(self, program_id, *, handoff_attempt_id=None, handoff_allocation_id=None, reason=None):
        """Reconcile leases, birth identities and live supervisor observations without PID adoption."""
        outcomes = []
        for r in self.records():
            d, kind = r['details'], r['kind']
            if kind == 'attempt' and d['state'] in ACTIVE and r['id'] in self.manager.workers.children:
                verified = self.manager.workers._owned(r['id'], d) and process_live(d.get('process'))
                observation = self.inspect_supervisor(d) if verified else {'state': 'unknown', 'reason': 'Launcher ownership unverified'}
                outcomes.append({'attempt_id': r['id'], 'state': 'retained_owned_launcher' if verified else 'ownership_unknown',
                                 'supervisor': observation, 'adopted': False,
                                 'session': d['session_dir'], 'connection': str(Path(d['session_dir']) / 'connection.json')})
                if observation.get('state') == 'live_observed':
                    allocation, reservation = self.locate(attempt_id=r['id'])
                    if allocation:
                        reservation['usage']['supervisor_observations'].append({'at': _now(), 'source': 'authenticated_supervisor_status',
                                                                               'observation': observation})
                        self.save(allocation)
            elif kind == 'attempt' and d['state'] in ACTIVE:
                owner_live = process_live(d['owner'].get('process'))
                if owner_live and d['owner'].get('process') != self.owner['process']:
                    outcomes.append({'attempt_id': r['id'], 'state': 'other_live_owner', 'signalled': False})
                    continue
                self.manager.workers._inventory(d)
                d.update(state='interrupted', cleanup='unknown; controller ownership lost',
                         recovery={'at': _now(), 'launcher_birth_matches': process_live(d.get('process')),
                                   'owner_live': owner_live, 'supervisor': self.inspect_supervisor(d), 'adopted': False})
                self.manager.workers._save(program_id, r['id'], d, assignment_state='failed')
                outcomes.append({'attempt_id': r['id'], **d['recovery']})
            elif kind == 'review_run' and d['state'] in ('running', 'cancelling') and r['id'] not in self.manager.reviews.children:
                if not process_live(d['owner'].get('process')) or d['owner'].get('process') == self.owner['process']:
                    d.update(state='interrupted', error='Controller ownership lost; no review admitted')
                    self.manager.reviews._save(program_id, r['id'], d)
            elif kind == 'runtime_allocation' and d['state'] in ('active', 'exhausted') and d['owner'] != self.owner:
                if not process_live(d['owner'].get('process')) or d['owner'].get('process') == self.owner['process']:
                    d.update(state='interrupted', queue=[])
                    d['events'].append({'at': _now(), 'event': 'controller_restart', 'previous_lease_until': d['lease_until']})
                    self.save(r)
        if handoff_attempt_id:
            _text(reason, 'handoff reason acknowledging unknown physical state')
            r = self.store.get_record(program_id, handoff_attempt_id, 'attempt')
            if r['details']['state'] in ACTIVE or not self.unknown_cleanup(r['details']):
                raise ProgramError('Only ended/interrupted unknown ownership can receive a bounded handoff')
            r['details']['handoff'] = {'reason': reason, 'at': _now(), 'physical_shutdown': 'unknown',
                                       'instruction': 'Use a new distinct bounded assignment; no world checkpoint or conversation was resumed'}
            self.save(r)
        if handoff_allocation_id:
            _text(reason, 'handoff reason acknowledging unknown startup')
            r = self.current(handoff_allocation_id)
            if r['details']['state'] != 'interrupted':
                raise ProgramError('Only an interrupted allocation can receive a startup handoff')
            for reservation in r['details']['reservations']:
                if reservation.get('attempt_id') is None:
                    reservation.update(state='interrupted', handoff={'reason': reason, 'at': _now(), 'physical_shutdown': 'unknown'})
            self.save(r)
        return {'reconciled': outcomes, 'runtime': self.status(program_id)}

    @staticmethod
    def unknown_cleanup(details):
        return (details['state'] == 'interrupted' or details['cleanup'].startswith(('failed', 'unverified', 'unknown')))

    @staticmethod
    def inspect_supervisor(details):
        session = Path(details['session_dir'])
        try:
            connection = json.loads((session / 'connection.json').read_text())
            launch = json.loads((session / 'launch.json').read_text())
            manifest = json.loads((session / 'session.json').read_text())
            if (Path(manifest['session_dir']).resolve() != session.resolve() or
                    Path(connection['repo']).resolve() != Path(manifest['repo']).resolve() or
                    connection['runtime_endpoint'] != launch['runtime_endpoint'] or
                    connection['session_id'] != launch['runtime_status']['session_id']):
                raise ValueError('Session and connection identities differ')
            endpoint = urlparse(connection['runtime_endpoint'])
            if endpoint.scheme != 'http' or endpoint.hostname not in ('127.0.0.1', 'localhost'):
                raise ValueError('Only the existing local supervisor can be inspected')
            from .services import request
            status = request(connection['runtime_endpoint'], 'status', token=connection['runtime_token'], timeout=.25)
            return {'state': 'live_observed', 'session_id': status['session_id'],
                    'world_matches_saved': status['session_id'] == connection['session_id'],
                    'observation': status, 'adopted': False}
        except (OSError, ValueError, KeyError, RuntimeError) as error:
            return {'state': 'unknown', 'reason': str(error), 'adopted': False}
