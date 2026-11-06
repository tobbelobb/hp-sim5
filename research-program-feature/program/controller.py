"""Worker capacity and recovery, without experiment planning or accounting."""
from .store import ProgramError, _now, _text
from .workers import ACTIVE, process_live
from . import pool


class ProgramController:
    def __init__(self, manager):
        self.manager, self.store = manager, manager.store
        self.recover(manager.program_id)

    def configure_pool(self, program_id, worker_limit, reason):
        """Set store-wide worker concurrency, from one through four."""
        if type(worker_limit) is not int or not 1 <= worker_limit <= pool.MAX_WORKERS:
            raise ProgramError('worker_limit must be an integer from 1 through 4')
        _text(reason, 'reason')
        with self.store.transaction():
            workers, _, _ = pool.occupancy(self.store)
            if len(workers) > worker_limit or (worker_limit > 1 and any(
                    not r['details'].get('settings', {}).get('worktree') for r in workers)):
                raise ProgramError('Drain workers before changing to incompatible capacity')
            self.store.connection.execute('UPDATE worker_pool SET worker_limit=? WHERE singleton=1', (worker_limit,))
            self.store._touch(program_id)
        return {'worker_limit': worker_limit}

    def recover(self, program_id, *, handoff_attempt_id=None, handoff_review_run_id=None, reason=None):
        """Observe lost owners; explicitly acknowledge cleanup without adopting saved PIDs."""
        outcomes = []
        for r in self.store.snapshot(program_id)['records']:
            d = r['details']
            if r['kind'] == 'attempt' and d['state'] in ACTIVE and r['id'] not in self.manager.workers.children:
                if process_live(d['owner'].get('process')) and d['owner'] != self.manager.workers.owner:
                    continue
                d.update(state='interrupted', cleanup='unknown; controller ownership lost',
                         recovery={'at': _now(), 'adopted': False, 'signalled': False})
                self.manager.workers._save(program_id, r['id'], d, assignment_state='failed')
                outcomes.append({'attempt_id': r['id'], **d['recovery']})
            elif r['kind'] == 'review_run' and d['state'] in ('running', 'cancelling') and r['id'] not in self.manager.reviews.children:
                if not process_live(d['owner'].get('process')) or d['owner'] == self.manager.reviews.owner:
                    d.update(state='interrupted', error='Controller ownership lost', cleanup='unknown; reviewer may remain live')
                    self.manager.reviews._save(program_id, r['id'], d)
        if handoff_attempt_id:
            _text(reason, 'handoff reason')
            r = self.store.get_record(program_id, handoff_attempt_id, 'attempt')
            d = r['details']
            if d['state'] in ACTIVE or process_live(d.get('process')):
                raise ProgramError('A live worker cannot be handed off')
            d['handoff'] = {'at': _now(), 'reason': reason, 'adopted': False, 'signalled': False}
            self.manager.workers._save(program_id, r['id'], d)
        if handoff_review_run_id:
            _text(reason, 'handoff reason')
            r = self.store.get_record(program_id, handoff_review_run_id, 'review_run')
            d = r['details']
            if d['state'] in ('running', 'cancelling') or process_live(d.get('process')):
                raise ProgramError('A live reviewer cannot be handed off')
            d['handoff'] = {'at': _now(), 'reason': reason, 'adopted': False, 'signalled': False}
            self.manager.reviews._save(program_id, r['id'], d)
        return {'reconciled': outcomes}
