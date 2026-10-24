"""Store-wide cooperative capacity checks. Call reservations inside BEGIN IMMEDIATE."""
from .program import ProgramError

MAX_WORKERS = 4
ACTIVE = ('starting', 'running', 'cancelling', 'ownership_lost')
SHARED_BOUNDARY = ('Workers own separate worktrees, worlds and session artifacts. Output, RRF/build, '
                   'dependencies and the starting native addon are shared. Treat shared paths and '
                   'datasets as immutable; use only your session directory for new artifacts. '
                   'Shared builds, dependency changes and combined evaluation require an exclusive '
                   'shared_runtime_write reservation. Same-UID cooperation is not adversarial isolation.')


def records(store):
    return [store._decode(r) for r in store.connection.execute(
        "SELECT * FROM records WHERE kind IN ('attempt','runtime_allocation','review_run')")]


def limit(store):
    return store.connection.execute('SELECT worker_limit FROM worker_pool WHERE singleton=1').fetchone()[0]


def backlog_limit(store):
    return store.connection.execute('SELECT review_backlog_limit FROM worker_pool WHERE singleton=1').fetchone()[0]


def obligations(store):
    saved = records(store)
    attempts = {r['id']: r['details']['state'] for r in saved if r['kind'] == 'attempt'}
    return [(r, x) for r in saved if r['kind'] == 'runtime_allocation' for x in r['details']['reservations']
            if (x.get('review_pending') or attempts.get(x.get('attempt_id'), x['state']) in (*ACTIVE, 'submitted'))
            and not x.get('superseded_by') and not x.get('admission_id')]


def check_backlog(store, *, transfer_attempt=None):
    count = sum(x.get('attempt_id') != transfer_attempt or transfer_attempt is None for _, x in obligations(store))
    if count >= backlog_limit(store):
        raise ProgramError('Store-wide future review backlog limit reached; complete reserved review before dispatch')


def unknown(details):
    return details['state'] == 'interrupted' or details['cleanup'].startswith(('failed', 'unverified', 'unknown'))


def occupancy(store, *, exclude=None):
    """Include unopened programs and the charged gap before an attempt exists."""
    workers, reviews, uncertain = [], [], []
    saved = records(store)
    started = {(r['details'].get('runtime', {}).get('allocation_id'), r['details']['assignment_id'])
               for r in saved if r['kind'] == 'attempt'}
    for r in saved:
        d = r['details']
        if r['kind'] == 'attempt':
            if d['state'] in ACTIVE:
                workers.append(r)
            elif unknown(d) and not d.get('handoff'):
                uncertain.append(r)
        elif r['kind'] == 'review_run' and (d['state'] in ('running', 'cancelling') or
                (d.get('cleanup', '').startswith('unknown') and not d.get('handoff'))):
            reviews.append(r)
        elif r['kind'] == 'runtime_allocation':
            for reservation in d['reservations']:
                for review in reservation['reviews']:
                    if review.get('run_id') is None and review['state'] == 'starting':
                        reviews.append(dict(r, reservation=reservation))
                if reservation.get('attempt_id') or reservation.get('handoff'):
                    continue
                if (r['id'], reservation['assignment_id']) in started:
                    continue
                if (r['program_id'], reservation['assignment_id']) == exclude:
                    continue
                if reservation['state'] in ('starting', 'interrupted'):
                    workers.append(dict(r, reservation=reservation))
                    if reservation['state'] == 'interrupted' or d['state'] == 'interrupted':
                        uncertain.append(r)
    return workers, reviews, uncertain


def check_worker(store, program_id, assignment_id, *, program_limit=None, exclusive=False):
    check_integration(store)
    workers, reviews, uncertain = occupancy(store, exclude=(program_id, assignment_id))
    if uncertain:
        raise ProgramError('Interrupted ownership or unknown cleanup needs an explicit bounded handoff, including unopened programs')
    if len(workers) >= limit(store):
        raise ProgramError('A worker is already reserved; store-wide worker pool is full')
    if program_limit is not None and sum(r['program_id'] == program_id for r in workers) >= program_limit:
        raise ProgramError('Program worker pool is full')
    if (exclusive and (workers or reviews)) or any(
            r.get('reservation', r['details']).get('settings', {}).get('shared_runtime_write') for r in workers):
        raise ProgramError('Shared runtime writes require exclusive worker and reviewer capacity')


def check_review(store, *, reservation=None):
    check_integration(store)
    workers, reviews, _ = occupancy(store)
    if reservation:
        reviews = [r for r in reviews if not (r['id'] == reservation[0] and
                   r.get('reservation', {}).get('assignment_id') == reservation[1])]
    if reviews:
        raise ProgramError('The separate review concurrency slot is occupied across this store')
    if any(r.get('reservation', r['details']).get('settings', {}).get('shared_runtime_write') for r in workers):
        raise ProgramError('Shared runtime write reservation blocks separate evaluation')


def integrations(store):
    return [store._decode(r) for r in store.connection.execute(
        "SELECT * FROM records WHERE kind='integration_job'")
        if store._decode(r)['details']['state'] in ('preparing', 'prepared', 'checking', 'ownership_lost')]


def check_integration(store):
    if integrations(store):
        raise ProgramError('Exclusive integration/evaluation lease blocks workers and reviewers across this store')


def check_exclusive(store):
    """Caller holds BEGIN IMMEDIATE; keep this charge from startup through checks."""
    check_integration(store)
    workers, reviews, uncertain = occupancy(store)
    if workers or reviews or uncertain:
        raise ProgramError('Integration requires exclusive worker and reviewer capacity, including unknown cleanup')
