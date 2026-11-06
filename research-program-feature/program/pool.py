"""Store-wide cooperative capacity checks. Call reservations inside BEGIN IMMEDIATE."""
from .store import ProgramError
from hp_sim5_research.client_transport import client_adapter

MAX_WORKERS = 4
ACTIVE = ('starting', 'running', 'cancelling', 'ownership_lost')
SHARED_BOUNDARY = ('Workers own separate worktrees, worlds and session artifacts. Output, RRF/build, '
                   'dependencies and the starting native addon are shared. Treat shared paths and '
                   'datasets as immutable; use only your session directory for new artifacts. '
                   'Shared builds, dependency changes and combined evaluation require an exclusive '
                   'shared_runtime_write reservation. Same-UID cooperation is not adversarial isolation.')


def records(store):
    return [store._decode(r) for r in store.connection.execute(
        "SELECT * FROM records WHERE kind IN ('attempt','review_run')")]


def limit(store):
    return store.connection.execute('SELECT worker_limit FROM worker_pool WHERE singleton=1').fetchone()[0]


def unknown(details):
    return details['state'] == 'interrupted' or details['cleanup'].startswith(('failed', 'unverified', 'unknown'))


def occupancy(store):
    """Include workers and reviewers belonging to unopened programs."""
    workers, reviews, uncertain = [], [], []
    saved = records(store)
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
    return workers, reviews, uncertain


def check_worker(store, program_id, assignment_id, *, exclusive=False):
    check_integration(store)
    workers, reviews, uncertain = occupancy(store)
    if uncertain:
        raise ProgramError('Interrupted ownership or unknown cleanup needs an explicit bounded handoff, including unopened programs')
    adapter = client_adapter()
    if adapter:
        adapter.check_capacity(workers, reviews)
    if len(workers) >= limit(store):
        raise ProgramError('A worker is already reserved; store-wide worker pool is full')
    if (exclusive and (workers or reviews)) or any(
            r.get('reservation', r['details']).get('settings', {}).get('shared_runtime_write') for r in workers):
        raise ProgramError('Shared runtime writes require exclusive worker and reviewer capacity')


def check_review(store):
    check_integration(store)
    workers, reviews, _ = occupancy(store)
    if reviews:
        raise ProgramError('The separate review concurrency slot is occupied across this store')
    adapter = client_adapter()
    if adapter:
        adapter.check_capacity(workers, [])
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
