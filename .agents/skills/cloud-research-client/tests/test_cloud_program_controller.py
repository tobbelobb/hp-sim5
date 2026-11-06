"""Codex Cloud dispatch guards; fixture children, no model or physics calls."""
import time

import pytest

from program.store import ProgramError
from test_research_program_review import context
from test_research_program_controller import local_manager, ready


def test_cloud_setup_failure_does_not_allocate_or_charge_attempt(context, monkeypatch):
    m = local_manager(context)
    ready(m, 'new')
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    blocked = {'ok': False, 'checks': {'launcher_sockets': {'ok': False, 'error': 'EPERM'}}}
    monkeypatch.setattr('program.controller.runtime_check', lambda *a, **kw: blocked)
    before = m.store.snapshot('p')['records']
    with pytest.raises(ProgramError, match='no worker attempt was charged'):
        m.controller.run_once('p', 'new', settings={'machine': 'hp3'})
    assert m.store.snapshot('p')['records'] == before
    allocation = m.controller.allocate('p', 1, 10)
    with pytest.raises(ProgramError, match='no worker attempt was charged'):
        m.controller.run_once('p', 'new', allocation_id=allocation['id'], settings={'machine': 'hp3'})
    details = m.controller.current(allocation['id'])['details']
    assert details['reservations'] == []
    assert details['wall_reserved_seconds'] == 0


def test_cloud_setup_failure_holds_queue_without_repeated_dispatch(context, monkeypatch):
    m = local_manager(context)
    ready(m, 'new')
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    calls = []
    def blocked(*args, **kwargs):
        calls.append(True)
        return {'ok': False}
    monkeypatch.setattr('program.controller.runtime_check', blocked)
    allocation = m.controller.allocate('p', 1, 10)
    m.controller.run_within_allocation('p', allocation['id'], ['new'], settings={'machine': 'hp3'})
    m.controller.poll()
    details = m.controller.current(allocation['id'])['details']
    assert len(calls) == 1
    assert details['reservations'] == details['queue'] == []
    assert details['events'][-1]['event'] == 'queue_dispatch_blocked'


def test_slow_cloud_setup_check_renews_lease_without_extending_wall_budget(context, monkeypatch):
    m = local_manager(context)
    m.workers = m.store.workers(context['root'], executable=context['tmp'] / 'worker')
    ready(m, 'new')
    allocation = m.controller.allocate('p', 1, 10)
    wall_deadline = allocation['details']['deadline']
    now = time.time()
    clock = [now]
    monkeypatch.setattr(time, 'time', lambda: clock[0])
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    def checked(*args, **kwargs):
        clock[0] += 20
        return {'ok': True}
    monkeypatch.setattr('program.controller.runtime_check', checked)
    attempt = m.controller.run_once('p', 'new', allocation_id=allocation['id'], settings={'machine': 'hp3'})
    m.workers.children[attempt['id']].wait(timeout=5)
    details = m.controller.current(allocation['id'])['details']
    assert details['lease_until'] > clock[0]
    assert details['deadline'] == wall_deadline
    assert len(details['reservations']) == 1


