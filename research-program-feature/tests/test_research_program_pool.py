"""Capacity is operational; no research planning or review-budget reservations."""
from pathlib import Path

import pytest

from program.store import ProgramStore
from program import pool
from test_research_program_manager import manager, attach, tool, until


def test_question_dispatch_has_no_planning_records_or_contract_bloat(manager):
    a = attach(manager)
    work = tool(a, 'commission', {'objective': 'Find and fix the smallest useful improvement',
                                  'settings': {'machine': 'hp3'}})
    worker = tool(a, 'worker_start', {'assignment_id': work['id']})
    until(lambda: Path(worker['details']['session_dir'], 'ready').exists())
    prompt = Path(worker['details']['attempt_dir'], 'prompt.txt').read_text()
    assert 'Find and fix the smallest useful improvement' in prompt
    assert not any(term in prompt for term in ('allocation', 'route_plan', 'investigation', 'fresh_fits'))
    assert '--wall-deadline' not in worker['details']['argv']
    assert '--worktree' in worker['details']['argv']
    with ProgramStore(manager['store']) as store:
        kinds = {r['kind'] for r in store.snapshot('p')['records']}
        assert not kinds.intersection({'runtime_allocation', 'route_plan', 'investigation', 'strategy_event'})
    overview = tool(a, 'overview')
    assert 'records' not in overview and 'strategy' not in overview
    assert len(str(overview)) < 3000
    assert not list(Path(a['session']).glob('overview.*'))


def test_capacity_and_exclusive_builds_still_work(manager):
    a = attach(manager)
    tool(a, 'configure_pool', {'worker_limit': 2, 'reason': 'Independent investigations'})
    one = tool(a, 'commission', {'objective': 'One', 'settings': {'machine': 'hp3'}})
    two = tool(a, 'commission', {'objective': 'Two', 'settings': {'machine': 'hp3'}})
    first = tool(a, 'worker_start', {'assignment_id': one['id'], 'settings': {'shared_runtime_write': True}})
    until(lambda: Path(first['details']['session_dir'], 'ready').exists())
    assert 'Shared runtime' in tool(a, 'worker_start', {'assignment_id': two['id']}, ok=False)['message']
    with ProgramStore(manager['store']) as store:
        with pytest.raises(ValueError, match='Shared runtime'):
            pool.check_review(store)
    tool(a, 'worker_cancel', {'attempt_id': first['id']})
    until(lambda: tool(a, 'worker_status', {'attempt_id': first['id']})['details']['state'] == 'cancelled')
    second = tool(a, 'worker_start', {'assignment_id': two['id']})
    assert second['details']['settings']['worktree']


def test_commissioned_machine_and_invalid_settings_do_not_spawn(manager):
    a = attach(manager)
    work = tool(a, 'commission', {'objective': 'Compare settings', 'settings': {'machine': 'hp3'}})
    assert 'conflicts' in tool(a, 'worker_start', {'assignment_id': work['id'], 'settings': {'machine': 'hp4'}}, ok=False)['message']
    assert tool(a, 'worker_sessions') == []
    for limit in (0, 5, True):
        assert tool(a, 'configure_pool', {'worker_limit': limit, 'reason': 'Invalid'}, ok=False)['code'] == 'invalid_request'
