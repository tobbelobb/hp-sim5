"""Cloud setup failures leave the direct-dispatch path unchanged."""
import pytest

from program.store import ProgramError
from test_research_program_review import context
from test_research_program_controller import local_manager, ready


def test_cloud_setup_failure_does_not_start_attempt(context, monkeypatch):
    m = local_manager(context)
    ready(m, 'new')
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    monkeypatch.setattr('program.manager.runtime_check', lambda *a, **kw: {'ok': False})
    before = m.store.snapshot('p')['records']
    with pytest.raises(ProgramError, match='no worker attempt was charged'):
        m.worker_start('p', 'new', settings={'machine': 'hp3'})
    assert m.store.snapshot('p')['records'] == before
