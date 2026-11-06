from types import SimpleNamespace

import pytest

from scripts.research_runtime import Runtime


@pytest.mark.parametrize('active', ['collection', 'autocal', 'test'])
def test_cloud_runtime_allows_only_one_job_type_at_a_time(monkeypatch, active):
    runtime = Runtime.__new__(Runtime)
    runtime.active_job = 'collection' if active == 'collection' else None
    runtime.autocal = SimpleNamespace(active='autocal' if active == 'autocal' else None)
    runtime.tests = SimpleNamespace(active='test' if active == 'test' else None)
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'local')
    runtime._check_client_capacity('start_test')
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    with pytest.raises(ValueError, match='Cloud research runs one job at a time'):
        runtime._check_client_capacity('start_test')

