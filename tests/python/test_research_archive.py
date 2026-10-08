import re

from hp_sim5_research.archive import catalog_entries, initial_abstract, render_index, session_name, write_index


def test_session_names_are_descriptive_safe_and_unique():
    first = session_name('../../HP3 held-out sweeps / radius & winding')
    assert re.fullmatch(r'\d{4}-\d{2}-\d{2}-hp3-held-out-sweeps-radius-winding-[0-9a-f]{8}', first)
    assert session_name('../../HP3 held-out sweeps / radius & winding') != first
    assert re.fullmatch(r'\d{4}-\d{2}-\d{2}-research-[0-9a-f]{8}', session_name(' / .. '))
    assert len(session_name('long topic ' * 100)) < 90


def test_catalog_deduplicates_aliases_and_keeps_evidence_links(tmp_path):
    sessions = tmp_path / 'sessions'
    session = sessions / ('a' * 32)
    session.mkdir(parents=True)
    (session / 'abstract.md').write_text(
        '# Radius validation\n\nRejected winding hypothesis.\n\n'
        '[Evidence](metrics.json) [Prior](../prior/report.md) '
        '[Paper](https://example.org/paper) [Code](/repo/code.py)\n')
    (session / 'report.md').write_text('Long report with an unrelated word: elasticity')
    alias = sessions / 'hp3-radius-validation'
    alias.symlink_to(session.name, target_is_directory=True)
    entries = list(catalog_entries(tmp_path))
    assert [p for p, _ in entries] == [alias]
    index = render_index(tmp_path)
    assert '[Evidence](sessions/hp3-radius-validation/metrics.json)' in index
    assert '[Prior](sessions/hp3-radius-validation/../prior/report.md)' in index
    assert '[Paper](https://example.org/paper)' in index
    assert '[Code](/repo/code.py)' in index
    assert 'Radius validation' in render_index(tmp_path, 'WINDING')
    assert 'Radius validation' not in render_index(tmp_path, 'elasticity')
    write_index(tmp_path)
    assert (tmp_path / 'index.md').read_text() == index
    assert not list(tmp_path.glob('.index-*'))


def test_pending_and_legacy_sessions_do_not_claim_completed_research(tmp_path):
    pending = tmp_path / 'pending'
    pending.mkdir()
    (pending / 'abstract.md').write_text(initial_abstract('HP3 candidate ranking'))
    legacy = tmp_path / 'legacy'
    legacy.mkdir()
    (legacy / 'exit.json').write_text('{"returncode": 0}')
    unrelated = tmp_path / 'experiments'
    unrelated.mkdir()
    (unrelated / 'telemetry.jsonl').write_text('{}')
    index = render_index(tmp_path)
    assert 'None recorded yet' in index
    assert 'Status: abstract missing' in index
    assert 'tested hypotheses have not been cataloged' in index
    assert '[experiments]' not in index
