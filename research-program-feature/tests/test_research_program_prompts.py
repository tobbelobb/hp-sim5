"""Prompt portfolio and packaged role-contract tests; no model or research execution."""
from pathlib import Path
import shutil

import pytest

from program.store import ProgramError, ProgramStore
from program.prompts import PromptPortfolio, role_instructions
from program.strategy import ProgramStrategy


ROOT = Path(__file__).resolve().parents[2]


def test_canonical_roles_are_the_only_role_prompts():
    for role, name in [('manager', 'MANAGER.md'), ('reviewer', 'REVIEWER.md')]:
        assert role_instructions(role) == (ROOT / 'research-program-feature/role-instructions' / name).read_text()
    for role in ('assignment', 'single-agent'):
        with pytest.raises(ProgramError, match='Unknown'):
            role_instructions(role)


def test_portfolio_contains_only_registered_mission_prompts():
    portfolio = PromptPortfolio(ROOT).inventory()
    assert {entry['path'] for entry in portfolio['prompts']} == {
        'research/prompts/autocal.txt',
        'research/prompts/autocal_broader.txt',
        'research/prompts/autocal_parameter_variation_focus.txt',
        'research/prompts/autocal_recording_focused.txt',
        'research/prompts/researcher-tooling-wish-list.md',
    }
    assert all(len(entry['sha256']) == 64 for entry in portfolio['prompts'])


def test_selected_prompt_returns_exact_text_for_worker_instructions():
    selected = PromptPortfolio(ROOT).resolve('autocal-parameter-variation', include_text=True)['prompt']
    assert selected['path'] == 'research/prompts/autocal_parameter_variation_focus.txt'
    assert selected['text'] == (ROOT / selected['path']).read_text()
    assert selected['text'].startswith('Improve encoder-only HP3 autocalibration')


def test_unregistered_prompt_fails_fast(tmp_path):
    root = tmp_path / 'repo'
    shutil.copytree(ROOT / 'research/prompts', root / 'research/prompts')
    (root / 'research/prompts/unclassified-plumbing.txt').write_text('Not a research mission.\n')
    with pytest.raises(ProgramError, match='Unregistered research prompt'):
        PromptPortfolio(root).inventory()


def test_route_plan_records_selected_prompt_provenance(tmp_path):
    with ProgramStore(tmp_path / 'store', create=True) as store:
        program = store.create_program('Does prompt triage work?')
        store.add_approach(program['id'], program['question_id'], 'Portfolio test', 'Fixture route', record_id='route')
        strategy = ProgramStrategy(store, PromptPortfolio(ROOT))
        plan = strategy.route(
            program['id'], 'route', 'A selected mission changes the next decision',
            {'machine': 'HP3'}, 1, {'wall_minutes': 5}, 'The result may repeat known work',
            'A reviewable patch or scoped rejection', 'Tests the selected mission rather than a generic brief',
            'Check whether maximum per-anchor error improves', 'A new reviewed result changes the decision',
            'The prompt is the closest fit to the current evidence gap',
            prompt_id='autocal-parameter-variation')
    assert plan['details']['prompt']['id'] == 'autocal-parameter-variation'
    assert plan['details']['prompt']['path'] == 'research/prompts/autocal_parameter_variation_focus.txt'
    assert len(plan['details']['prompt']['sha256']) == 64
