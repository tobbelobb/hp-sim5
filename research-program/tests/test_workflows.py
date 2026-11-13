from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from hp_sim5_research.workspace import Sessions, read_json, write_json
from hp_sim5_research.archive import render_index


def invoke(module, capsys, *args):
    assert module.main(list(args)) == 0
    return json.loads(capsys.readouterr().out)


def test_program_and_researcher_resume_without_empty_artifacts(cli, repo, capsys):
    program = invoke(cli, capsys, '--question', 'Explain calibration failures')
    directory = Path(program['directory'])
    agenda = directory / 'agenda.md'
    agenda.write_text('Learned: investigate observability before tuning.\n')
    worker = invoke(cli, capsys, '--resume', program['program_id'], '--agent', 'Check observability')
    assert not Path(worker['session_dir']).exists()
    reopened = invoke(cli, capsys, '--resume', program['program_id'], '--agent-id', worker['id'])
    assert reopened['id'] == worker['id'] and reopened['session_dir'] == worker['session_dir']
    assert agenda.read_text().startswith('Learned:')
    session = Sessions(repo).open(resume=worker['id'])
    artifact = Path(session['directory']);artifact.mkdir()
    (artifact/'report.md').write_text('An informative negative result.\n')
    assert Sessions(repo).open(resume=str(artifact))['id'] == worker['id']
    assert len(list(directory.glob('session-*'))) == 1


def test_dry_run_writes_neither_state_nor_corpus(cli, repo, capsys):
    before = set(repo.rglob('*'))
    invoke(cli, capsys, '--question', 'Try a different method', '--dry-run')
    assert set(repo.rglob('*')) == before


def test_parallel_commissions_do_not_lose_records(cli, repo, capsys):
    program = invoke(cli, capsys, '--question', 'Explore alternatives')
    # Exercise real CLI processes and advisory locking rather than a shared mock.
    from conftest import ROOT
    script = repo/'research-program/cli.py'
    script.write_text((ROOT/'research-program/cli.py').read_text())
    (repo/'src/python').mkdir(parents=True)
    (repo/'src/python/hp_sim5_research').symlink_to(ROOT/'src/python/hp_sim5_research', target_is_directory=True)
    def commission(index):
        output = subprocess.check_output([sys.executable, str(script), '--resume', program['program_id'], '--agent', f'Alternative {index}'], text=True)
        return json.loads(output)
    with ThreadPoolExecutor(max_workers=4) as pool:
        workers=list(pool.map(commission, range(8)))
    saved=read_json(Path(program['directory'])/'program.json')
    assert {w['id'] for w in workers} == {w['id'] for w in saved['agents']}
    assert len({w['session_dir'] for w in workers}) == 8
    assert all(not Path(w['session_dir']).exists() for w in workers)


def test_worktree_keeps_tracked_research_and_new_output_private(launcher, repo, tmp_path):
    (repo/'.venv/bin').mkdir(parents=True);(repo/'.venv/bin/python').touch()
    plan=launcher.create_worktree('independent',tmp_path/'worktrees')
    worktree=plan['path']
    assert not (worktree/'research/output').is_symlink()
    assert (worktree/'research/output/session-prior/report.md').read_text() == 'Prior evidence\n'
    status=subprocess.check_output(['git','-C',str(worktree),'status','--porcelain'],text=True)
    assert status == ''
    new=worktree/'research/output/session-new/report.md';new.parent.mkdir();new.write_text('Private evidence\n')
    assert not (repo/'research/output/session-new').exists()


def test_resume_registry_shared_across_worktrees(launcher, repo, tmp_path):
    store=Sessions(repo)
    session=store.open(topic='Question')
    (repo/'.venv/bin').mkdir(parents=True);(repo/'.venv/bin/python').touch()
    worktree=launcher.create_worktree('worker',tmp_path/'worktrees')['path']
    session['repo']=str(worktree);session['directory']=str(worktree/'research/output'/session['id']);store.save(session)
    assert Sessions(worktree).open(resume=session['id'])['directory'] == session['directory']
    assert store.open(resume=session['id'])['repo'] == str(worktree)


def test_opening_agent_needs_no_physics_or_client_and_no_corpus(launcher, repo, monkeypatch, capsys):
    monkeypatch.setattr(sys,'argv',['research_agent.py','--prompt','Study saved evidence'])
    assert launcher.main() == 0
    result=json.loads(capsys.readouterr().out)
    assert not Path(result['session_dir']).exists()
    assert 'Study saved evidence' in result['task']
    monkeypatch.setattr(sys,'argv',['research_agent.py','--resume'])
    assert launcher.main()==0
    assert json.loads(capsys.readouterr().out)['id']==result['id']


def test_existing_artifacts_require_explicit_resume(launcher, repo, monkeypatch, capsys):
    prior=repo/'research/output/session-prior'
    monkeypatch.setattr(sys,'argv',['research_agent.py','--session-dir',str(prior)])
    with pytest.raises(SystemExit) as error:launcher.main()
    assert error.value.code==1
    assert '--resume' in capsys.readouterr().err
    assert (prior/'report.md').read_text()=='Prior evidence\n'


def test_legacy_resume_cannot_silently_allocate_artifacts(launcher, repo, monkeypatch, capsys):
    monkeypatch.setattr(sys,'argv',['research_agent.py','--codex','--','--resume','unknown-client'])
    with pytest.raises(SystemExit):launcher.main()
    assert 'artifact directory' in capsys.readouterr().err
    assert not (repo/'.research-state/sessions').exists()


def test_sealed_review_preserves_exact_bytes_and_detects_tampering(cli, repo, capsys):
    program=invoke(cli,capsys,'--question','Test claim')
    selection=repo/'selection.json'
    write_json(selection,{'claim':'A scoped negative result','files':['research/output/session-prior/report.md']})
    sealed=invoke(cli,capsys,'--resume',program['program_id'],'--review',str(selection))
    bundle=Path(sealed['directory'])
    (repo/'research/output/session-prior/report.md').write_text('Later revision\n')
    checked=invoke(cli,capsys,'--resume',program['program_id'],'--verify',str(bundle))
    assert checked['verified'] and checked['scientific_validity']=='not_assessed'
    captured=bundle/sealed['manifest']['files'][0]['path']
    assert captured.read_text()=='Prior evidence\n'
    captured.write_text('Modified evidence\n')
    with pytest.raises(SystemExit):cli.main(['--resume',program['program_id'],'--verify',str(bundle)])
    assert 'evidence changed' in capsys.readouterr().err


def test_review_refuses_private_runtime_state(cli,repo,capsys):
    program=invoke(cli,capsys,'--question','Test')
    write_json(repo/'selection.json',{'claim':'Unsafe selection','files':['.research-state/last-program.json']})
    with pytest.raises(SystemExit):cli.main(['--resume',program['program_id'],'--review',str(repo/'selection.json')])
    assert 'private launcher state' in capsys.readouterr().err


def test_launcher_only_and_placeholder_sessions_are_not_catalog_findings(tmp_path):
    empty=tmp_path/'session-empty';empty.mkdir()
    (empty/'exit.json').write_text('{"returncode":0}')
    (empty/'abstract.md').write_text('# Pending\n\nNo research result is available yet.\n')
    assert 'Pending' not in render_index(tmp_path)
    (empty/'report.md').write_text('Useful partial evidence')
    assert 'Pending' in render_index(tmp_path)


def test_prepared_artifact_path_cannot_be_reserved_twice(repo):
    store=Sessions(repo)
    target=repo/'research/output/custom-path'
    store.open(directory=target,topic='First owner')
    with pytest.raises(ValueError,match='already reserved'):
        store.open(directory=target,topic='Second owner')


def test_worktree_reexec_preserves_question_and_stable_identity(launcher,repo,tmp_path,monkeypatch,capsys):
    (repo/'.venv/bin').mkdir(parents=True);(repo/'.venv/bin/python').touch()
    prompt=repo/'question.txt';prompt.write_text('The complete research question')
    monkeypatch.chdir(repo)
    calls=[]
    def capture(executable,argv):
        calls.append(argv)
        raise RuntimeError('intercepted exec')
    monkeypatch.setattr(launcher.os,'execv',capture)
    monkeypatch.setattr(sys,'argv',['research_agent.py','--codex','--worktree','--worktree-dir',str(tmp_path/'worktrees'),'--prompt-file','question.txt'])
    with pytest.raises(SystemExit):launcher.main()
    argv=calls[0]
    assert '--worktree' not in argv and '--worktree-dir' not in argv
    assert argv[argv.index('--prompt-file')+1]==str(prompt)
    identity=argv[argv.index('--resume')+1]
    saved=Sessions(repo).open(resume=identity)
    assert saved['topic']=='The complete research question'
    assert Path(saved['directory']).is_relative_to(Path(saved['repo']))
    assert not Path(saved['directory']).exists()


def test_legacy_program_preserves_history_when_commissioning(cli, repo, capsys):
    directory = repo / 'research/output/program-legacy'
    original = {'program': {'id': directory.name, 'goal': 'An old question'},
                'records': [{'id': 'important-old-evidence'}]}
    write_json(directory / 'program.json', original)
    reopened = invoke(cli, capsys, '--resume', directory.name, '--dry-run')
    assert reopened['program_id'] == directory.name
    assert not (directory / 'agenda.md').exists()
    invoke(cli, capsys, '--resume', directory.name, '--agent', 'A new investigation')
    saved = read_json(directory / 'program.json')
    assert saved['records'] == original['records'] and len(saved['agents']) == 1
    assert (directory / 'agenda.md').is_file()


@pytest.mark.parametrize('action', [['--agent', 'Subquestion'], ['--review', 'missing.json']])
def test_rejected_backend_cannot_change_program_or_registry(cli, repo, capsys, action):
    program = invoke(cli, capsys, '--question', 'Existing question')
    before = {p: p.read_bytes() for p in repo.rglob('*') if p.is_file()}
    with pytest.raises(SystemExit) as error:
        cli.main(['--resume', program['program_id'], *action, '--codex'])
    assert error.value.code == 2
    assert {p: p.read_bytes() for p in repo.rglob('*') if p.is_file()} == before


def test_review_reads_committed_blobs_even_when_absent_from_checkout(cli, repo, capsys):
    program = invoke(cli, capsys, '--question', 'Review committed claim')
    report = repo / 'research/output/session-prior/report.md'
    report.unlink()
    write_json(repo / 'selection.json', {'claim': 'Historical observation',
        'files': ['research/output/session-prior/report.md']})
    sealed = invoke(cli, capsys, '--resume', program['program_id'], '--review', str(repo/'selection.json'))
    item = sealed['manifest']['files'][0]
    assert (Path(sealed['directory']) / item['path']).read_text() == 'Prior evidence\n'
    assert item['blob'] == subprocess.check_output(['git', '-C', str(repo), 'rev-parse',
        'HEAD:research/output/session-prior/report.md'], text=True).strip()


def test_platform_worktree_requires_explicit_binding(launcher, repo, tmp_path, monkeypatch, capsys):
    session = Sessions(repo).open(topic='Worker')
    (repo/'.venv/bin').mkdir(parents=True)
    (repo/'.venv/bin/python').touch()
    worktree = launcher.create_worktree('platform-worker', tmp_path/'worktrees')['path']
    monkeypatch.setattr(launcher, 'ROOT', worktree)
    monkeypatch.setattr(sys, 'argv', ['research_agent.py', '--resume', session['id'], '--runtime-status'])
    with pytest.raises(SystemExit):
        launcher.main()
    assert '--workspace first' in capsys.readouterr().err
    monkeypatch.setattr(sys, 'argv', ['research_agent.py', '--resume', session['id'], '--workspace'])
    assert launcher.main() == 0
    assert json.loads(capsys.readouterr().out)['repo'] == str(worktree)


def test_missing_worker_workspace_does_not_return_a_stale_handoff(cli, repo, tmp_path, capsys):
    program = invoke(cli, capsys, '--question', 'Question')
    worker = invoke(cli, capsys, '--resume', program['program_id'], '--agent', 'Worker')
    store = Sessions(repo)
    saved = store.open(resume=worker['id'])
    saved['repo'] = str(tmp_path / 'deleted-worktree')
    store.save(saved)
    with pytest.raises(SystemExit):
        cli.main(['--resume', program['program_id'], '--agent-id', worker['id']])
    assert 'Saved workspace is unavailable' in capsys.readouterr().err


def test_runtime_process_creates_log_directory_on_first_use(tmp_path):
    from conftest import ROOT, load
    module = load('runtime_lazy_directory', ROOT/'scripts/research_runtime.py')
    runtime = module.Runtime.__new__(module.Runtime)
    runtime.directory = tmp_path/'evidence/native'
    runtime.processes = {}
    assert not runtime.directory.exists()
    process, log = runtime.spawn('browser', [sys.executable, '-c', 'print("ready")'])
    try:
        assert process.wait(timeout=10) == 0
        assert log.read_text().strip() == 'ready'
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
