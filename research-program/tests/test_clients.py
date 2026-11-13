import json
import os
from pathlib import Path
import sys

import pytest

from hp_sim5_research.workspace import Sessions


class Service:
    def __init__(self,*args,**kwargs):
        self.token='private-token';self.endpoint='http://127.0.0.1:1';self.closed=False
    def start(self):return self
    def call(self,operation):return {'session_id':None}
    def close(self):self.closed=True


def client(repo,name):
    directory=repo/'bin';directory.mkdir(exist_ok=True)
    path=directory/name
    path.write_text('''#!/usr/bin/env python3
import json,os,sys
from pathlib import Path
assert 'OPENAI_API_KEY' not in os.environ if 'exec' in sys.argv else True
args=Path(os.environ['TEST_CLIENT_ARGS']);args.write_text(json.dumps(sys.argv[1:]))
artifact=Path(os.environ['TEST_ARTIFACT'])
if os.environ.get('TEST_WRITE_REPORT'):
 artifact.mkdir(parents=True,exist_ok=True)
 with (artifact/'report.md').open('a') as stream:stream.write('Observed result\\n')
if 'exec' in sys.argv:
 sys.stdin.read()
 print(json.dumps({'type':'thread.started','thread_id':'saved-codex-conversation'}))
 print(json.dumps({'type':'item.completed','item':{'type':'agent_message','text':'Done'}}))
elif '--print' in sys.argv:
 sys.stdin.read()
 print(json.dumps({'type':'result','is_error':False,'result':'Done','session_id':'saved-claude-conversation'}))
''')
    path.chmod(0o755)
    return directory


@pytest.mark.parametrize('backend',['codex','claude'])
def test_batch_resume_preserves_evidence_and_exact_conversation(launcher,repo,monkeypatch,capsys,backend):
    artifact=repo/'research/output/session-test'
    executable=client(repo,backend)
    monkeypatch.setenv('PATH',str(executable)+os.pathsep+os.environ['PATH'])
    monkeypatch.setenv('TEST_CLIENT_ARGS',str(repo/'client-args.json'))
    monkeypatch.setenv('TEST_ARTIFACT',str(artifact))
    monkeypatch.setenv('TEST_WRITE_REPORT','1')
    monkeypatch.setenv('OPENAI_API_KEY','sentinel')
    services=[]
    def service(*a,**kw):
        value=Service();services.append(value);return value
    monkeypatch.setattr(launcher,'RuntimeService',service)
    monkeypatch.setattr(sys,'argv',['research_agent.py','--'+backend,'--batch','--session-dir',str(artifact),'--prompt','Investigate'])
    assert launcher.main()==0
    first=Sessions(repo).open(resume='latest')
    assert first['conversations'][backend]=='saved-'+backend+'-conversation'
    monkeypatch.setattr(sys,'argv',['research_agent.py','--'+backend,'--batch','--resume',first['id'],'--prompt','Continue'])
    assert launcher.main()==0
    assert (artifact/'report.md').read_text()=='Observed result\nObserved result\n'
    args=json.loads((repo/'client-args.json').read_text())
    assert ('resume' if backend=='codex' else '--resume') in args
    assert 'saved-'+backend+'-conversation' in args
    assert all(s.closed for s in services)
    assert not (artifact/'connection.json').exists()
    assert not (artifact/'launch.json').exists()
    assert len(list((repo/'research/output').glob('session-test*')))==1


def test_empty_interactive_client_leaves_no_corpus_directory(launcher,repo,monkeypatch):
    executable=client(repo,'codex')
    artifact=repo/'research/output/session-empty'
    monkeypatch.setenv('PATH',str(executable)+os.pathsep+os.environ['PATH'])
    monkeypatch.setenv('TEST_CLIENT_ARGS',str(repo/'client-args.json'))
    monkeypatch.setenv('TEST_ARTIFACT',str(artifact))
    monkeypatch.setattr(launcher,'RuntimeService',Service)
    monkeypatch.setattr(sys,'argv',['research_agent.py','--codex','--session-dir',str(artifact)])
    assert launcher.main()==0
    assert not artifact.exists()


def test_interrupted_startup_cleans_owned_service(launcher,repo,monkeypatch):
    services=[]
    class Interrupted(Service):
        def __init__(self,*a,**kw):super().__init__();services.append(self)
        def start(self):raise KeyboardInterrupt
    monkeypatch.setattr(launcher,'RuntimeService',Interrupted)
    artifact=repo/'research/output/session-interrupted'
    monkeypatch.setattr(sys,'argv',['research_agent.py','--codex','--session-dir',str(artifact)])
    assert launcher.main()==130
    assert services[0].closed
    assert not artifact.exists()


def test_manager_claude_resume_is_bound_to_program(cli,repo,monkeypatch,capsys):
    calls=[]
    monkeypatch.setattr(cli.subprocess,'call',lambda argv,**kwargs:calls.append(argv) or 0)
    assert cli.main(['--question','An important question','--claude'])==0
    from hp_sim5_research.workspace import read_json
    program=read_json(Sessions(repo).state/'last-program.json')['directory']
    original=calls[0][calls[0].index('--session-id')+1]
    assert cli.main(['--resume',program,'--claude'])==0
    assert calls[1][calls[1].index('--resume')+1]==original


def test_prepared_handoff_starts_a_new_batch_client_on_first_use(launcher,repo,monkeypatch):
    session=Sessions(repo).open(topic='First execution',backend='codex')
    executable=client(repo,'codex')
    monkeypatch.setenv('PATH',str(executable)+os.pathsep+os.environ['PATH'])
    monkeypatch.setenv('TEST_CLIENT_ARGS',str(repo/'client-args.json'))
    monkeypatch.setenv('TEST_ARTIFACT',session['directory'])
    monkeypatch.setattr(launcher,'RuntimeService',Service)
    monkeypatch.setattr(sys,'argv',['research_agent.py','--codex','--batch','--resume',session['id'],'--prompt','Investigate'])
    assert launcher.main()==0
    assert 'resume' not in json.loads((repo/'client-args.json').read_text())


def test_codex_batch_options_belong_to_exec_and_shared_state_is_writable(repo, tmp_path):
    from hp_sim5_research import coding_client
    command = coding_client.codex_command(repo, tmp_path/'run', batch=True,
        model='chosen-model', codex_args=['resume', 'conversation-id'])
    assert command[:2] == ['codex', 'exec']
    assert command.index('--sandbox') < command.index('resume')
    assert command[command.index('--model')+1] == 'chosen-model'
    settings, _ = coding_client.forwarded_settings(command)
    assert str(repo/'.research-state') in settings['sandbox_workspace_write.writable_roots']
    assert command[-1] == '-' and '--json' in command


def test_forwarded_provider_selects_matching_environment(launcher, repo, monkeypatch):
    from hp_sim5_research import coding_client
    executable = client(repo, 'codex')
    artifact = repo / 'research/output/session-provider'
    monkeypatch.setenv('PATH', str(executable)+os.pathsep+os.environ['PATH'])
    monkeypatch.setenv('TEST_CLIENT_ARGS', str(repo/'client-args.json'))
    monkeypatch.setenv('TEST_ARTIFACT', str(artifact))
    monkeypatch.setattr(launcher, 'RuntimeService', Service)
    providers = []
    monkeypatch.setattr(launcher, 'codex_environment', lambda provider: providers.append(provider) or dict(os.environ))
    monkeypatch.setattr(sys, 'argv', ['research_agent.py', '--codex', '--session-dir', str(artifact),
        '--', '-c', 'model_provider=other'])
    assert launcher.main() == 0
    assert providers == ['other']
    settings, _ = coding_client.forwarded_settings(json.loads((repo/'client-args.json').read_text()))
    assert settings['model_provider'] == 'other'
