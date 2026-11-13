"""Real supervisor + MCP relay, no model calls. Slow opt-in tests own and retire their processes."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from conftest import ROOT


@pytest.fixture
def live_repo(repo):
    (repo/'scripts').mkdir()
    for name in ('research_agent.py','research_tools.py','research_runtime.py','research_attach.py','hp_sim5_mcp.py'):
        shutil.copyfile(ROOT/'scripts'/name,repo/'scripts'/name)
    for path in (ROOT/'scripts').glob('*.mjs'):
        (repo/'scripts'/path.name).symlink_to(path)
    for name in ('src','.venv','node_modules','machines','hp-sim-3d','autocal','integrations'):
        (repo/name).symlink_to(ROOT/name,target_is_directory=(ROOT/name).is_dir())
    (repo/'public').mkdir()
    shutil.copytree(ROOT/'public/usd_scenes',repo/'public/usd_scenes')
    (repo/'RRF/run/vsd').mkdir(parents=True)
    shutil.copytree(ROOT/'RRF/run/vsd/sys',repo/'RRF/run/vsd/sys')
    (repo/'RRF/build').symlink_to(ROOT/'RRF/build',target_is_directory=True)
    return repo


def agent(repo,*args):
    result=subprocess.run([sys.executable,str(repo/'scripts/research_agent.py'),*args],capture_output=True,text=True,timeout=70)
    assert result.returncode==0,result.stderr
    return json.loads(result.stdout)


def tool(repo,connection,name,params=None):
    result=subprocess.run([sys.executable,str(repo/'scripts/research_tools.py'),'call',connection,name,
                           '--params',json.dumps(params or {})],capture_output=True,text=True,timeout=100)
    assert result.returncode==0,result.stderr
    payload=json.loads(result.stdout)
    assert not payload.get('isError'),payload
    return payload['structuredContent']


@pytest.mark.slow
def test_two_real_researchers_have_isolated_optional_worlds_and_reopen(live_repo):
    repo=live_repo
    first=agent(repo,'--prompt','First independent investigation')
    second=agent(repo,'--prompt','Second independent investigation')
    live=[]
    try:
        for session in (first,second):
            value=agent(repo,'--resume',session['id'],'--start-runtime','--no-record')
            live.append(value)
            assert tool(repo,value['tool_connection'],'runtime_status')['world_id'] is None
            assert not Path(session['session_dir']).exists()
        a=tool(repo,live[0]['tool_connection'],'configure_session',{'configuration':{'machine':'hp3'}})
        assert a['world_id']
        assert tool(repo,live[1]['tool_connection'],'runtime_status')['world_id'] is None
        b=tool(repo,live[1]['tool_connection'],'configure_session',{'configuration':{'machine':'hp4'}})
        assert a['world_id'] != b['world_id']
        assert a['session_directory'] != b['session_directory']
        assert tool(repo,live[0]['tool_connection'],'runtime_status')['world_id']==a['world_id']
    finally:
        for session in (first,second):
            agent(repo,'--resume',session['id'],'--stop-runtime')
    restarted=agent(repo,'--resume',first['id'],'--start-runtime','--no-record')
    try:
        assert restarted['session_dir']==first['session_dir']
        assert tool(repo,restarted['tool_connection'],'runtime_status')['world_id'] is None
        assert len(list((Path(first['session_dir'])/'native').glob('*/manifest.json'))) == 1
    finally:
        agent(repo,'--resume',first['id'],'--stop-runtime')


@pytest.mark.slow
def test_real_open_close_does_not_publish_empty_session(live_repo):
    repo=live_repo
    prepared=agent(repo,'--prompt','Only open tools')
    try:
        live=agent(repo,'--resume',prepared['id'],'--start-runtime')
        tool(repo,live['tool_connection'],'runtime_status')
    finally:
        agent(repo,'--resume',prepared['id'],'--stop-runtime')
    assert not Path(prepared['session_dir']).exists()
    assert not (repo/'research/output/index.md').exists()


@pytest.mark.slow
def test_runtime_retires_itself_after_supervisor_loss(live_repo):
    import signal
    import time
    from hp_sim5_research.services import process_live
    repo=live_repo
    session=agent(repo,'--prompt','Owner-loss boundary')
    live=agent(repo,'--resume',session['id'],'--start-runtime')
    try:
        os.kill(live['owner']['pid'],signal.SIGKILL)
        deadline=time.monotonic()+15
        while process_live(live['runtime_owner']) and time.monotonic()<deadline:
            time.sleep(.1)
        assert not process_live(live['runtime_owner'])
        assert not agent(repo,'--resume',session['id'],'--runtime-status')['running']
        assert not Path(session['session_dir']).exists()
    finally:
        agent(repo,'--resume',session['id'],'--stop-runtime')
