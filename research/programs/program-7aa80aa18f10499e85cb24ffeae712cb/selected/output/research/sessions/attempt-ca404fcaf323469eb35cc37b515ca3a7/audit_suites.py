import asyncio
import hashlib
import json
import os
from pathlib import Path
import time

from mcp import Client, StdioServerParameters

ROOT = Path('/workspace/hp-sim5-worktrees/bounded-research-assignment-0889b918')
SESSION = Path('/workspace/hp-sim5/output/research/sessions/attempt-ca404fcaf323469eb35cc37b515ca3a7')

def save(name, value):
    (SESSION / name).write_text(json.dumps(value, indent=2, default=str) + '\n')

async def main():
    parameters = StdioServerParameters(command=str(ROOT / '.venv/bin/python'), args=[str(ROOT / 'scripts/research_attach.py'), str(SESSION / 'connection.json')], cwd=str(ROOT), env=dict(os.environ))
    async with Client(parameters, read_timeout_seconds=90) as client:
        async def call(name, args=None, filename=None):
            started = time.monotonic()
            result = await client.call_tool(name, args or {})
            value = {'tool':name, 'args':args or {}, 'wall_s':time.monotonic()-started, 'is_error':result.is_error, 'result':result.structured_content, 'content':[c.model_dump() for c in result.content] if result.is_error else []}
            save(filename or name+'.json', value)
            with (SESSION / 'tool-calls.jsonl').open('a') as stream:
                stream.write(json.dumps(value, default=str)+'\n')
            print(json.dumps({'tool':name,'wall_s':value['wall_s'],'is_error':result.is_error,'result':result.structured_content if result.is_error else 'saved'}), flush=True)
            if result.is_error:
                return None
            return result.structured_content
        before = await call('runtime_status', filename='runtime-before-suites.json')
        jobs = []
        for suite in ('research-services', 'headless-autocal'):
            started = await call('start_test', {'suite':suite,'timeout_s':600}, filename=suite+'-start.json')
            if not started:
                continue
            job_id = started.get('job_id') or started.get('id')
            if not job_id:
                raise ValueError('start_test did not return a job identity')
            end = time.monotonic()+660
            while time.monotonic()<end:
                status = await call('test_status', {'job_id':job_id}, filename=suite+'-status.json')
                state = status.get('status', status.get('state')) if status else None
                print(json.dumps({'suite':suite,'job_id':job_id,'state':state}), flush=True)
                if state not in ('running','cancelling','starting'):
                    jobs.append({'suite':suite,'job_id':job_id,'result':status})
                    break
                await asyncio.sleep(5)
            else:
                await call('cancel_test', {'job_id':job_id}, filename=suite+'-cancel.json')
                raise TimeoutError(suite)
        save('suite-results.json', jobs)
        runs = []
        for label, command, record in [('baseline',[],True),('motion',[{'type':'Move','A':0.01}],False)]:
            result = await call('run_experiment', {'scene':'public/usd_scenes/hp3_rigid_body.usda','steps':500,'commands':command,'record':record,'label':'cloud audit '+label}, filename='experiment-'+label+'.json')
            if result:
                runs.append(result)
                await call('read_experiment', {'run_id':result['run_id'],'start_step':500,'limit':1}, filename='read-'+label+'.json')
        if len(runs)==2:
            await call('compare_experiments', {'baseline_id':runs[0]['run_id'],'candidate_id':runs[1]['run_id']})
            await call('capture_native_context', {'message':'Cloud audit selected final HP3 native step','run_id':runs[0]['run_id'],'step':500,'selected_entity':'effector'})
        await call('runtime_status', filename='runtime-after-suites.json')
        save('experiment-results.json', runs)

asyncio.run(main())
