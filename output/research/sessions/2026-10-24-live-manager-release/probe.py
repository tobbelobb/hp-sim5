"""Bounded real CLI probes. Raw control/client data stays in ignored private storage."""
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[4]
SESSION = Path(__file__).resolve().parent
PRIVATE = ROOT / '.research-programs/task016-private'
PRIVATE.mkdir(parents=True, exist_ok=True, mode=0o700)
STORE = PRIVATE / 'store'
CLI = [str(ROOT / 'hp-sim5-research-program')]
TRANSCRIPT = SESSION / 'cli-transcript.json'
SENSITIVE = re.compile(r'token|secret|authorization|password|capability', re.I)


def redact(value):
    if isinstance(value, dict):
        return {k: '[REDACTED]' if SENSITIVE.search(k) else redact(v) for k, v in value.items()}
    if isinstance(value, list):
        return [redact(v) for v in value]
    if isinstance(value, str):
        return re.sub(r'(?i)(bearer\s+|sk-proj-)[A-Za-z0-9_.-]+', r'\1[REDACTED]', value)
    return value


def run(label, argv, seconds=15, input_text=None):
    started = time.time()
    process = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
    timed_out = False
    try:
        stdout, stderr = process.communicate(None if input_text is None else input_text.encode(), timeout=seconds)
    except subprocess.TimeoutExpired:
        timed_out = True
        os.killpg(process.pid, signal.SIGTERM)
        try:
            stdout, stderr = process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            stdout, stderr = process.communicate(timeout=3)
    raw = {'stdout': stdout.decode(errors='replace'), 'stderr': stderr.decode(errors='replace')}
    path = PRIVATE / (label + '.json')
    path.write_text(json.dumps(raw, indent=2) + '\n')
    path.chmod(0o600)
    decoded = {}
    for key, text in raw.items():
        try:
            decoded[key] = json.loads(text)
        except ValueError:
            decoded[key] = text
    item = dict(label=label, argv=argv, timeout_seconds=seconds, timed_out=timed_out,
                returncode=process.returncode, wall_seconds=time.time()-started,
                raw_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), **redact(decoded))
    entries = json.loads(TRANSCRIPT.read_text()) if TRANSCRIPT.exists() else []
    entries.append(item)
    TRANSCRIPT.write_text(json.dumps(entries, indent=2)+'\n')
    print(json.dumps({'label': label, 'returncode': process.returncode, 'timed_out': timed_out,
                      'wall_seconds': item['wall_seconds']}), flush=True)
    return decoded.get('stdout'), item


def tool(connection, label, operation, params=None):
    data, _ = run(label, CLI + ['tool', connection, operation, '--params', json.dumps(params or {})])
    if not isinstance(data, dict) or not data.get('ok'):
        raise RuntimeError(label + ' failed; inspect redacted transcript')
    return data['result']


if __name__ == '__main__':
    plan = json.loads((SESSION / 'plan.json').read_text())
    opened, _ = run('ordinary-question', CLI + ['--store', str(STORE), '--question', plan['question'],
                                             '--constraints', json.dumps(plan['constraints'])], 35)
    if not isinstance(opened, dict) or not opened.get('ok'):
        raise SystemExit('Ordinary entry failed; no live result is claimed')
    attachment = opened['result']['attachment']
    connection, program = attachment['connection'], attachment['program_id']
    state = dict(connection=connection, program=program, manager_session=attachment['session'])
    (PRIVATE/'state.json').write_text(json.dumps(state,indent=2)+'\n')
    caps = tool(connection, 'capabilities', 'capabilities')
    question = opened['result']['overview']['program']['question_id']
    for route in plan['routes']:
        tool(connection, route['id'], 'approach', dict(question_id=question, title=route['method'],
             reason=route['purpose'], record_id=route['id']))
        tool(connection, route['id']+'-plan', 'route_plan', dict(
            approach_id=route['id'], hypothesis=route['purpose'], scope={'machine':'HP3','same_measurements':True},
            priority=10 if route['id']=='fit-route' else 20, estimated_cost={'wall_minutes':2},
            uncertainty='No new managed scientific result yet', decision_value='Separate fit behavior from membership effects',
            novelty='Integration pilot on saved evidence', observation=route['method'],
            reopening_condition='Real launcher and separate client review work within the saved-data scope',
            reason=route['purpose'], retry_limit=1, record_id=route['id']+'-plan'))
    tool(connection,'fit-assignment','assignment',dict(approach_id='fit-route',
        objective='Run one production fit on at most 30 points of autocal/data/references/five_points_bigger_deltas.json, with one restart and max_iterations 25. Record selected-input hash, actual points, cost, nfev and termination. This is bounded integration evidence; do not claim calibration accuracy or improvement without a fair comparator.',
        allocation={'wall_minutes':2,'fresh_fits':1,'max_fit_evaluations':25,'points':30,'physics_steps':0,'collections':0},record_id='fit-assignment'))
    tool(connection,'fit-ready','transition',dict(record_id='fit-assignment',target='ready',reason='One bounded saved-data production fit'))
    tool(connection,'saved-assignment','assignment',dict(approach_id='membership-route',
        objective='Inspect CM-E1 results.json and report.md, verify saved inputs and scoped negative membership-default decision. Preserve original scope and do not rerun science.',
        allocation={'wall_minutes':2,'fresh_fits':0,'physics_steps':0,'collections':0},record_id='saved-assignment'))
    tool(connection,'saved-ready','transition',dict(record_id='saved-assignment',target='ready',reason='Saved-evidence route prepared, not dispatched'))
    allocation = tool(connection,'allocate','allocate',dict(task_limit=2,wall_minutes=8,review_minutes=1,cleanup_seconds=20))
    state['allocation']=allocation['id']
    attempt = tool(connection,'worker-start','worker_start',dict(assignment_id='fit-assignment',allocation_id=allocation['id'],
        settings={'backend':'codex','viewer':'none','record':False,'machine':'hp3','worktree':True},
        instructions='Call research_preflight on attached supervisor before production work. Keep the saved-data point/evaluation caps. Stop at bounds; retain errors. No alternate auth, standalone substitute or numerical accuracy overclaim.'))
    state['attempt']=attempt['id']
    (PRIVATE/'state.json').write_text(json.dumps(state,indent=2)+'\n')
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        observed=tool(connection,'worker-status-'+str(time.time_ns()),'worker_status',dict(attempt_id=attempt['id']))
        if observed['details']['state'] not in ('starting','running','cancelling'):
            break
        time.sleep(.2)
    else:
        tool(connection,'worker-cancel','worker_cancel',dict(attempt_id=attempt['id']))
    tool(connection,'overview-after-worker','overview')
    run('dedicated-codex',CLI+['--store',str(STORE),'--codex','--program',program,'--','exec','--json'],15)
    run('serve-attachment',CLI+['--store',str(STORE),'--serve','--program',program],15)
    print(json.dumps({'program':program,'attempt':attempt['id'],'state_file':str(PRIVATE/'state.json')}))
