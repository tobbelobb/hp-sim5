"""Real client/review/graph follow-up after the recorded worker startup failure."""
import hashlib
import json
from pathlib import Path
import time
from probe import ROOT, SESSION, PRIVATE, STORE, CLI, run, tool, redact

state = json.loads((PRIVATE/'state.json').read_text())
connection, program, attempt = (state[k] for k in ('connection','program','attempt'))
if not any(x['label']=='dedicated-codex-ignore-user-config' for x in json.loads((SESSION/'cli-transcript.json').read_text())):
    run('dedicated-codex-ignore-user-config',CLI+['--store',str(STORE),'--codex','--program',program,
        '--','exec','--ignore-user-config','--json'],15)

worker = STORE/'workers'/attempt
for name in ('stdout.log','stderr.log','assignment.json','prompt.txt'):
    text=(worker/name).read_text()
    try:
        safe=json.dumps(redact(json.loads(text)),indent=2)+'\n'
    except ValueError:
        safe=redact(text)
    (SESSION/('worker-'+name)).write_text(safe)
partial = '''# Startup-only partial evidence

The real controller dispatched attempt-c4f103629fb948949279aa671a03b84d through
hp-sim5-research-agent in an isolated worktree. Its native build failed with
missing Node-API headers, exit 1. The launcher recorded finally cleanup.
No production fitting/physics/collection operation or worker coding conversation
ran. Both scientific hypotheses remain not tested. This manager-written startup
report may support an engineering blocker review only. It is not a completed
scientific investigation or a worker-authored finding.
'''
(SESSION/'startup-partial.md').write_text(partial)
(SESSION/'startup-abstract.md').write_text('# Startup-only blocker\n\nStatus: blocked.\n\n'+partial.split('\n\n',1)[1])
previous=json.loads((SESSION/'cli-transcript.json').read_text())
claim=next((x['stdout']['result'] for x in previous if x['label']=='startup-claim'),None)
if claim is None:
    claim=tool(connection,'startup-claim','claim',dict(title='The pilot worker stopped at native build before science; both hypotheses are not tested',
        scope={'type':'engineering-startup-only','attempt_id':attempt},record_id='startup-only'))
submission=next((x['stdout']['result'] for x in previous if x['label']=='startup-submit'),None)
if submission is None:
    submission=tool(connection,'startup-submit','submit',dict(attempt_id=attempt,claim_id=claim['id'],
    base_revision='b15697fb50edd4ac2c358051c77ddce57abeacea',config={'scope':'startup-only','science':'not-tested'},
    input_identities=['git:b15697fb50edd4ac2c358051c77ddce57abeacea'],producer_checks=[],selected=[
        {'id':'report','role':'report','path':str(SESSION/'startup-partial.md')},
        {'id':'abstract','role':'abstract','path':str(SESSION/'startup-abstract.md')},
        {'id':'stderr','role':'evidence','path':str(SESSION/'worker-stderr.log')}]))
tool(connection,'fund-startup-review-after-host-reload','fund_review',dict(submission_id=submission['id'],wall_minutes=.25,cleanup_seconds=5))
review=tool(connection,'startup-review-start','review_start',dict(submission_id=submission['id'],backend='codex',wall_minutes=.25))
state.update(submission=submission['id'],review=review['id'])
deadline=time.monotonic()+18
while time.monotonic()<deadline:
    observed=tool(connection,'review-status-'+str(time.time_ns()),'review_status',dict(run_id=review['id']))
    if observed['details']['state'] not in ('running','cancelling'):
        break
    time.sleep(.5)
for name in ('stdout.log','stderr.log'):
    path=STORE/'reviews'/review['id']/name
    if path.exists():
        (SESSION/('review-'+name)).write_text(redact(path.read_text()))
request={'jsonrpc':'2.0','id':1,'method':'initialize','params':{'protocolVersion':'2025-11-25',
    'capabilities':{},'clientInfo':{'name':'task016-stock-stdio-probe','version':'1'}}}
run('stock-mcp-initialize',[str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/research_program_mcp.py'),connection],8,json.dumps(request)+'\n')

# Existing imported evidence stays unreviewed. This is a counterfactual graph exercise.
graph_store=PRIVATE/'graph-store'
run('graph-import',CLI+['--store',str(graph_store),'import','research/program-graph/autocal-seed.json'])
run('graph-support-before',CLI+['--store',str(graph_store),'query','autocal-evidence','support','keep-defaults'])
run('graph-withdraw-counterfactual',CLI+['--store',str(graph_store),'review-event','autocal-evidence',
    'Task016 counterfactual only; no real measurement retraction','--subject','cm-regressions','--event','withdrawal'])
run('graph-support-after',CLI+['--store',str(graph_store),'query','autocal-evidence','support','keep-defaults'])
run('graph-export',CLI+['--store',str(graph_store),'export','autocal-evidence','--format','markdown'])

tool(connection,'knowledge-query','query',dict(query='support',subject_id='startup-only'))
tool(connection,'record-direction','steering',dict(text='Preserve startup failure and suspend numerical dispatch until the documented prerequisites work',
    intent='instruction',provenance={'source':'task016 release-gate constraints','actor':'manager recording existing direction'}))
tool(connection,'pause','pause',dict(reason='Actual worker and client prerequisites prevent the pilot'))
tool(connection,'resume','resume',dict(reason='Inspect durable state and stop this finite pilot without scientific admission'))
tool(connection,'recover','recover')
tool(connection,'stop','stop',dict(reason='Live release blocked; preserve partial evidence and stop owned work'))
tool(connection,'shutdown','shutdown')
time.sleep(.3)
reopened,_=run('fresh-cli-reopen',CLI+['--store',str(STORE),'--program',program])
new_connection=reopened['result']['attachment']['connection']
tool(new_connection,'fresh-cli-overview','overview')
tool(new_connection,'fresh-cli-shutdown','shutdown')
run('program-export',CLI+['--store',str(STORE),'export',program])
(PRIVATE/'state.json').write_text(json.dumps(state,indent=2)+'\n')
print(json.dumps({'program':program,'attempt':attempt,'review':review['id'],'submission':submission['id']}))
