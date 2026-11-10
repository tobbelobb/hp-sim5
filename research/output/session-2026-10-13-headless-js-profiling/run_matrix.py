from pathlib import Path
import subprocess,json,time
ROOT=Path.cwd()
OUT=ROOT/'output/research/sessions/2026-10-13-headless-js-profiling'
for machine in ['hp3','hp4']:
 for workload in ['logo','autocal']:
  for layering in ['on','off']:
   name=f'{machine}-{workload}-{layering}'
   flag='--commands' if workload=='logo' else '--events'
   file=OUT/'inputs'/f'{machine}-{workload}.{ "json" if workload=="logo" else "jsonl" }'
   common=['scripts/profile_headless.mjs','--machine',machine,flag,str(file),'--layering',layering]
   baseline=OUT/f'{name}-plain.json'
   for mode in ['plain','systems','cpu','heap']:
    result=OUT/f'{name}-{mode}.json'
    command=['node']
    if mode=='cpu': command+=['--cpu-prof',f'--cpu-prof-dir={OUT}',f'--cpu-prof-name={name}.cpuprofile']
    command+=common+['--repeats','3' if mode=='plain' else '1','--output',str(result)]
    if mode!='plain':command+=['--compare',str(baseline)]
    if mode=='systems':command+=['--systems']
    if mode=='heap':command+=['--heap-sampling']
    print('START',name,mode,flush=True)
    with (OUT/f'{name}-{mode}.log').open('w') as log:
     run=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=600)
    print((OUT/f'{name}-{mode}.log').read_text(),flush=True)
    if run.returncode:raise RuntimeError(f'{name} {mode} failed: {run.returncode}')
    if mode in ['cpu','heap']:
     prof=OUT/f'{name}.cpuprofile' if mode=='cpu' else Path(str(result)+'.0.heapprofile')
     cmd=['node','scripts/summarize_v8_profile.mjs',str(prof)]
     if mode=='cpu':cmd+=[str(result)]
     with (OUT/f'{name}-{mode}-summary.json').open('w') as output:
      subprocess.run(cmd,stdout=output,check=True,cwd=ROOT)
print('COMPLETE',flush=True)
