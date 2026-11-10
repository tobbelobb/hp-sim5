import json,os,socket,subprocess,sys,time,urllib.request
from pathlib import Path
ROOT=Path.cwd()
SESSION=ROOT/'output/research/2026-10-09-slip-recovery-and-encoder-model'
label=sys.argv[1]
run=SESSION/label
run.mkdir(exist_ok=False)
def port():
 with socket.socket() as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]
rrf_port,ws_port,api_port=port(),port(),port()
processes=[]; logs=[]
try:
 for name,cmd in [('rrf',[str(ROOT/'RRF/build/rrf_simulator'),'--vsd','RRF/run/vsd','-c','sys/config_hp3.g','--server','-p',str(rrf_port)])]:
  log=(run/(name+'.log')).open('w');logs.append(log);processes.append(subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True))
 for _ in range(200):
  try:
   urllib.request.urlopen(f'http://127.0.0.1:{rrf_port}/rr_model?key=state',timeout=1);break
  except OSError:time.sleep(.1)
 else:raise RuntimeError('RRF startup timeout')
 log=(run/'physics-service.log').open('w');logs.append(log)
 processes.append(subprocess.Popen(['node',str(SESSION/'physics-service.mjs'),str(SESSION/'scene.usda'),f'http://127.0.0.1:{rrf_port}',str(ws_port),str(api_port),str(run)],stdout=log,stderr=log,start_new_session=True))
 for _ in range(200):
  try:urllib.request.urlopen(f'http://127.0.0.1:{api_port}/status',timeout=1);break
  except OSError:time.sleep(.1)
 else:raise RuntimeError('Physics service startup timeout')
 force='1.1363575731199993' if label=='reduced-force' else '12.884911335493843'
 cmd=['node','autocal/control/cli/collect_sweep_data.mjs','--headless-url',f'http://127.0.0.1:{api_port}','--sim','--no-spawn-rrf-simulator','--no-trace','--machineType','hangprinter_4','--sweep-config-file',str(SESSION/'upper.cfg'),'--fixed-targets','0,-489.0747667304494','--force-base-radii','30','--force-buildup-factor','0.636619','--force-low','0.011363575731199994','--force-mid','0.22727151462399986','--force-max',force,'--no-auto-tune-force','--output-file',str(run/'sweeps.json'),'--return-to-origin']
 if label=='reduced-force':cmd+=['--sensor-collection-force',force]
 if label.startswith('patched-464'):cmd[cmd.index('--fixed-targets')+1]='0,-464.62102839392696'
 if label.startswith('overload-guard'):cmd+=['--sensor-collection-force','6.442455667746922']
 if label.startswith('patched-bootstrap'):
  start=cmd.index('--fixed-targets');del cmd[start:start+2]
  cmd[cmd.index('--sweep-config-file')+1]=str(ROOT/'output/extended-autocal/trial-004/sweeps.bootstrap_cfg.txt')
  if label!='patched-bootstrap-auto-size':cmd+=['--max-travel-mm','489.0747667304494']
 (run/'command.json').write_text(json.dumps(cmd,indent=2))
 env=dict(os.environ);env.pop('AUTOCAL_REFERENCE_WS',None)
 with (run/'collector.log').open('w') as log:
  result=subprocess.run(cmd,env=env,stdout=log,stderr=log,timeout=300)
 (run/'result.json').write_text(json.dumps({'returncode':result.returncode,'command':cmd},indent=2))
 print(label,result.returncode,flush=True)
finally:
 for process in reversed(processes):
  if process.poll() is None:
   process.terminate()
   try:process.wait(timeout=10)
   except subprocess.TimeoutExpired:process.kill();process.wait(timeout=10)
 for log in logs:log.close()
