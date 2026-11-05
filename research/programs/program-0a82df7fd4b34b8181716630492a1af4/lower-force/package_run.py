"""Archive one stopped run, all journals and fits; verify every original byte."""
from pathlib import Path
import sys,json,hashlib,tarfile,gzip,re,shutil,subprocess,datetime
R=Path.cwd();S=Path(__file__).resolve().parent;name=sys.argv[1];run=S/'runs'/name
assert run.is_dir() and name not in {'.','..'} and '/' not in name
D=R/'research/programs/program-0a82df7fd4b34b8181716630492a1af4/lower-force'/name;D.mkdir(parents=True,exist_ok=True)
shutil.copyfile(S/(name+'.stdout.log'),run/'command.stdout.log')
for name0 in ['policy-frozen.json','candidate.patch','fit-ledger.json']:
 if (S/name0).exists():shutil.copyfile(S/name0,run/name0)
shutil.copytree(S/'harness',run/'harness-source',dirs_exist_ok=True)
(run/'setup-evidence').mkdir(exist_ok=True)
for src in [S/'preflight.json',S/'socket-probe.json',S/'evidence/motor-probe.json',S/'evidence/callsite-audit.md',S/'verify_result.py',S/'package_run.py']:
 shutil.copyfile(src,run/'setup-evidence'/src.name)
metrics=json.loads((S/'evidence/measurements.json').read_text())[name]
(D/'result.json').write_text(json.dumps(metrics,indent=2)+'\n')
for name0 in ['policy-frozen.json','candidate.patch']:
 shutil.copyfile(S/name0,D/name0)
private={'connection.json','tool-connection.json','connection.lease','attachment.json','attachment.toml','auth.json','launch.json','session.json'}
secret=re.compile(rb'(?i)(?:Bearer\s+[A-Za-z0-9_\-.]{20,}|sk-proj-[A-Za-z0-9_-]+|"(?:api_key|access_token|auth_token|bearer_token|authorization|password)"\s*:\s*"[^"\r\n]+")')
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  while b:=f.read(1048576):h.update(b)
 return h.hexdigest()
files=[]
for p in sorted(run.rglob('*')):
 if not p.is_file():continue
 assert not p.is_symlink() and p.name not in private,p
 if p.suffix.lower() in {'.json','.jsonl','.md','.txt','.log','.py','.mjs','.js','.toml'}:
  with p.open('rb') as f:
   assert not any(secret.search(line) for line in f),p
 files.append({'path':str(p.relative_to(S)),'bytes':p.stat().st_size,'sha256':sha(p)})
a=D/'raw-run.tar.gz'
with a.open('wb') as out,gzip.GzipFile(filename='',mode='wb',fileobj=out,compresslevel=3,mtime=0) as gz,tarfile.open(fileobj=gz,mode='w') as tar:
 for r in files:
  p=S/r['path'];i=tar.gettarinfo(str(p),arcname=r['path']);i.mtime=i.uid=i.gid=0;i.uname=i.gname=''
  with p.open('rb') as f:tar.addfile(i,f)
expected={r['path']:r for r in files}
with tarfile.open(a,'r:gz') as tar:
 for m in tar:
  h=hashlib.sha256()
  with tar.extractfile(m) as f:
   while b:=f.read(1048576):h.update(b)
  assert h.hexdigest()==expected.pop(m.name)['sha256'],m.name
assert not expected
record={'name':name,'uncompressed_bytes':sum(r['bytes'] for r in files),'gzip_sha256':sha(a),'files':files,'parts':[{'path':a.name,'bytes':a.stat().st_size,'sha256':sha(a)}]}
assert a.stat().st_size<90*1024*1024,'split required'
(D/'raw-artifact-manifest.json').write_text(json.dumps({'session':str(S),'source_revision':'a68d6adffec3aea86a55a17d6b4d65a27badd42a','omitted':[],'groups':[record],'verified_by_readback':True},indent=2)+'\n')
m=metrics.get('applied') or {};summary=f"# {name}\n\n{metrics['category']}; stop={metrics['stop_reason']}; normal acceptance={metrics['normal_automatic_acceptance']}.\n\nApplied per-anchor error mm: {m.get('per_anchor_error_mm')}. Maximum {m.get('max_mm')}, sqrt(mean of four squared anchor distances) {m.get('rmse_mm')}. Every<5: {m.get('all_below5')}; anchors>20: {m.get('anchors_over20')}.\n\n{metrics['physical_collected_points']} physical points, {metrics['sweeps']} sweeps, {metrics['fit_count']} fits, {metrics['total_steps']} steps, {metrics['simulated_s']} simulated seconds, {metrics['wall_s']} wall seconds. Wall time depends on host load. Intermediate max tail {metrics['iteration_tail_max_mm']} mm, invalid fits {metrics['invalid_fits']}.\n\nFull raw archive includes all fits, dataset, selection log, command/noise journals, initial/terminal native snapshots, invocation, candidate patch, policy/source hashes. SHA256 readback verified every archived file. No omissions.\n"
(D/'README.md').write_text(summary)
allm=json.loads((S/'evidence/measurements.json').read_text());ledger={'limit':json.loads((S/'policy-frozen.json').read_text())['budget'],'completed_or_failed_collections':len(allm),'fresh_fits':sum(x['fit_count'] for x in allm.values()),'physics_steps':sum(x['total_steps'] for x in allm.values())+3003,'physical_points':sum(x['physical_collected_points'] for x in allm.values()),'torque_probe_steps':3003,'runs':list(allm)}
(S/'budget-ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
(S/'research.md').open('a').write('\n## '+name+'\n\n'+summary.split('\n\n',1)[1])
print(json.dumps({'run':name,'files':len(files),'raw_bytes':record['uncompressed_bytes'],'gzip_bytes':a.stat().st_size,'verified_by_readback':True,'budget':ledger}))
