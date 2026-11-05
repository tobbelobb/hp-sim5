"""Archive a stopped numerical run and verify its exact original bytes."""
import gzip,hashlib,json,re,sys,tarfile
from pathlib import Path
s=json.load(open('/tmp/hp-autocal-program-state.json'));session=Path(s['session2']);job=sys.argv[1]
if not job or '/' in job or job in {'.','..'}:raise ValueError('Job basename required')
run=session/'runs'/job;dest=Path('research/programs')/s['program']/'checkpoints'/s['attempt2']/job
if not run.is_dir() or not dest.is_dir():raise ValueError('Copy curated completed checkpoint first')
private={'connection.json','tool-connection.json','connection.lease','attachment.json','attachment.toml','auth.json','launch.json','session.json'}
secret=re.compile(rb'(?i)(?:Bearer\s+[A-Za-z0-9_\-.]{20,}|sk-proj-[A-Za-z0-9_-]+|"(?:api_key|access_token|auth_token|bearer_token|authorization|password)"\s*:\s*"[^"\r\n]+")')
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  while b:=f.read(1024*1024):h.update(b)
 return h.hexdigest()
files=[]
for p in sorted(run.rglob('*')):
 if not p.is_file():continue
 rel=p.relative_to(session)
 if p.is_symlink() or p.name in private:raise ValueError('Unexpected control file in run')
 if p.suffix.lower() in {'.json','.jsonl','.md','.txt','.log','.py','.mjs','.js','.toml'}:
  with p.open('rb') as f:
   if any(secret.search(line) for line in f):raise ValueError('Credential-like text found; stop before writing archive')
 files.append({'path':str(rel),'bytes':p.stat().st_size,'sha256':sha(p)})
a=dest/'raw-job.tar.gz'
with a.open('wb') as out,gzip.GzipFile(filename='',mode='wb',fileobj=out,compresslevel=3,mtime=0) as gz,tarfile.open(fileobj=gz,mode='w') as tar:
 for r in files:
  p=session/r['path'];i=tar.gettarinfo(str(p),arcname=r['path']);i.mtime=i.uid=i.gid=0;i.uname=i.gname=''
  with p.open('rb') as f:tar.addfile(i,f)
expected={r['path']:r for r in files}
with tarfile.open(a,'r:gz') as tar:
 for m in tar:
  h=hashlib.sha256()
  with tar.extractfile(m) as f:
   while b:=f.read(1024*1024):h.update(b)
  assert h.hexdigest()==expected.pop(m.name)['sha256'],m.name
assert not expected
if a.stat().st_size>90*1024*1024:raise ValueError('Archive needs splitting before commit')
record={'name':job,'uncompressed_bytes':sum(r['bytes'] for r in files),'gzip_sha256':sha(a),'files':files,'parts':[{'path':a.name,'bytes':a.stat().st_size,'sha256':sha(a)}]}
(dest/'raw-artifact-manifest.json').write_text(json.dumps({'session':str(session),'source_revision':s['worker2_start']['details']['source_revision'],'omitted':[],'groups':[record],'verified_by_readback':True},indent=2)+'\n')
print(json.dumps({'job':job,'files':len(files),'raw_bytes':record['uncompressed_bytes'],'gzip_bytes':a.stat().st_size,'verified_by_readback':True}))
