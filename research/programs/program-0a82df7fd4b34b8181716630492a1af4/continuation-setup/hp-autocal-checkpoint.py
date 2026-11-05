"""Manager copy of a worker-frozen, explicitly curated checkpoint."""
import hashlib,json,shutil,sys
from pathlib import Path
state=json.load(open('/tmp/hp-autocal-program-state.json'))
session=Path(state['session2']);name=sys.argv[1]
if not name or '/' in name or name in {'.','..'}:raise ValueError('Job basename required')
source=session/'checkpoint-ready'/name
dest=Path('research/programs')/state['program']/'checkpoints'/state['attempt2']/name
if not source.is_dir():raise ValueError('Checkpoint is not ready')
private={'connection.json','tool-connection.json','connection.lease','attachment.json','attachment.toml','auth.json','launch.json','session.json'}
files=[]
for path in sorted(source.rglob('*')):
 if path.is_symlink():raise ValueError('Symlink in curated checkpoint')
 if not path.is_file():continue
 rel=path.relative_to(source)
 if path.name in private or any(x.startswith('.env') or x in {'.git','node_modules','.venv'} for x in rel.parts):raise ValueError('Private control/dependency file in checkpoint')
 if path.stat().st_size>25*1024*1024:raise ValueError('Compress large checkpoint file first')
 data=path.read_bytes();h=hashlib.sha256(data).hexdigest();target=dest/rel
 if target.exists() and target.read_bytes()!=data:raise ValueError('Checkpoint changed after copy: '+str(rel))
 target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
 if hashlib.sha256(target.read_bytes()).hexdigest()!=h:raise ValueError('Copy readback mismatch')
 files.append({'path':str(rel),'bytes':len(data),'sha256':h})
(dest/'checkpoint-manifest.json').write_text(json.dumps({'attempt':state['attempt2'],'source_revision':state['worker2_start']['details']['source_revision'],'source':str(source),'files':files,'verified_by_readback':True},indent=2)+'\n')
print(json.dumps({'directory':str(dest),'files':len(files),'bytes':sum(x['bytes'] for x in files)}))
