"""Duplicate compact review view; full raw archives remain authoritative."""
from pathlib import Path
import json,hashlib,gzip,tarfile,io,shutil
S=Path(__file__).resolve().parent;R=Path.cwd();D=R/'research/programs/program-0a82df7fd4b34b8181716630492a1af4/lower-force';M=json.loads((S/'evidence/measurements.json').read_text());assert len(M) in {4,6}
sha=lambda b:hashlib.sha256(b).hexdigest();members=[];omissions=[];raw_refs=[]
for name in M:
 manifest=json.loads((D/name/'raw-artifact-manifest.json').read_text());raw_refs.append({'run':name,'manifest':str((D/name/'raw-artifact-manifest.json').relative_to(R)),'groups':[{'gzip_sha256':g['gzip_sha256'],'parts':g['parts']} for g in manifest['groups']]})
 for g in manifest['groups']:
  for f in g['files']:
   if Path(f['path']).name=='events.jsonl':omissions.append({**f,'reason':'Duplicate compact view omits large complete command/noise events.jsonl; exact full file is retained in the pushed per-run raw archive.'})
   else:members.append({**f,'source':S/f['path']})
for rel in ['report.md','abstract.md','research.md','policy-frozen.json','budget-ledger.json','evidence/measurements.json','evidence/pairs.json','evidence/decision.json','verify_result.py','package_run.py','write_report.py','build_review_bundle.py','evidence/callsite-audit.md','evidence/motor-probe.json','preflight.json','socket-probe.json','check_evidence.py','evidence/n-candidate-cleanup.json','evidence/n-candidate-transport-excerpt.jsonl','evidence/n-candidate-transport-excerpt-receipt.json','continuation-handoff.md','partial-decision.json','evidence/focused-guard-tests.log']:
 p=S/rel;members.append({'path':rel,'bytes':p.stat().st_size,'sha256':sha(p.read_bytes()),'source':p})
a=D/'review-evidence.tar.gz'
with a.open('wb') as out,gzip.GzipFile(filename='',mode='wb',fileobj=out,compresslevel=6,mtime=0) as gz,tarfile.open(fileobj=gz,mode='w') as tar:
 for f in members:
  p=f['source'];assert p.stat().st_size==f['bytes'] and sha(p.read_bytes())==f['sha256'];i=tar.gettarinfo(str(p),arcname=f['path']);i.mtime=i.uid=i.gid=0;i.uname=i.gname=''
  with p.open('rb') as stream:tar.addfile(i,stream)
expected={f['path']:f for f in members}
with tarfile.open(a,'r:gz') as tar:
 for item in tar:
  b=tar.extractfile(item).read();f=expected.pop(item.name);assert len(b)==f['bytes'] and sha(b)==f['sha256']
assert not expected and a.stat().st_size<28*1024*1024
record={'tested_primary_runs':list(M),'unrun_primary_runs':[n for n in ['n-default','n-candidate','d-default','d-candidate','e-default','e-candidate'] if n not in M],'compact_view_only':True,'full_raw_omissions':[],'compact_omissions':omissions,'raw_archive_references':raw_refs,'gzip_sha256':sha(a.read_bytes()),'gzip_bytes':a.stat().st_size,'files':[{k:v for k,v in x.items() if k!='source'} for x in members],'verified_by_readback':True}
(D/'review-evidence-manifest.json').write_text(json.dumps(record,indent=2)+'\n');shutil.copyfile(D/'review-evidence-manifest.json',S/'evidence/review-evidence-manifest.json')
print(json.dumps({'files':len(members),'compact_omissions':len(omissions),'gzip_bytes':a.stat().st_size,'gzip_sha256':record['gzip_sha256'],'verified_by_readback':True}))
