from pathlib import Path
import json,hashlib,tarfile,gzip,io,shutil,re
S=Path(__file__).resolve().parent;R=Path.cwd();D=R/'research/programs/program-0a82df7fd4b34b8181716630492a1af4/lower-force';M=json.loads((S/'evidence/measurements.json').read_text())
sha=lambda b:hashlib.sha256(b).hexdigest();files=[];omissions=[];refs=[];seen=set();a=D/'combined-review-evidence.tar.gz'
private={'connection.json','tool-connection.json','attachment.json','attachment.toml','launch.json','session.json','manager-question.json','manager-answer.json'}
def add(out,name,b):
 assert name not in seen and Path(name).name not in private,name
 seen.add(name);info=tarfile.TarInfo(name);info.size=len(b);info.mode=0o644;out.addfile(info,io.BytesIO(b));files.append({'path':name,'bytes':len(b),'sha256':sha(b)})
with a.open('wb') as stream,gzip.GzipFile(filename='',mode='wb',fileobj=stream,compresslevel=6,mtime=0) as gz,tarfile.open(fileobj=gz,mode='w') as out:
 for name in M:
  p=D/name/'raw-artifact-manifest.json';manifest=json.loads(p.read_text());refs.append({'run':name,'manifest':str(p.relative_to(R)),'groups':[{'gzip_sha256':g['gzip_sha256'],'parts':g['parts']} for g in manifest['groups']]})
  for g in manifest['groups']:
   assert len(g['parts'])==1
   archive=p.parent/g['parts'][0]['path'];assert sha(archive.read_bytes())==g['parts'][0]['sha256'];expected={f['path']:f for f in g['files']}
   with tarfile.open(archive,'r:gz') as raw:
    for m in raw:
     b=raw.extractfile(m).read();f=expected.pop(m.name);assert len(b)==f['bytes'] and sha(b)==f['sha256'],m.name
     if Path(m.name).name=='events.jsonl':omissions.append({**f,'run':name,'reason':'Duplicate compact view omits only this events.jsonl. Exact bytes remain in referenced full raw archive.'})
     else:add(out,m.name,b)
   assert not expected
 for rel in ['report.md','abstract.md','research.md','policy-frozen.json','budget-ledger.json','measurements.json','pairs.json','decision.json','verify_result.py','package_run.py','write_combined_report.py','build_combined_bundle.py','check_combined_evidence.py','preflight.json','socket-probe.json']:
  p=S/rel
  if p.is_file():add(out,rel,p.read_bytes())
 for p in sorted((S/'evidence').rglob('*')):
  if p.is_file() and p.name not in private and 'combined-review-evidence' not in p.name and p.name!='combined-fixed-checks.json':add(out,str(p.relative_to(S)),p.read_bytes())
expected={f['path']:f for f in files}
with tarfile.open(a,'r:gz') as t:
 for m in t:
  b=t.extractfile(m).read();f=expected.pop(m.name);assert sha(b)==f['sha256'] and len(b)==f['bytes']
assert not expected
record=dict(compact_view_only=True,tested_primary_runs=[n for n in M if n!='invalid-path-startup'],invalid_setup_runs=['invalid-path-startup'],full_raw_omissions=[],compact_omissions=omissions,raw_archive_references=refs,gzip_sha256=sha(a.read_bytes()),gzip_bytes=a.stat().st_size,files=files,verified_by_readback=True,prior_partial_review_commit='57658240')
(D/'combined-review-evidence-manifest.json').write_text(json.dumps(record,indent=2)+'\n');shutil.copyfile(D/'combined-review-evidence-manifest.json',S/'evidence/combined-review-evidence-manifest.json')
print(json.dumps({'files':len(files),'omitted_duplicate_journals':len(omissions),'bytes':a.stat().st_size,'sha256':record['gzip_sha256'],'readback_verified':True}))
