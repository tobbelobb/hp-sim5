import json,re,sys,hashlib
from pathlib import Path
scene,baked=map(Path,sys.argv[1:]);names=['SpoolA','AnchorAL','AnchorAR','SpoolB','AnchorBL','AnchorBR','SpoolC','AnchorCL','AnchorCR','SpoolD','AnchorD0','AnchorD1','AnchorD2'];out=[]
for name in names:
 pat=r'def (?:Xform|Circle) "'+name+r'"[\s\S]*?double3 xformOp:translate = \(([^)]*)\)';a=re.search(pat,scene.read_text());b=re.search(pat,baked.read_text());assert a and b,name
 va=[float(x) for x in a.group(1).split(',')];vb=[float(x) for x in b.group(1).split(',')];out.append({'name':name,'authored_m':va,'baked_m':vb,'equal':all(abs(x-y)<1e-12 for x,y in zip(va,vb))})
print(json.dumps({'authored_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),'baked_sha256':hashlib.sha256(baked.read_bytes()).hexdigest(),'entities':out,'all_equal':all(r['equal'] for r in out)},indent=2))
