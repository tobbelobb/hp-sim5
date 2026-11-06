from pathlib import Path
import json,re,hashlib
p=Path('research/sessions/attempt-c18177d871fb404895e32c03363c8a5e/harness')
s=(p/'hp3-stock.usda').read_text()
changes={'SpoolA':(0,.06,0),'AnchorAL':(0,.06,0),'AnchorAR':(0,.06,0),'SpoolB':(-.09,0,0),'AnchorBL':(-.09,0,0),'AnchorBR':(-.09,0,0),'SpoolC':(.08,0,0),'AnchorCL':(.08,0,0),'AnchorCR':(.08,0,0),'SpoolD':(0,0,-.10),'AnchorD0':(0,0,-.10),'AnchorD1':(0,0,-.10),'AnchorD2':(0,0,-.10)}
for name,shift in changes.items():
 pat=r'(def (?:Xform|Circle) "'+name+r'"[\s\S]*?double3 xformOp:translate = \()([^)]*)(\))'
 m=re.search(pat,s); assert m, name
 coords=[float(x.strip()) for x in m.group(2).split(',')];new=', '.join(str(x+d) for x,d in zip(coords,shift));s=s[:m.start(2)]+new+s[m.end(2):]
scene=p/'hp3-offset8301.usda';scene.write_text(s); d=json.loads((p/'fresh6301.json').read_text());d.update(id='layout8301',split='evaluation',noise_seed=8301,scene=str(scene.resolve()),scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),motor_factors={'A':{'coulombFriction':2.0,'stictionTorque':.5},'B':{'coulombFriction':.5,'stictionTorque':1.5},'C':{'coulombFriction':1.25,'stictionTorque':2.0},'D':{'coulombFriction':1.5,'stictionTorque':.75}});(p/'layout8301.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps({'changes_m':changes,'scene_hash':d['scene_sha256']},indent=2))
