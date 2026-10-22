import subprocess,sys
from pathlib import Path
OUT=Path(__file__).resolve().parent
for variant in ('cache_actual_disabled','cache_actual_repeat'):
 with (OUT/f'E10-{variant}-progress.txt').open('w') as log:
  subprocess.run([str(Path.cwd()/'.venv/bin/python'),str(OUT/'replay.py'),variant,'layered_manual_some_bad_points','seventh_hp3_dataset'],stdout=log,stderr=subprocess.STDOUT,check=True)
 print(variant,'complete',flush=True)
