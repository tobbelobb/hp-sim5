from pathlib import Path
import shutil, subprocess
s=Path('research/sessions/attempt-0b9a865f76514b04930528d43f2c31ea'); b=s/'scratchbook/buildroot'
(b/'scripts').mkdir(parents=True, exist_ok=True)
shutil.copy('scripts/build_native_physics.mjs',b/'scripts')
shutil.copytree('src/wasm/cable_solver',b/'src/wasm/cable_solver',ignore=shutil.ignore_patterns('target'),dirs_exist_ok=True)
(s/'evidence/native').mkdir(parents=True,exist_ok=True)
for label in ['current','old']:
 if label=='old':
  (b/'src/wasm/cable_solver/src/engine.rs').write_bytes(subprocess.check_output(['git','show','2c0b86ff:src/wasm/cable_solver/src/engine.rs']))
 with (s/f'evidence/build-{label}.log').open('w') as log:
  subprocess.run(['node',str(b/'scripts/build_native_physics.mjs'),'--no-pgo','--output',str((s/f'evidence/native/{label}.node').resolve())],check=True,stdout=log,stderr=subprocess.STDOUT)
