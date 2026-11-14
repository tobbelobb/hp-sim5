"""Rebuild the baseline native engine in an isolated crate, without checkout edits."""
from pathlib import Path
import shutil,subprocess
s=Path('research/sessions/attempt-64d28c7e5e6c4e778656afa4639a46c6');b=s/'scratchbook/rebuild-old';b.mkdir(parents=True,exist_ok=False)
(b/'scripts').mkdir();shutil.copy('scripts/build_native_physics.mjs',b/'scripts')
shutil.copytree('src/wasm/cable_solver',b/'src/wasm/cable_solver',ignore=shutil.ignore_patterns('target'))
(b/'src/wasm/cable_solver/src/engine.rs').write_bytes(subprocess.check_output(['git','show','2c0b86ff:src/wasm/cable_solver/src/engine.rs']))
subprocess.run(['node',str(b/'scripts/build_native_physics.mjs'),'--no-pgo','--output',str((b/'old.node').resolve())],check=True)
print((b/'old.node').resolve())
