"""Rebuild catalogue print streams through the actual RRF and Klipper planners.

.venv/bin/python scripts/compile_machine_commands.py --machine hp5
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import machines


def compile_machine(name):
    machine = machines.machine(name)
    with tempfile.TemporaryDirectory(prefix='compile-machine-') as temporary:
        work = Path(temporary)
        shutil.copytree(ROOT / 'RRF/run/vsd/sys', work / 'sys')
        (work / 'gcodes').mkdir()
        for gcode, outputs in machine['compiled_commands'].items():
            source = ROOT / gcode
            shutil.copyfile(source, work / 'gcodes' / source.name)
            if 'rrf' in outputs:
                capture = work / f'{source.stem}.csv'
                subprocess.run([str(ROOT / 'RRF/build/rrf_simulator'), '--vsd', str(work),
                    '--gcode', f'gcodes/{source.name}', '-c', 'sys/' + Path(machine['rrf_config']).name,
                    '--can-log', str(capture)], cwd=ROOT, check=True)
                subprocess.run([sys.executable, str(ROOT / 'RRF/csv2can.py'), str(capture)], check=True)
                target = ROOT / outputs['rrf']
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(capture.with_suffix('.can'), target)
            if 'klipper' in outputs:
                target = ROOT / outputs['klipper']
                if target.suffix != '.serial':
                    raise ValueError('Use binary .serial catalogue outputs for Klipper compilation')
                target.parent.mkdir(parents=True, exist_ok=True)
                subprocess.run([sys.executable, 'klipper/klippy/klippy.py', machine['klipper_config'],
                    '-i', gcode, '-o', str(target), '-d', 'public/klipper/linux_mcu/klipper.dict'],
                    cwd=ROOT, check=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--machine', action='append', required=True)
    for name in parser.parse_args().machine:
        compile_machine(name)
