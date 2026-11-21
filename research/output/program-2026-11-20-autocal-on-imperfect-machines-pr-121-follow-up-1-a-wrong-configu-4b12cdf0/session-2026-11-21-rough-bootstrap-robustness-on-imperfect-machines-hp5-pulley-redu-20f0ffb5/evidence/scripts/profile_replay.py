"""Rewrite a run's bootstrap profiles with this checkout's equal_strain and compare to the recorded ones."""
import json, sys, tempfile
from pathlib import Path
import numpy as np
from autocal.rough_bootstrap import assess_rough_geometry
from autocal.active_learning import dataset_sweep_configs
from autocal.equal_strain import write_sensor_force_profile
for run in map(Path, sys.argv[1:]):
    rough = json.loads((run / 'sweeps.rough_bootstrap.json').read_text())
    anchors, _ = assess_rough_geometry(rough)
    out = []
    for index, cfg in enumerate(dataset_sweep_configs(rough), start=1):
        recorded = run / f'sweeps.bootstrap_force_{index}.json'
        if not recorded.exists(): break
        with tempfile.NamedTemporaryFile(suffix='.json') as tmp:
            write_sensor_force_profile(Path(tmp.name), rough, anchors, [cfg])
            new = json.loads(Path(tmp.name).read_text())
        old = json.loads(recorded.read_text())
        same = new == old
        out.append('same' if same else f"DIFF target {old[0]['strand_tension_n']:.3f}->{new[0]['strand_tension_n']:.3f} pts {len(old[0]['force_n'])}->{len(new[0]['force_n'])}")
    print(run.name, out)
