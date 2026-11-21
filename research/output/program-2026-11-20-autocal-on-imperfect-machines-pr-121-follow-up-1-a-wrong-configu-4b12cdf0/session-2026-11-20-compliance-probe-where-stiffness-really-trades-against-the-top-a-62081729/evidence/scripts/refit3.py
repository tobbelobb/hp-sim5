"""Test the replay-derived probe corrections offline on one live run.

usage: refit3.py RUN_DIR

Corrections (section 7 of the report):
  step23  k from the 2x -> 3x step only: both levels are reached by raising the force in the probe,
          so the sweep point's own force-realisation offset (the 1x level) drops out.
  yfix    HP5 pulley-reduced: add the effector-side reeving per path (pinhole -> wheel -> wheel -> pinhole,
          85.512 mm in hp5_pulley_reduced.usda) to M666 Y of the low lines, in both the probe and the fit.
Variants: probe (as before), probe23, probe23_yfix, probe_yfix, and config / oracle with and without yfix.
"""
import copy, json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import probe
from refit2 import fit_at
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.tools.imperfect_machine import convention_frame

REEVING_MM = {'hangprinter_5_pulley_reduced': [85.512, 85.512, 85.512, 85.512, 0.0]}


def with_yfix(dataset, machine):
    out = copy.deepcopy(dataset)
    extra = REEVING_MM.get(machine)
    if extra:
        m = out['config']['m666_before_data_collection']
        m['Y'] = [y + e for y, e in zip(m['Y'], extra)]
    return out


def main():
    run = Path(sys.argv[1])
    imp = json.loads((run / 'imperfection.json').read_text())
    fitter = 'ellipsoid' if imp.get('ellipsoid_shape') else 'sweep'
    ds = json.loads((run / 'sweeps.json').read_text()); ds.setdefault('machine_type', imp['machine']); normalize_dataset_point_roles(ds)
    live = np.asarray(json.loads((run / 'result.json').read_text())['fitted_mm'], float)
    truth_frame = convention_frame(np.array(list(imp['truth_mm'].values())))[0]
    k_true = 1 / imp['stiffness_scale']
    fixed = with_yfix(ds, imp['machine'])

    def summary(data, k):
        result = fit_at(data, fitter, k, live)
        f = convention_frame(np.asarray(result['anchors']))[0] - truth_frame
        return result, dict(k=float(k), k_error=float(k / k_true - 1), max_abs_frame_mm=float(np.abs(f).max()),
                            top_z_frame_mm=float(f[-1, 2]), low_z_frame_mm=float(f[:-1, 2].mean()))

    out = dict(run=run.name, machine=imp['machine'], fitter=fitter, k_true=k_true)
    for name, data in (('', ds), ('_yfix', fixed)):
        config, out['config' + name] = summary(data, 1.0)
        anchors, home = config['anchors'], config['home_offsets_mm']
        k_all = probe.estimate(data, anchors, home)[0]
        k_23 = probe.estimate(data, anchors, home, pairs=((1, 2),))[0]
        out['probe' + name] = summary(data, k_all)[1]
        out['probe23' + name] = summary(data, k_23)[1]
        out['oracle' + name] = summary(data, k_true)[1]
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
