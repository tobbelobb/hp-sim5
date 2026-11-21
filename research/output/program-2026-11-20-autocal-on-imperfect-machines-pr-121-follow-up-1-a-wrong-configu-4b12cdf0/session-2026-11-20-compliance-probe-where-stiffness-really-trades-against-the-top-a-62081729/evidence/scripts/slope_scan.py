"""Where does assumed compliance trade against top-anchor height? Offline, existing nominal datasets.

usage: slope_scan.py MACHINE DATASET FITTER [k ...]
FITTER: sweep (the sweep autocal's two-stage point fit) or ellipsoid (ellipsoid_autocal.fit).
Every compliance the fit takes from config.g (M666 S: dataset_stretch's flex model and
HP4's ABC home strain) is multiplied by k. The data were collected with true = configured
EA, so k plays the role of configured/true compliance: the error at k is, to first order,
the error a stiffness mismatch of that ratio would give. Errors are in the print
convention frame (autocal.tools.imperfect_machine.convention_frame).
"""
import copy, json, sys
import numpy as np
import autocal.pose_fit as pf
from autocal.flex import FlexModel
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.anchor_seed import build_anchor_initial_guess
from autocal.tests.test_disturbed_robustness import TRUTH, _fit as sweep_fit
from autocal.ellipsoid_autocal import fit, SEED_SCALES
from autocal.tools.imperfect_machine import convention_frame

machine, path, fitter = sys.argv[1:4]
ks = [float(v) for v in sys.argv[4:]] or [0.8, 1.0, 1.2]
base = json.load(open(path)); base.setdefault('machine_type', machine); normalize_dataset_point_roles(base)
truth = np.asarray(TRUTH[machine], float); truth_frame = convention_frame(truth)[0]
from_m666, home_strain = FlexModel.from_m666.__func__, pf._hp4_home_strain
rows = []
for k in ks:
    FlexModel.from_m666 = classmethod(lambda cls, m666, *, num_axes, spring_k_multiplier=1.0:
                                      from_m666(cls, m666, num_axes=num_axes, spring_k_multiplier=spring_k_multiplier / k))
    pf._hp4_home_strain = lambda d: (lambda h: None if h is None else (h[0] * k, h[1]))(home_strain(d))
    try:
        dataset = copy.deepcopy(base)
        if fitter == 'sweep':
            result = sweep_fit(dataset)
        else:
            seed = build_anchor_initial_guess(dataset, machine_type=machine)
            result = fit(dataset, [seed * s for s in SEED_SCALES], True, fitter)
    finally:
        FlexModel.from_m666, pf._hp4_home_strain = classmethod(from_m666), home_strain
    f = convention_frame(np.asarray(result['anchors']))[0] - truth_frame
    rows.append(dict(k=k, Dz=float(f[-1, 2]), low_z=float(f[:-1, 2].mean()), max=float(np.abs(f).max()),
                     rms=float(result['rms_mm']), corrected=bool(result.get('stretch_corrected'))))
slope = np.polyfit([r['k'] for r in rows], [r['Dz'] for r in rows], 1)[0]
print(json.dumps(dict(machine=machine, dataset=path.split('/')[-2], fitter=fitter, dDz_dk_mm=round(float(slope), 3),
                      rows=[{key: (round(v, 4) if isinstance(v, float) else v) for key, v in r.items()} for r in rows])))
