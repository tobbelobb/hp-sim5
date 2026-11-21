"""Synthetic extra force-realisation error and an emulated home-stretch term, for joint_k_cog.

PERTURB (JSON list) makes the *recorded* force of every force-mode line wrong by an extra
amount, so realised - recorded = (sim cogging) + extra; the realised physics is unchanged:
  {"ripple": h, "amp": a}            extra = U a sin(h th)            (same phase as the sim's at h = 50)
  {"ripple": h, "amp": a, "seed": s} extra = U a sin(h th + phi_m)    (random phase per motor)
  {"hyst": a}                        extra = U a * direction          (sign of that line's length change
                                                                       since the sweep's previous point:
                                                                       a hysteresis-like, direction-dependent offset)
  {"hyst": a, "two_sided": true}     as hyst, plus every sweep repeated from the other direction
                                     (same poses, offset of the opposite sign): two-sided approach
                                     at twice the collection
a is spool-side N, U the line's M666 U, th the motor's encoder angle.

HOMESTRAIN=1 emulates the commissioned home-stretch default on equal-strain sweep datasets:
the low lines' home offsets get k * T_strand / S * (|anchor| + Y / U), anchors fixed at the live
fit's (the term varies by < 1e-3 mm across mm-level anchor changes).
"""
import copy, json, os
import numpy as np

from autocal.pose_fit import shift_dataset_home_offsets


def _directions(values):
    v = np.asarray(values, float)
    if len(v) < 2:
        return np.ones_like(v)
    d = np.sign(np.diff(v))
    return np.concatenate([[d[0]], d])


def point_directions(sweep):
    """Per point, {anchor: sign of that force-mode line's length change since the sweep's previous point}."""
    pts = sweep['data_points']
    s = int(sweep['sensor_anchor'])
    sens = _directions([p.get('l_sensor_mu', p['l_sensor']) for p in pts])
    out = []
    for i, p in enumerate(pts):
        dirs = {int(p.get('source_sensor_anchor', s)): float(sens[i])}
        prev = pts[i - 1] if i else (pts[1] if len(pts) > 1 else p)
        for e in p.get('extra_sensor_ranges', []):
            if 'force_n' not in e:
                continue
            other = next((x for x in prev.get('extra_sensor_ranges', []) if int(x['anchor']) == int(e['anchor'])), None)
            delta = 0.0 if other is None else (e['length_mm'] - other['length_mm']) * (1 if i else -1)
            dirs[int(e['anchor'])] = float(np.sign(delta) or 1.0)
        out.append(dirs)
    return out


def perturb(ds, spec=None):
    spec = json.loads(spec if spec is not None else os.environ.get('PERTURB', '[]'))
    if not spec:
        return ds
    out = copy.deepcopy(ds)
    U = out['config']['m666_before_data_collection']['U']
    n = int(out['num_anchors'])
    for term in spec:
        mirrors = copy.deepcopy(out['sweeps']) if term.get('two_sided') else []
        if 'ripple' in term:
            rng = np.random.default_rng(term['seed']) if 'seed' in term else None
            phase = rng.uniform(0, 2 * np.pi, n) if rng is not None else np.zeros(n)
        for sign, sweeps in ((1.0, out['sweeps']), (-1.0, mirrors)):
            for sweep in sweeps:
                pts = sweep['data_points']
                s = int(sweep['sensor_anchor'])
                dirs = point_directions(sweep)
                for i, p in enumerate(pts):
                    ang = np.deg2rad(np.asarray(p.get('mu') or p['raw_angles_deg'], float))
                    a_s = int(p.get('source_sensor_anchor', s))
                    lines = [(a_s, p, 'sensor_force_n', None)] + [
                        (int(e['anchor']), e, 'force_n', e) for e in p.get('extra_sensor_ranges', []) if 'force_n' in e]
                    for a, holder, key, extra in lines:
                        if 'ripple' in term:
                            err = term['amp'] * np.sin(term['ripple'] * ang[a] + phase[a])
                        else:
                            err = sign * term['hyst'] * dirs[i][a]
                        holder[key] = float(holder[key] - U[a] * err)
        out['sweeps'] += mirrors
    return out


def home_strain(ds, k, anchors):
    """Dataset shifted by the emulated home-stretch offsets at compliance scale k (None if not applicable)."""
    targets = [(s.get('metadata') or {}).get('strand_tension_n') for s in ds['sweeps']]
    if not all(isinstance(t, (int, float)) and t > 0 for t in targets):
        return None
    m = ds['config']['m666_before_data_collection']
    n = int(ds['num_anchors'])
    U = np.broadcast_to(np.asarray(m['U'], float), (n,))
    Y = np.broadcast_to(np.asarray(m['Y'], float), (n,))
    norm = np.linalg.norm(np.asarray(anchors, float), axis=1)
    offsets = np.zeros(n)
    offsets[:-1] = k * float(targets[0]) / float(m['S']) * (norm[:-1] + Y[:-1] / U[:-1])
    return shift_dataset_home_offsets(ds, offsets)
