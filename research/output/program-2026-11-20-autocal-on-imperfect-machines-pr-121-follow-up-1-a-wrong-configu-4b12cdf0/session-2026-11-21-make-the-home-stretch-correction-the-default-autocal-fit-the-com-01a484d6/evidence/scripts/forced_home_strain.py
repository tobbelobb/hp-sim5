"""Offline: does forcing the home-strain term onto a redundant machine's fit help or double-count?

usage: forced_home_strain.py RUN_DIR [RUN_DIR...]   (imperfect-machine run dirs with sweeps.json.gz)

Refits each stored dataset with the sweep autocal's two-stage point fit
(compliance study synth.production_fit), once as shipped and once with
pose_fit._home_strain forced to (T/S, Y/U) on the first three low lines.
Scores in the harness print frame against the scene truth.
"""
import copy, gzip, json, sys
from pathlib import Path
import numpy as np
import autocal.pose_fit as pf
from autocal.anchor_seed import build_anchor_initial_guess
from autocal.tools.imperfect_machine import convention_frame

shipped = pf._home_strain


def forced(dataset):
    m = dataset["config"]["m666_before_data_collection"]
    n = int(dataset["num_anchors"])
    target = float(dataset["sweeps"][0]["metadata"]["strand_tension_n"])
    u = np.broadcast_to(np.asarray(m["U"], float), (n,))
    return target / float(m["S"]), np.asarray(m["Y"], float)[:3] / u[:3]


def production_fit(dataset):
    dataset = copy.deepcopy(dataset)
    seed = build_anchor_initial_guess(dataset, machine_type=dataset["machine_type"])
    sigma = pf.HOME_OFFSET_PRIOR_SIGMA_MM
    offsets = pf.fit_pose_least_squares(dataset, seed, fit_home_offsets=True, home_offset_prior_sigma_mm=sigma)
    fit = pf.fit_pose_least_squares(pf.shift_dataset_home_offsets(dataset, offsets["home_offsets_mm"]),
                                    offsets["anchors"], fit_home_offsets=False, home_offset_prior_sigma_mm=sigma)
    return fit, offsets


for run in map(Path, sys.argv[1:]):
    dataset = json.load(gzip.open(run / "sweeps.json.gz"))
    truth = np.array(list(json.loads((run / "imperfection.json").read_text())["truth_mm"].values()))
    truth_frame = convention_frame(truth)[0]
    row = dict(run=run.name, machine=dataset["machine_type"], equal_strain_target=dataset["sweeps"][0]["metadata"].get("strand_tension_n"))
    for name, fn in (("shipped", shipped), ("forced", forced)):
        pf._home_strain = fn
        try:
            fit, offsets = production_fit(dataset)
        finally:
            pf._home_strain = shipped
        e = convention_frame(np.asarray(fit["anchors"]))[0] - truth_frame
        row[name] = dict(max_abs_frame_mm=round(float(np.abs(e).max()), 3), top_z_mm=round(float(e[-1, 2]), 3),
                         low_z_mean_mm=round(float(e[:-1, 2].mean()), 3), rms_mm=round(float(fit["rms_mm"]), 4),
                         offsets_skipped=offsets.get("home_offsets_skipped"),
                         home_offsets_mm=np.round(offsets["home_offsets_mm"], 3).tolist())
    print(json.dumps(row), flush=True)
