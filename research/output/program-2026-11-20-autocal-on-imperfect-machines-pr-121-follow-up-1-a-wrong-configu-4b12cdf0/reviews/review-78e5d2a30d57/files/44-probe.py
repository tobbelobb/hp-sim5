"""Compliance scale k (true compliance / configured M666 S) from fixed-pose force steps.

At a probe the drive and held lines keep their spools fixed and the sensor force
steps by dF. Equilibrium fixes the held lines' tension changes dT (independent of k).
With line compliance c_i = k (U d_i + Y_i) / (S U^2 N_i), each held line obeys
  -u_i . dp - c_i dT_i = dm_i          (dm_i: its measured encoder change)
which gives the effector shift dp, linear in k, and the sensor's encoder change
  dm_s = -u_s . dp - c_s dF = a + k b.
k is the least-squares slope over all steps of all probes.
"""
from __future__ import annotations

import numpy as np

from autocal.pose_fit import fit_point_positions


def line_description(dataset):
    m = dataset["config"]["m666_before_data_collection"]
    n = len(m["N"])
    return (float(m["S"]), np.asarray(m["N"], float), np.broadcast_to(np.asarray(m["U"], float), (n,)),
            np.asarray(m["Y"], float))


def probe_rows(dataset):
    mm = np.asarray(dataset["config"]["mm_per_degree"], float)
    for sweep in dataset["sweeps"]:
        for probe in (sweep.get("metadata") or {}).get("compliance_probes", []):
            levels = probe["levels"]
            lengths = np.array([np.asarray(level["mu_deg"], float) * mm for level in levels])
            forces = np.array([level["force_n"] for level in levels])
            yield int(probe["sensor_anchor"]), forces, lengths


def step_terms(anchors, position, sensor, d_force, d_lengths, stiffness, strands, advantage, extra):
    """(a, b) with dm_s = a + k b for one force step at one pose."""
    to_anchor = anchors - position
    distance = np.linalg.norm(to_anchor, axis=1)
    unit = to_anchor / distance[:, None]
    held = [i for i in range(len(anchors)) if i != sensor]
    d_tension = np.linalg.solve(unit[held].T, -d_force * unit[sensor])
    compliance = (advantage * distance + extra) / (stiffness * advantage ** 2 * strands)
    # u_i . dp = -dm_i - k c_i dT_i  ->  dp = P + k Q
    basis = unit[held]
    p = np.linalg.solve(basis, -d_lengths[held])
    q = np.linalg.solve(basis, -compliance[held] * d_tension)
    a = -unit[sensor] @ p
    b = -unit[sensor] @ q - compliance[sensor] * d_force
    return a, b


def estimate(dataset, anchors, home_offsets):
    stiffness, strands, advantage, extra = line_description(dataset)
    rows = []
    for sensor, forces, lengths in probe_rows(dataset):
        position = fit_point_positions(np.asarray(anchors, float), lengths[:1], np.asarray(home_offsets, float))[0][0]
        for j in range(1, len(forces)):
            a, b = step_terms(np.asarray(anchors, float), position, sensor, forces[j] - forces[0],
                              lengths[j] - lengths[0], stiffness, strands, advantage, extra)
            rows.append(dict(sensor=sensor, d_force=forces[j] - forces[0], a=a, b=b,
                             measured=lengths[j][sensor] - lengths[0][sensor]))
    if not rows:
        return None, rows
    a = np.array([r["a"] for r in rows]); b = np.array([r["b"] for r in rows])
    m = np.array([r["measured"] for r in rows])
    k = float(np.sum(b * (m - a)) / np.sum(b * b))
    for r in rows:
        r["k"] = (r["measured"] - r["a"]) / r["b"] if r["b"] else float("nan")
    return k, rows
