"""Geometric two-port reduction of the actual HP4 ABC routing; no dynamics."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from pinhole_gate import PinholeGate

ROOT = Path(__file__).resolve().parent
initial = json.loads((ROOT / "E02.json").read_text())["rows"][0]
centre = np.array(initial["xyz"])
rng = np.random.default_rng(62300644)
records = []
for path in initial["paths"][:6]:
    joints = path["joints"]
    entry = np.array(joints[1]["pb"]) - centre
    exit = np.array(joints[5]["pa"]) - centre
    anchors = np.array([joints[1]["pa"], joints[5]["pb"]])
    internal = [(np.array(j["pa"]) - centre, np.array(j["pb"]) - centre) for j in joints[2:5]]
    arc_length = sum(path["stored"][3:5])
    length_internal = sum(np.linalg.norm(a-b) for a,b in internal) + arc_length
    ports = np.array([entry, exit])
    model = PinholeGate(entry, exit, float(length_internal))

    def full(position, rotation):
        exterior = np.linalg.norm(position + ports @ rotation.T - anchors, axis=1).sum()
        hidden = sum(np.linalg.norm(position+rotation@a-position-rotation@b) for a,b in internal) + arc_length
        return exterior + hidden

    def gate(position, rotation):
        return model.length_and_jacobian(position, rotation, anchors)[0]

    def midpoint(position, rotation):
        point = position + rotation @ ports.mean(axis=0)
        return np.linalg.norm(point-anchors, axis=1).sum()

    offset_midpoint = full(centre, np.eye(3))-midpoint(centre,np.eye(3))
    errors, gradient_errors, midpoint_errors, torque_errors = [], [], [], []
    for _ in range(500):
        position = centre + rng.uniform(-.3,.3,3)
        rotation = Rotation.from_rotvec(rng.uniform(-.1,.1,3)).as_matrix()
        transformed = ports @ rotation.T
        u = position + transformed - anchors
        u /= np.linalg.norm(u,axis=1)[:,None]
        _, gradient = model.length_and_jacobian(position, rotation, anchors)
        eps=1e-6
        finite=[]
        for axis in np.eye(3):
            finite.append((full(position+eps*axis,rotation)-full(position-eps*axis,rotation))/(2*eps))
        for axis in np.eye(3):
            dr=Rotation.from_rotvec(eps*axis).as_matrix()
            finite.append((full(position,dr@rotation)-full(position,dr.T@rotation))/(2*eps))
        errors.append(abs(full(position,rotation)-gate(position,rotation)))
        gradient_errors.append(float(np.max(np.abs(gradient-finite))))
        midpoint_errors.append(abs(full(position,rotation)-midpoint(position,rotation)-offset_midpoint))
        rm=rotation@ports.mean(axis=0)
        um=position+rm-anchors
        um/=np.linalg.norm(um,axis=1)[:,None]
        torque_errors.append(float(np.linalg.norm(gradient[3:]-np.cross(rm,um).sum(axis=0))))
    records.append(dict(path=path["name"],entry_local=entry.tolist(),exit_local=exit.tolist(),
        hidden_length_m=float(length_internal),direct_bridge_m=float(np.linalg.norm(entry-exit)),
        omitted_bridge_offset_m=float(length_internal-np.linalg.norm(entry-exit)),
        max_gate_length_error_m=max(errors),max_analytic_gradient_error=max(gradient_errors),
        max_midpoint_length_error_m=max(midpoint_errors),max_midpoint_torque_error_Nm_at_1N=max(torque_errors)))

result=dict(id="E25",kind="synthetic geometry and finite-difference audit; not a dynamic solver",seed=62300644,
    poses_per_branch=500,translation_range_m=[-.3,.3],rotation_vector_component_range_rad=[-.1,.1],
    reflected_flow_mass_per_ABC_wheel_kg=5e-7/.0065**2,records=records)
(ROOT/"E25-gate-contract.json").write_text(json.dumps(result,indent=2)+"\n")
print(json.dumps(result,indent=2))
