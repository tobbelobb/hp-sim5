"""Audit DaRUS-4614 stationary force balance directly from PlaneA Scope exports."""
import argparse
import csv
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts/reference_data'))
from summarize_4614 import EXPERIMENTS, P_TARGETS, read_scope


def vectors(nodes, fields):
    return np.array([[float(node.get(key)) for key in fields] for node in nodes])


def directions(geometry, poses):
    base, local, axis, radii = geometry
    rotations = Rotation.from_euler('XYZ', poses[:, 3:]).as_matrix()
    attach = poses[:, None, :3] + local @ rotations.transpose(0, 2, 1)
    delta = attach - base
    axial = np.sum(delta * axis, axis=2, keepdims=True)
    radial = delta - axial * axis
    distance = np.linalg.norm(radial, axis=2, keepdims=True)
    angle = np.arctan2(axial, distance - radii) + np.arccos(radii / np.hypot(distance - radii, axial))
    tangent = base + radii * ((1 + np.cos(angle)) * radial / distance + np.sin(angle) * axis)
    span = tangent - attach
    return span / np.linalg.norm(span, axis=2, keepdims=True)


def block(scope, name, count):
    return np.column_stack([scope[f'{name}[{i}]'] for i in range(count)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base', type=Path, help='Directory with robot/ and COPacabana.xml, or experiments/')
    parser.add_argument('--exports', type=int, default=30)
    args = parser.parse_args()
    xml = args.base / 'COPacabana.xml'
    tree = ET.parse(xml if xml.exists() else args.base.parent / xml.name)
    chains = sorted(tree.findall('.//geometry/chain'), key=lambda c: int(c.get('id')))
    bases = [chain.find('base') for chain in chains]
    geometry = (vectors(bases, 'xyz'), vectors([c.find('platform') for c in chains], 'xyz'),
                vectors(bases, ['a31', 'a32', 'a33']), vectors(bases, ['radius']))
    out = csv.writer(sys.stdout)
    out.writerow(['experiment', 'export', 'pose', 'samples', 'mass_kg', 'fx_N', 'fy_N', 'fz_minus_mg_N',
                  'closure_N', 'target_closure_N', 'wc_x', 'wc_y', 'wc_z', 'max_abs_logged_angle',
                  'minimum_static_adjustment_rms_N'])
    for exp, _, mass in EXPERIMENTS:
        for run in range(1, args.exports + 1):
            path = args.base / 'robot' / exp / 'PlaneA' / f'Export {run}.csv'
            scope = read_scope(path)
            target, pose = block(scope, 'p_tar', 3), block(scope, 'p', 6)
            direction = directions(geometry, pose)
            gravity = np.array([0, 0, -mass * 9.81])
            logged_force = block(scope, 'f', 8)
            net = np.sum(direction * logged_force[:, :, None], axis=1) + gravity
            planned = np.sum(direction * block(scope, 'f_tar', 8)[:, :, None], axis=1) + gravity
            wrench = block(scope, 'wc', 3)
            for idx, point in enumerate(P_TARGETS, 1):
                at_target = np.flatnonzero(np.all(np.abs(target - point) < 1e-7, axis=1))
                for visit in np.split(at_target, np.where(np.diff(at_target) != 1)[0] + 1):
                    if not len(visit):
                        continue
                    keep = visit[scope.t[visit] - scope.t[visit[0]] > 0.5]
                    if not len(keep):
                        continue
                    force = net[keep].mean(axis=0)
                    # At the mean controller pose, find the closest eight cable readings
                    # that balance both the weight and moments about the XML center of mass.
                    mean_pose = pose[keep].mean(axis=0)
                    unit = directions(geometry, mean_pose[None, :])[0]
                    arms = Rotation.from_euler('XYZ', mean_pose[3:]).apply(geometry[1])
                    matrix = np.vstack((unit.T, np.cross(arms, unit).T))
                    imbalance = matrix @ logged_force[keep].mean(axis=0) + np.r_[gravity, np.zeros(3)]
                    adjustment = np.linalg.lstsq(matrix, -imbalance, rcond=None)[0]
                    out.writerow([exp, run, idx, len(keep), mass, *force, np.linalg.norm(force),
                                  np.linalg.norm(planned[keep].mean(axis=0)), *wrench[keep].mean(axis=0),
                                  np.abs(pose[keep, 3:]).max(), np.sqrt(np.mean(adjustment**2))])


if __name__ == '__main__':
    main()
