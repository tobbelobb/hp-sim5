import numpy as np
import warp as wp

from cable_joints.ecs import (
    PositionComponent,
    OrientationComponent,
    MassComponent,
    MomentOfInertiaComponent,
)
from cable_joints.cable_joints_components import (
    CableJointComponent,
    CablePathComponent,
)


@wp.kernel
def solve_cable_joints(
    positions: wp.array(dtype=wp.vec3),
    angles: wp.array(dtype=float),
    inv_masses: wp.array(dtype=float),
    inv_inertias: wp.array(dtype=float),
    entity_a: wp.array(dtype=int),
    entity_b: wp.array(dtype=int),
    rest_lengths: wp.array(dtype=float),
    attach_a: wp.array(dtype=wp.vec3),
    attach_b: wp.array(dtype=wp.vec3),
    compliance: wp.array(dtype=float),
    dt: float,
):
    epsilon = 1e-9
    tid = wp.tid()
    id_a = entity_a[tid]
    id_b = entity_b[tid]

    p_a = attach_a[tid]
    p_b = attach_b[tid]

    inv_mass_a = inv_masses[id_a]
    inv_mass_b = inv_masses[id_b]
    inv_inertia_a = inv_inertias[id_a]
    inv_inertia_b = inv_inertias[id_b]

    w_sum = inv_mass_a + inv_mass_b + inv_inertia_a + inv_inertia_b
    if w_sum <= epsilon:
        return

    diff = p_b - p_a
    length = wp.length(diff)
    if length <= epsilon:
        return

    direction = diff / length
    C = length - rest_lengths[tid]
    if C <= epsilon:
        return

    grad_pos_a = direction
    grad_pos_b = -direction

    r_a = p_a - positions[id_a]
    grad_ang_a = r_a[0] * direction[1] - r_a[1] * direction[0]

    r_b = p_b - positions[id_b]
    grad_ang_b = r_b[0] * (-direction[1]) - r_b[1] * (-direction[0])

    denom = (
        inv_mass_a * wp.dot(grad_pos_a, grad_pos_a)
        + inv_inertia_a * grad_ang_a * grad_ang_a
        + inv_mass_b * wp.dot(grad_pos_b, grad_pos_b)
        + inv_inertia_b * grad_ang_b * grad_ang_b
    )
    if dt > 0.0:
        denom += compliance[tid] / (dt * dt)
    if denom <= epsilon:
        return

    lambda_val = -C / denom
    if inv_mass_a > 0.0:
        delta_pos = grad_pos_a * (-inv_mass_a * lambda_val)
        wp.atomic_add(positions, id_a, delta_pos)
    if inv_inertia_a > 0.0:
        delta_ang = -inv_inertia_a * lambda_val * grad_ang_a
        wp.atomic_add(angles, id_a, delta_ang)
    if inv_mass_b > 0.0:
        delta_pos = grad_pos_b * (-inv_mass_b * lambda_val)
        wp.atomic_add(positions, id_b, delta_pos)
    if inv_inertia_b > 0.0:
        delta_ang = -inv_inertia_b * lambda_val * grad_ang_b
        wp.atomic_add(angles, id_b, delta_ang)
    lambda_val = -C / denom


class WarpCableConstraintSolver:
    """XPBD cable constraint solver implemented with Warp kernels."""

    run_in_pause = False

    def __init__(self, device=None):
        self.device = device or wp.get_preferred_device()

    def update(self, world, _dt_unused):
        dt = world.get_resource("dt")
        path_entities = world.query([CablePathComponent])

        joint_info = []
        entity_ids = set()
        for pid in path_entities:
            path = world.get_component(pid, CablePathComponent)
            if not path.joint_entities:
                continue
            for jid in path.joint_entities:
                joint = world.get_component(jid, CableJointComponent)
                joint_info.append((joint, path.compliance))
                entity_ids.add(joint.entity_a)
                entity_ids.add(joint.entity_b)

        if not joint_info:
            return

        id_to_idx = {eid: i for i, eid in enumerate(sorted(entity_ids))}
        num_entities = len(id_to_idx)

        pos = np.zeros((num_entities, 3), dtype=np.float32)
        ang = np.zeros(num_entities, dtype=np.float32)
        inv_mass = np.zeros(num_entities, dtype=np.float32)
        inv_inertia = np.zeros(num_entities, dtype=np.float32)

        for eid, idx in id_to_idx.items():
            pcomp = world.get_component(eid, PositionComponent)
            if pcomp:
                pos[idx] = pcomp.pos
            ocomp = world.get_component(eid, OrientationComponent)
            if ocomp:
                ang[idx] = ocomp.angle
            mcomp = world.get_component(eid, MassComponent)
            if mcomp and mcomp.mass > 0 and np.isfinite(mcomp.mass):
                inv_mass[idx] = 1.0 / mcomp.mass
            else:
                inv_mass[idx] = 0.0
            icomp = world.get_component(eid, MomentOfInertiaComponent)
            inv_inertia[idx] = icomp.inv_inertia if icomp else 0.0

        num_joints = len(joint_info)
        ent_a = np.zeros(num_joints, dtype=np.int32)
        ent_b = np.zeros(num_joints, dtype=np.int32)
        rest_len = np.zeros(num_joints, dtype=np.float32)
        attach_a = np.zeros((num_joints, 3), dtype=np.float32)
        attach_b = np.zeros((num_joints, 3), dtype=np.float32)
        comp = np.zeros(num_joints, dtype=np.float32)

        for i, (joint, compliance) in enumerate(joint_info):
            ent_a[i] = id_to_idx[joint.entity_a]
            ent_b[i] = id_to_idx[joint.entity_b]
            rest_len[i] = joint.rest_length
            attach_a[i] = joint.attachment_point_a_world
            attach_b[i] = joint.attachment_point_b_world
            comp[i] = compliance

        pos_d = wp.array(pos, dtype=wp.vec3, device=self.device)
        ang_d = wp.array(ang, dtype=float, device=self.device)
        inv_mass_d = wp.array(inv_mass, dtype=float, device=self.device)
        inv_inertia_d = wp.array(inv_inertia, dtype=float, device=self.device)
        ent_a_d = wp.array(ent_a, dtype=int, device=self.device)
        ent_b_d = wp.array(ent_b, dtype=int, device=self.device)
        rest_len_d = wp.array(rest_len, dtype=float, device=self.device)
        attach_a_d = wp.array(attach_a, dtype=wp.vec3, device=self.device)
        attach_b_d = wp.array(attach_b, dtype=wp.vec3, device=self.device)
        comp_d = wp.array(comp, dtype=float, device=self.device)

        wp.launch(
            solve_cable_joints,
            dim=num_joints,
            inputs=[
                pos_d,
                ang_d,
                inv_mass_d,
                inv_inertia_d,
                ent_a_d,
                ent_b_d,
                rest_len_d,
                attach_a_d,
                attach_b_d,
                comp_d,
                dt,
            ],
            device=self.device,
        )

        pos[:] = pos_d.numpy()
        ang[:] = ang_d.numpy()

        for eid, idx in id_to_idx.items():
            pcomp = world.get_component(eid, PositionComponent)
            if pcomp:
                pcomp.pos[:] = pos[idx]
            ocomp = world.get_component(eid, OrientationComponent)
            if ocomp:
                ocomp.angle = float(ang[idx])
