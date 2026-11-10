"""Double-precision, ordered XPBD solve compiled by Warp (CPU or CUDA).

One thread solves one world's coupled constraints in reference order. Parallel
joint atomics would change Gauss-Seidel semantics. The other ECS systems remain
Python. CPU structured buffers are zero-copy; CUDA transfers are explicit.
"""
import sys
from contextlib import redirect_stdout
import numpy as np
import warp as wp

from . import ecs
from .cable_joints_components import CableJointComponent, CableLinkComponent, CablePathComponent
from .pbd_cable_constraint_solver import _hybrid_stored_gradient, has_axis_only_cable_spin_dof
from .rigid_bodies import resolve_rigid_body_solver_entity
from .spools import SpoolStateComponent
from .stepper_motor import (StepperMotorComponent, finite_number,
                           open_loop_stepper_holding_torque_limit, position_stepper_constraint_stiffness)

F = wp.float64
V = wp.vec3d
Q = wp.quatd
M = wp.mat33d
EPS = wp.constant(F(1e-9))


@wp.struct
class Body:
    p: V
    q: Q
    prev_p: V
    prev_q: Q
    local_p: V
    local_q: Q
    tensor: M
    inverse: M
    axis: V
    inv_mass: F
    stiffness: F
    limit: F
    encoder: F
    parent: int
    flags: int  # position=1, orientation=2, previous position=4, previous q=8, axis spin=16, torque=32, encoder=64, inverse inertia=128


@wp.struct
class Endpoint:
    entity: int
    solver: int
    spin: int
    coupled_joint: int
    coupled_first: int
    stored: F
    stored_solve: int
    stored_load: int
    implicit: int
    defer: int
    transfer: int


@wp.struct
class Joint:
    a: Endpoint
    b: Endpoint
    local_a: V
    local_b: V
    point_a: V
    point_b: V
    rest: F
    output: int
    transfer_a: int
    transfer_b: int


@wp.struct
class Path:
    start: int
    count: int
    iterations: int
    compliance: F
    damping: F
    spring: F


@wp.struct
class End:
    entity: int
    spin: int
    gradient: V
    angular: V
    axis: V
    inv_mass: F
    angular_denom: F
    free_inverse: F
    inverse: F
    solve_gradient: F
    load_gradient: F
    displacement: F
    limit: F


@wp.func
def unit(q: Q):
    length = wp.sqrt(wp.dot(q, q))
    result = Q(F(0), F(0), F(0), F(1))
    if length > F(0):
        result = q / length
    return result


@wp.func
def rotate(q: Q, v: V):
    # Raw frame semantics: do not normalize here.
    x = q[3]*v[0] + q[1]*v[2] - q[2]*v[1]
    y = q[3]*v[1] + q[2]*v[0] - q[0]*v[2]
    z = q[3]*v[2] + q[0]*v[1] - q[1]*v[0]
    w = -q[0]*v[0] - q[1]*v[1] - q[2]*v[2]
    return V(x*q[3] - w*q[0] - y*q[2] + z*q[1],
             y*q[3] - w*q[1] - z*q[0] + x*q[2],
             z*q[3] - w*q[2] - x*q[1] + y*q[0])


@wp.func
def conjugate(q: Q):
    return Q(-q[0], -q[1], -q[2], q[3])


@wp.func
def world_position(bodies: wp.array(dtype=Body), entity: int):
    b = bodies[entity]
    result = b.p
    if b.parent >= 0:
        parent = bodies[b.parent]
        if (parent.flags & 3) == 3:
            result = parent.p + rotate(parent.q, b.local_p)
    return result


@wp.func
def world_orientation(bodies: wp.array(dtype=Body), entity: int):
    b = bodies[entity]
    result = unit(b.q)
    if b.parent >= 0:
        parent = bodies[b.parent]
        if (parent.flags & 2) != 0:
            result = unit(parent.q * b.local_q)
    return result


@wp.func
def attachment(bodies: wp.array(dtype=Body), entity: int, point: V):
    b = bodies[entity]
    result = point
    if (b.flags & 1) != 0 or (b.parent >= 0 and (bodies[b.parent].flags & 3) == 3):
        result = world_position(bodies, entity) + rotate(world_orientation(bodies, entity), point)
    return result


@wp.func
def angular_delta(previous: Q, current: Q):
    delta = unit(current * unit(conjugate(previous)))
    if delta[3] < F(0):
        delta = -delta
    w = wp.clamp(delta[3], F(-1), F(1))
    angle = F(2)*wp.acos(w)
    sine = wp.sqrt(wp.max(F(0), F(1)-w*w))
    result = V(F(0))
    if angle > EPS and sine > EPS:
        result = V(delta[0], delta[1], delta[2]) * (angle/sine)
    return result


@wp.func
def inverse_inertia(b: Body, vector: V):
    # Explicit normalized rotation matrix, as in the reference tensor helper.
    q = unit(b.q)
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    two = F(2)
    r = M(F(1)-two*y*y-two*z*z, two*x*y-two*z*w, two*x*z+two*y*w,
          two*x*y+two*z*w, F(1)-two*x*x-two*z*z, two*y*z-two*x*w,
          two*x*z-two*y*w, two*y*z+two*x*w, F(1)-two*x*x-two*y*y)
    return r * b.inverse * wp.transpose(r) * vector


@wp.func
def local_orientation(bodies: wp.array(dtype=Body), entity: int, previous: bool):
    b = bodies[entity]
    result = b.q
    if previous:
        result = b.prev_q
    if b.parent >= 0:
        result = b.local_q
        if previous:
            result = unit(unit(conjugate(bodies[b.parent].prev_q)) * b.prev_q)
    return result


@wp.func
def spin_delta(bodies: wp.array(dtype=Body), entity: int):
    b = bodies[entity]
    result = F(0)
    if (b.flags & 10) == 10 and (b.parent < 0 or (bodies[b.parent].flags & 8) != 0):
        previous = local_orientation(bodies, entity, True)
        current = local_orientation(bodies, entity, False)
        relative = unit(unit(conjugate(previous))*current)
        axis = b.axis
        if wp.length(axis) > F(0):
            axis = axis / wp.length(axis)
        projection = wp.dot(V(relative[0], relative[1], relative[2]), axis)
        twist = Q(axis[0]*projection, axis[1]*projection, axis[2]*projection, relative[3])
        if wp.dot(twist, twist) > EPS:
            twist = unit(twist)
            result = F(2)*wp.atan2(wp.dot(V(twist[0], twist[1], twist[2]), axis), twist[3])
            pi = F(3.141592653589793)
            while result > pi:
                result = result - F(2)*pi
            while result < -pi:
                result = result + F(2)*pi
    return result


@wp.func
def end_info(bodies: wp.array(dtype=Body), joints: wp.array(dtype=Joint), ep: Endpoint,
             point: V, gradient: V, dt: F):
    b = bodies[ep.solver]
    end = End()
    end.entity = ep.solver
    end.spin = -1
    end.gradient = gradient
    end.inv_mass = b.inv_mass
    end.angular = wp.cross(point-b.p, gradient)
    if (b.flags & 2) != 0 and (b.flags & 16) == 0:
        value = wp.dot(end.angular, inverse_inertia(b, end.angular))
        if wp.isfinite(value) and value > F(1e-12):
            end.angular_denom = value
    if (b.flags & 4) != 0:
        end.displacement = wp.dot(gradient, b.p-b.prev_p)
    if (b.flags & 10) == 10:
        end.displacement = end.displacement + wp.dot(end.angular, angular_delta(b.prev_q, b.q))
    if ep.spin >= 0:
        spin = bodies[ep.spin]
        axis = rotate(spin.q, spin.axis)
        spin_point = point
        spin_gradient = gradient
        if ep.coupled_joint >= 0:
            neighbor = joints[ep.coupled_joint]
            if ep.coupled_first != 0:
                spin_point = attachment(bodies, ep.spin, neighbor.local_a)
            else:
                spin_point = attachment(bodies, ep.spin, neighbor.local_b)
            spin_gradient = point-spin_point
        if wp.dot(axis, axis) > EPS and wp.dot(spin_gradient, spin_gradient) > EPS:
            axis = axis / wp.length(axis)
            if ep.coupled_joint >= 0:
                spin_gradient = spin_gradient / wp.length(spin_gradient)
            local_axis = rotate(unit(conjugate(spin.q)), axis)
            if wp.length(local_axis) > F(1e-12):
                local_axis = local_axis / wp.length(local_axis)
            else:
                local_axis = V(F(0), F(0), F(1))
            inertia = wp.dot(local_axis, spin.tensor*local_axis)
            if wp.isfinite(inertia) and inertia > F(1e-12):
                free_inverse = F(1)/inertia
                if free_inverse > EPS:
                    end.spin = ep.spin
                    end.axis = axis
                    end.free_inverse = free_inverse
                    end.inverse = free_inverse
                    if (spin.flags & 32) != 0:
                        end.inverse = F(0)
                    elif spin.stiffness > EPS and wp.isfinite(dt) and dt > EPS:
                        effective_inertia = inertia
                        if inertia <= EPS:
                            effective_inertia = F(1)/free_inverse
                        effective = effective_inertia + spin.stiffness*dt*dt
                        if effective > EPS:
                            end.inverse = F(1)/effective
                    end.limit = spin.limit
                    end.solve_gradient = wp.dot(wp.cross(spin_point-world_position(bodies, ep.spin), spin_gradient), axis)
                    if ep.stored_solve != 0 and (spin.flags & 32) != 0:
                        end.solve_gradient = ep.stored
                    end.load_gradient = end.solve_gradient
                    if ep.stored_load != 0:
                        end.load_gradient = ep.stored
                    end.displacement = end.displacement + end.solve_gradient*spin_delta(bodies, ep.spin)
    return end


@wp.func
def mechanical(a: End, b: End):
    return (a.inv_mass*wp.dot(a.gradient, a.gradient) + a.angular_denom
            + b.inv_mass*wp.dot(b.gradient, b.gradient) + b.angular_denom
            + a.inverse*a.solve_gradient*a.solve_gradient + b.inverse*b.solve_gradient*b.solve_gradient)


@wp.func
def multiplier(a: End, b: End, path: Path, dt: F, error: F):
    valid = wp.isfinite(dt) and dt > EPS
    value = F(0)
    if wp.isinf(path.compliance) and path.compliance > F(0):
        step = F(0)
        if valid:
            step = wp.max(F(0), path.damping)*dt
        value = step*(a.displacement+b.displacement)/(F(1)+step*mechanical(a,b))
    else:
        alpha = F(0)
        gamma = F(0)
        if valid:
            alpha = path.compliance/(dt*dt)
            gamma = wp.max(F(0), path.compliance*path.damping/dt)
        denom = (F(1)+gamma)*mechanical(a,b)+alpha
        value = F(wp.nan)
        if denom > EPS:
            value = (-error+gamma*(a.displacement+b.displacement))/denom
    return value


@wp.func
def correct_angle(bodies: wp.array(dtype=Body), entity: int, delta: V):
    b = bodies[entity]
    angle = wp.length(delta)
    if (b.flags & 2) != 0 and angle > EPS:
        axis = delta/angle
        sine = wp.sin(angle/F(2))
        correction = Q(axis[0]*sine, axis[1]*sine, axis[2]*sine, wp.cos(angle/F(2)))
        b.q = unit(correction*b.q)
        if b.parent >= 0 and (bodies[b.parent].flags & 2) != 0:
            b.local_q = unit(unit(conjugate(bodies[b.parent].q))*b.q)
        bodies[entity] = b


@wp.func
def apply_correction(bodies: wp.array(dtype=Body), end: End, value: F):
    b = bodies[end.entity]
    if end.inv_mass > F(0):
        b.p = b.p + end.gradient*(-end.inv_mass*value)
        bodies[end.entity] = b
    if end.angular_denom > F(0) and (b.flags & 2) != 0:
        correct_angle(bodies, end.entity, inverse_inertia(b, end.angular)*(-value))
    if end.inverse > F(0) and end.spin >= 0 and wp.abs(end.solve_gradient) > EPS:
        delta = -end.inverse*value*end.solve_gradient
        spin = bodies[end.spin]
        if wp.abs(delta) > EPS and (spin.flags & 2) != 0:
            correct_angle(bodies, end.spin, end.axis*delta)
            spin = bodies[end.spin]
            if (spin.flags & 64) != 0 and wp.isfinite(spin.encoder):
                spin.encoder = spin.encoder + delta
                bodies[end.spin] = spin


@wp.func
def record_load(bodies: wp.array(dtype=Body), loads: wp.array2d(dtype=F), ep: Endpoint,
                end: End, path: Path, value: F, inv_dt2: F):
    if end.spin >= 0 and (bodies[end.spin].flags & 32) != 0 and ep.defer == 0 and inv_dt2 > F(0) and wp.abs(end.load_gradient) > EPS:
        torque = value*inv_dt2*end.load_gradient
        if wp.isfinite(torque) and wp.abs(torque) > EPS:
            loads[end.spin,0] = loads[end.spin,0]+torque
            loads[end.spin,3] = F(1)
            if ep.implicit != 0:
                square = end.load_gradient*end.load_gradient
                if wp.isfinite(path.spring) and path.spring > F(0):
                    loads[end.spin,1] = loads[end.spin,1]+path.spring*square
                    loads[end.spin,4] = F(1)
                if wp.isfinite(path.damping) and path.damping > F(0):
                    loads[end.spin,2] = loads[end.spin,2]+path.damping*square
                    loads[end.spin,5] = F(1)


@wp.kernel(enable_backward=False)
def solve(bodies: wp.array(dtype=Body), joints: wp.array(dtype=Joint), paths: wp.array(dtype=Path),
          loads: wp.array2d(dtype=F), telemetry: wp.array2d(dtype=F), dt: F):
    for i in range(telemetry.shape[0]):
        for field in range(6):
            telemetry[i,field] = F(0)
    for i in range(bodies.shape[0]):
        for field in range(6):
            loads[i,field] = F(0)
    # Freeze attachment locals once per update; live frames change within solve.
    for i in range(joints.shape[0]):
        joint = joints[i]
        joint.local_a = rotate(unit(conjugate(world_orientation(bodies, joint.a.entity))), joint.point_a-world_position(bodies,joint.a.entity))
        joint.local_b = rotate(unit(conjugate(world_orientation(bodies, joint.b.entity))), joint.point_b-world_position(bodies,joint.b.entity))
        joints[i] = joint
    iterations = int(1)
    for p in range(paths.shape[0]):
        iterations = wp.max(iterations,paths[p].iterations)
    inv_dt2 = F(0)
    if wp.isfinite(dt) and dt > EPS:
        inv_dt2 = F(1)/(dt*dt)
    for iteration in range(iterations):
        for pi in range(paths.shape[0]):
            path_index = pi
            if iteration % 2 != 0:
                path_index = paths.shape[0]-1-pi
            path = paths[path_index]
            if iteration >= path.iterations:
                continue
            for ji in range(path.count):
                index = path.start+ji
                if iteration % 2 != 0:
                    index = path.start+path.count-1-ji
                joint = joints[index]
                pa = attachment(bodies,joint.a.entity,joint.local_a)
                pb = attachment(bodies,joint.b.entity,joint.local_b)
                length = wp.length(pb-pa)
                error = length-joint.rest
                if length <= EPS or error <= EPS or (bodies[joint.a.solver].flags & 1) == 0 or (bodies[joint.b.solver].flags & 1) == 0:
                    continue
                direction = (pb-pa)/length
                a = end_info(bodies,joints,joint.a,pa,direction,dt)
                b = end_info(bodies,joints,joint.b,pb,-direction,dt)
                value = multiplier(a,b,path,dt,error)
                if wp.isnan(value):
                    continue
                released = bool(False)
                if a.spin >= 0 and wp.isfinite(a.limit) and a.limit >= F(0) and inv_dt2 > F(0) and wp.abs(a.solve_gradient) > EPS:
                    if wp.abs(value)*inv_dt2*wp.abs(a.solve_gradient) > a.limit+EPS:
                        a.inverse = a.free_inverse
                        released = True
                if b.spin >= 0 and wp.isfinite(b.limit) and b.limit >= F(0) and inv_dt2 > F(0) and wp.abs(b.solve_gradient) > EPS:
                    if wp.abs(value)*inv_dt2*wp.abs(b.solve_gradient) > b.limit+EPS:
                        b.inverse = b.free_inverse
                        released = True
                if released:
                    value = multiplier(a,b,path,dt,error)
                if wp.isnan(value):
                    continue
                record_load(bodies,loads,joint.a,a,path,value,inv_dt2)
                record_load(bodies,loads,joint.b,b,path,value,inv_dt2)
                if mechanical(a,b) <= EPS:
                    continue
                if iteration == 0:
                    output = joint.output
                    telemetry[output,0] = value
                    force = direction*(value*inv_dt2)
                    telemetry[output,1] = force[0]
                    telemetry[output,2] = force[1]
                    telemetry[output,3] = force[2]
                    magnitude = wp.abs(value)*inv_dt2
                    telemetry[output,4] = magnitude
                    for side in range(4):
                        transfer = joint.transfer_a
                        if side == 1:
                            transfer = joint.transfer_b
                        elif side == 2:
                            transfer = -1
                            if a.spin >= 0:
                                transfer = joint.a.transfer
                        elif side == 3:
                            transfer = -1
                            if b.spin >= 0:
                                transfer = joint.b.transfer
                        if transfer >= 0 and wp.isfinite(magnitude) and magnitude > F(0):
                            telemetry[transfer,5] = wp.max(telemetry[transfer,5],magnitude)
                apply_correction(bodies,a,value)
                apply_correction(bodies,b,value)


class WarpCableConstraintSolver:
    """Opt-in drop-in solver; all ECS coupling is packed once per update."""

    def __init__(self, device='cpu'):
        # Fresh experiments can run inside a stdio MCP server. Compiler and
        # device diagnostics must never enter its protocol stream.
        with redirect_stdout(sys.stderr):
            wp.init()
        if isinstance(device, str) and device.startswith('cuda') and not wp.is_cuda_available():
            raise ValueError('CUDA cable solver requested, but no CUDA device is available')
        self.device = wp.get_device(device)
        self.buffers = None
        self.first_launch = True

    def _allocate(self, sizes):
        if self.buffers is not None and self.sizes == sizes:
            return
        self.sizes = sizes
        n, j, p, o = sizes
        self.host = [wp.zeros(n,dtype=Body,device='cpu'), wp.zeros(j,dtype=Joint,device='cpu'),
                     wp.zeros(p,dtype=Path,device='cpu'), wp.zeros((n,6),dtype=F,device='cpu'),
                     wp.zeros((o,6),dtype=F,device='cpu')]
        self.views = [array.numpy() for array in self.host]
        self.buffers = self.host if self.device.is_cpu else [wp.empty_like(a,device=self.device) for a in self.host]

    def update(self, world, _dt_unused):
        paths = [world.get_component(p,CablePathComponent) for p in world.query([CablePathComponent])]
        joint_ids = list(dict.fromkeys(j for path in paths for j in path.joint_entities))
        components = world.components
        get = lambda entity,kind: components.get(kind,{}).get(entity)
        needed = set()
        for jid in joint_ids:
            joint = get(jid,CableJointComponent)
            needed.update((joint.entity_a,joint.entity_b))
        pending = list(needed)
        while pending:
            member = get(pending.pop(),ecs.RigidBodyMemberComponent)
            if member is not None and member.body_entity not in needed:
                needed.add(member.body_entity)
                pending.append(member.body_entity)
        entities = [entity for entity in world.entities if entity in needed]
        ids = {entity:i for i,entity in enumerate(entities)}
        outputs = {j:i for i,j in enumerate(joint_ids)}
        self._allocate((len(entities),sum(len(p.joint_entities) for p in paths),len(paths),len(joint_ids)))
        bs, js, ps, loads, telemetry = self.views
        bs[:] = np.zeros((),dtype=bs.dtype)
        js[:] = np.zeros((),dtype=js.dtype)
        dt = world.get_resource('dt')
        for entity,i in ids.items():
            b = bs[i]
            b['parent'] = -1
            flags = 0
            for kind,field,attr,flag in [(ecs.PositionComponent,'p','pos',1), (ecs.OrientationComponent,'q','quaternion',2),
                (ecs.PrevFinalPosComponent,'prev_p','pos',4), (ecs.PrevFinalOrientationComponent,'prev_q','quaternion',8)]:
                c = get(entity,kind)
                if c is not None:
                    b[field] = getattr(c,attr).as_xyzw() if attr == 'quaternion' else getattr(c,attr)
                    flags |= flag
                elif attr == 'quaternion':
                    b[field] = [0,0,0,1]
            member = get(entity,ecs.RigidBodyMemberComponent)
            if member is not None:
                b['parent'] = ids[member.body_entity]
                b['local_p'] = member.local_position
                b['local_q'] = member.local_orientation.as_xyzw()
            mass = get(entity,ecs.MassComponent)
            b['inv_mass'] = 1/mass.mass if mass is not None and mass.mass > 0 else 0
            moment = get(entity,ecs.MomentOfInertiaComponent)
            if moment is not None:
                b['tensor'] = moment.inertia_tensor
                b['inverse'] = moment.inv_inertia_tensor
                if np.any(np.abs(moment.inv_inertia_tensor) > 1e-12):
                    flags |= 128
            link = get(entity,CableLinkComponent)
            if link is not None and link.cable_plane_normal_local is not None:
                b['axis'] = link.cable_plane_normal_local
                if get(entity,SpoolStateComponent) is not None:
                    flags |= 16
            stepper = get(entity,StepperMotorComponent)
            b['stiffness'] = position_stepper_constraint_stiffness(world,stepper)
            b['limit'] = open_loop_stepper_holding_torque_limit(world,stepper)
            if stepper is not None and stepper.torque_mode:
                flags |= 32
            encoder = get(entity,ecs.EncoderComponent)
            if encoder is not None:
                flags |= 64
                b['encoder'] = encoder.angle
            b['flags'] = flags
        offset = 0
        for pi,path in enumerate(paths):
            ps[pi] = (offset,len(path.joint_entities),path.solver_iterations,path.compliance,path.damping,path.spring_constant)
            for index,jid in enumerate(path.joint_entities):
                joint = get(jid,CableJointComponent)
                j = js[offset+index]
                j['point_a'],j['point_b'],j['rest'],j['output'] = joint.attachment_point_a_world,joint.attachment_point_b_world,joint.rest_length,outputs[jid]
                j['transfer_a'] = outputs[path.joint_entities[index-1]] if path.link_types[index] == 'pinhole' and index > 0 else -1
                j['transfer_b'] = outputs[path.joint_entities[index+1]] if path.link_types[index+1] == 'pinhole' and index+1 < len(path.joint_entities) else -1
                for first,entity,other,key in [(True,joint.entity_a,joint.entity_b,'a'),(False,joint.entity_b,joint.entity_a,'b')]:
                    ep = j[key]
                    solver,internal = resolve_rigid_body_solver_entity(world,entity,other)
                    ep['entity'],ep['solver'] = ids[entity],ids[solver]
                    ep['spin'],ep['coupled_joint'],ep['transfer'] = -1,-1,-1
                    link_index = index if first else index+1
                    spin_entity = None
                    spin_first = first
                    spin_index = link_index
                    if has_axis_only_cable_spin_dof(world,entity):
                        spin_entity = entity
                        stored = _hybrid_stored_gradient(world,path,link_index,first,entity)
                        ep['stored_solve'] = int(abs(stored) > 1e-9)
                        ep['defer'] = int(abs(stored) > 1e-9 and (
                            (first and path.link_types[index+1] == 'pinhole' and index+1 < len(path.joint_entities))
                            or (not first and path.link_types[index] == 'pinhole' and index > 0)))
                    elif not internal and len(path.joint_entities) >= 2 and path.link_types[link_index] == 'pinhole':
                        ni = -1
                        if first and link_index > 0 and path.link_types[link_index-1] in ('rolling','hybrid','hybrid-attachment'):
                            ni,spin_first = index-1,True
                        elif not first and link_index < len(path.link_types)-1 and path.link_types[link_index+1] in ('rolling','hybrid','hybrid-attachment'):
                            ni,spin_first = index+1,False
                        if 0 <= ni < len(path.joint_entities):
                            neighbor_id = path.joint_entities[ni]
                            neighbor = get(neighbor_id,CableJointComponent)
                            spin_entity = neighbor.entity_a if spin_first else neighbor.entity_b
                            ep['coupled_joint'],ep['coupled_first'],ep['transfer'] = offset+ni,int(spin_first),outputs[neighbor_id]
                            spin_index = ni if spin_first else ni+1
                    if spin_entity is not None:
                        link = get(spin_entity,CableLinkComponent)
                        spin_body = bs[ids[spin_entity]]
                        if link is not None and link.cable_plane_normal_local is not None and (spin_body['flags'] & 130) == 130:
                            ep['spin'] = ids[spin_entity]
                            stored = _hybrid_stored_gradient(world,path,spin_index,spin_first,spin_entity)
                            ep['stored'] = stored
                            ep['stored_load'] = ep['implicit'] = int(abs(stored) > 1e-9)
            offset += len(path.joint_entities)
        if not self.device.is_cpu:
            for target,source in zip(self.buffers,self.host):
                wp.copy(target,source)
        if self.first_launch:
            with redirect_stdout(sys.stderr):
                wp.launch(solve,dim=1,inputs=[*self.buffers,F(dt)],device=self.device)
            self.first_launch = False
        else:
            wp.launch(solve,dim=1,inputs=[*self.buffers,F(dt)],device=self.device)
        if not self.device.is_cpu:
            for index in (0,3,4):
                wp.copy(self.host[index],self.buffers[index])
            wp.synchronize_device(self.device)
        for entity,i in ids.items():
            b = bs[i]
            position = get(entity,ecs.PositionComponent)
            if position is not None:
                position.pos[:] = b['p']
            orientation = get(entity,ecs.OrientationComponent)
            if orientation is not None:
                orientation.quaternion.x,orientation.quaternion.y,orientation.quaternion.z,orientation.quaternion.w = map(float,b['q'])
            member = get(entity,ecs.RigidBodyMemberComponent)
            if member is not None:
                member.local_orientation.x,member.local_orientation.y,member.local_orientation.z,member.local_orientation.w = map(float,b['local_q'])
            encoder = get(entity,ecs.EncoderComponent)
            if encoder is not None:
                encoder.angle = float(b['encoder'])
        for jid,i in outputs.items():
            joint = get(jid,CableJointComponent)
            values = telemetry[i]
            joint.constraint_lambda = float(values[0])
            joint.constraint_force[:] = values[1:4]
            joint.constraint_force_magnitude,joint.transferred_constraint_force_magnitude = map(float,values[4:6])
        for column,key in enumerate(('torqueModeCableLoadTorques','torqueModeCableLoadStiffnesses','torqueModeCableLoadDampings')):
            world.set_resource(key,{entity:float(loads[i,column]) for entity,i in ids.items() if loads[i,column+3] != 0})
