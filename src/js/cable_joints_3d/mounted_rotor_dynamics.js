import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';
import { OrientationComponent, MomentOfInertiaComponent, RigidBodyMemberComponent } from './ecs.js';
import { getEntityWorldOrientation } from './rigid_bodies.js';
import { invertMatrix3, multiplyMatrix3Vector, transformInertiaTensorToWorld } from './inertia_tensor.js';

// Coordinates are carrier rotation and each rotor's twist RELATIVE to it.
// The locked assembly tensor includes the rotors, so the mass matrix has
// off-diagonal columns I_rotor * axis. Eliminate the relative coordinates
// before solving the carrier, then back substitute (also for unloaded rotors).
export function createCableAngularResponse(world, getRotor) {
  const rotors = new Map();
  for (const id of world.query([RigidBodyMemberComponent])) {
    const rotor = getRotor(id);
    if (!rotor) continue;
    const parent = world.getComponent(id, RigidBodyMemberComponent).bodyEntity;
    if (!rotors.has(parent)) rotors.set(parent, []);
    rotors.get(parent).push(id);
  }
  return ends => {
    const groups = new Map(), standalone = [];
    for (const end of ends) {
      const parent = world.getComponent(end.entity, RigidBodyMemberComponent)?.bodyEntity;
      const body = parent ?? end.entity;
      if (!rotors.has(body)) {
        standalone.push(end);
        continue;
      }
      if (!groups.has(body)) groups.set(body, { gradient: new Vector3(), spins: new Map() });
      const group = groups.get(body);
      if (parent === undefined) group.gradient.add(end.angular);
      if (end.spin) {
        const previous = group.spins.get(end.spin.entityId)?.gradient ?? 0;
        group.spins.set(end.spin.entityId, { gradient: previous + end.spinGradient, inverse: end.inverse });
      }
    }
    let denominator = 0;
    const corrections = [];
    for (const [body, group] of groups) {
      const orientation = world.getComponent(body, OrientationComponent)?.quaternion;
      const moment = world.getComponent(body, MomentOfInertiaComponent);
      if (!orientation) continue;
      const tensor = transformInertiaTensorToWorld(moment?.inertiaTensor ?? [[0, 0, 0], [0, 0, 0], [0, 0, 0]], orientation);
      const rhs = group.gradient.clone(), members = [];
      for (const id of rotors.get(body)) {
        const rotor = getRotor(id), load = group.spins.get(id);
        const inverse = load?.inverse ?? rotor.invInertia;
        const gradient = load?.gradient ?? 0;
        const q = getEntityWorldOrientation(world, id);
        const coupling = multiplyMatrix3Vector(transformInertiaTensorToWorld(
          world.getComponent(id, MomentOfInertiaComponent).inertiaTensor, q,
        ), rotor.axisWorld);
        const c = [coupling.x, coupling.y, coupling.z];
        for (let row = 0; row < 3; row++) for (let col = 0; col < 3; col++) {
          tensor[row][col] -= inverse * c[row] * c[col];
        }
        rhs.add(coupling, -inverse * gradient);
        members.push({ id, inverse, gradient, coupling, axis: rotor.axisWorld });
      }
      const angular = multiplyMatrix3Vector(invertMatrix3(tensor), rhs);
      denominator += group.gradient.dot(angular);
      for (const rotor of members) {
        rotor.angle = rotor.inverse * (rotor.gradient - rotor.coupling.dot(angular));
        denominator += rotor.gradient * rotor.angle;
      }
      corrections.push({ body, angular, members, orientation: orientation.clone() });
    }
    for (const end of standalone) {
      denominator += end.angularDenominator + end.inverse * end.spinGradient * end.spinGradient;
    }
    return {
      denominator,
      standalone,
      apply(multiplier, applyStandalone, recordEncoder) {
        for (const { body, angular, members, orientation } of corrections) {
          const delta = angular.clone().scale(-multiplier), angle = delta.length();
          const q = world.getComponent(body, OrientationComponent).quaternion;
          if (angle > 1e-12) q.multiplyQuaternions(new Quaternion().setFromAxisAngle(delta.scale(1 / angle), angle), q).normalize();
          for (const rotor of members) {
            const angle = -multiplier * rotor.angle;
            if (Math.abs(angle) <= 1e-12) continue;
            const member = world.getComponent(rotor.id, RigidBodyMemberComponent);
            const oldLocal = member.localOrientation.clone();
            const localAxis = orientation.clone().conjugate().normalize().transformVector(rotor.axis);
            member.localOrientation.multiplyQuaternions(new Quaternion().setFromAxisAngle(localAxis, angle), member.localOrientation).normalize();
            // Keep the raw member pose in its last synchronized carrier frame.
            // RigidBodySyncSystem transports that frame once after the solve.
            const raw = world.getComponent(rotor.id, OrientationComponent).quaternion;
            raw.multiply(oldLocal.conjugate().normalize()).multiply(member.localOrientation).normalize();
            recordEncoder(rotor.id, angle);
          }
        }
        for (const end of standalone) applyStandalone(end);
      },
    };
  };
}
