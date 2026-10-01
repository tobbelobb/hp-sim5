import {
  EncoderComponent, MachineTagComponent, PositionComponent, RenderableComponent,
  RigidBodyMemberComponent, SceneEntityInfoComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import {
  CableJointComponent, CableLinkComponent, CablePathComponent, cableStoredLengthAfterRotation,
} from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { getEntityWorldOrientation, getEntityWorldPosition } from '../../src/js/cable_joints_3d/rigid_bodies.js';
import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import { writeSlackCablePositions } from '../../src/js/cable_joints_3d/render_system_3d.js';
import { ExtruderComponent, estimateEffectorRotation } from './hangprinter_extruder.js';
import { StepperMotorComponent } from './hangprinter_stepper_motor.js';

const xyz = (v) => [v.x, v.y, v.z];
const xyzw = (q) => q ? [q.x, q.y, q.z, q.w] : [0, 0, 0, 1];
const pathPart = (name) => String(name).replace(/[^a-zA-Z0-9_.-]/g, '_');
const machineId = (world, id) => world.getComponent(id, MachineTagComponent)?.id || 'default';
const entityName = (world, id) => `${pathPart(world.getComponent(id, SceneEntityInfoComponent)?.name || 'entity')}_${id}`;
const machineRoot = (world, id) => `world/machines/${pathPart(machineId(world, id))}`;

function framePath(world, id) {
  const member = world.getComponent(id, RigidBodyMemberComponent);
  if (member) return `${framePath(world, member.bodyEntity)}/members/${entityName(world, id)}`;
  const info = world.getComponent(id, SceneEntityInfoComponent);
  const category = info?.tags.includes('Anchor') ? 'anchors' : 'bodies';
  return `${machineRoot(world, id)}/${category}/${entityName(world, id)}`;
}

function collectFrames(world) {
  const frames = world.query([PositionComponent]).map((id) => {
    const member = world.getComponent(id, RigidBodyMemberComponent);
    const info = world.getComponent(id, SceneEntityInfoComponent);
    return {
      path: framePath(world, id),
      name: info?.name || `entity ${id}`,
      position: xyz(member?.localPosition || getEntityWorldPosition(world, id)),
      quaternion: xyzw(member?.localOrientation || getEntityWorldOrientation(world, id)),
      kind: info?.tags.includes('Anchor') ? 'anchor' : 'body',
    };
  });
  for (const id of world.query([ExtruderComponent])) {
    const extruder = world.getComponent(id, ExtruderComponent);
    for (const [machine, sources] of Object.entries(extruder.centerSources)) {
      const positions = sources.map((source) => getEntityWorldPosition(world, source)).filter(Boolean);
      if (positions.length === 0) continue;
      const center = positions.reduce((sum, pos) => sum.add(pos), new Vector3()).scale(1 / positions.length);
      const parent = world.getComponent(sources[0], RigidBodyMemberComponent)?.bodyEntity;
      const commonBody = parent != null && sources.every((source) => (
        world.getComponent(source, RigidBodyMemberComponent)?.bodyEntity === parent
      ));
      const orientation = commonBody
        ? getEntityWorldOrientation(world, parent)
        : estimateEffectorRotation(extruder.centerSourceOffsets[machine], center, sources, world);
      frames.push({
        path: `world/machines/${pathPart(machine)}/effector`, name: 'Effector',
        position: xyz(center), quaternion: xyzw(orientation), kind: 'effector',
      });
    }
  }
  return frames;
}

// Sample the guide arc in its local cable plane, including its winding direction.
function guideWrap(world, path, index, before, after) {
  if (!['rolling', 'hybrid'].includes(path.linkTypes[index])) return null;
  const id = before.entityB;
  const center = getEntityWorldPosition(world, id);
  const link = world.getComponent(id, CableLinkComponent);
  const orientation = getEntityWorldOrientation(world, id);
  const normal = link?.cablePlaneNormalLocal && orientation
    ? orientation.transformVector(link.cablePlaneNormalLocal).normalize()
    : (link?.cablePlaneNormal || new Vector3(0, 0, 1)).clone().normalize();
  const start = before.attachmentPointB_world.clone().subtract(center);
  const end = after.attachmentPointA_world.clone().subtract(center);
  const radius = start.length();
  if (radius < 1e-9) return null;
  const u = start.clone().normalize();
  const v = normal.cross(u).normalize();
  let sweep = Math.atan2(end.dot(v), end.dot(u));
  if (path.cw[index] && sweep > 0) sweep -= Math.PI * 2;
  if (!path.cw[index] && sweep < 0) sweep += Math.PI * 2;
  const points = Array.from({ length: 17 }, (_, step) => {
    const angle = sweep * step / 16;
    return xyz(center.clone().add(u, radius * Math.cos(angle)).add(v, radius * Math.sin(angle)));
  });
  points[0] = xyz(before.attachmentPointB_world);
  points[16] = xyz(after.attachmentPointA_world);
  return points;
}

function collectCables(world) {
  const gravity = world.getResource('gravity');
  const up = gravity?.lengthSq() > 1e-9 ? gravity.clone().normalize().scale(-1) : new Vector3(0, 0, 1);
  return world.query([CablePathComponent]).map((id) => {
    const path = world.getComponent(id, CablePathComponent);
    const joints = path.jointEntities.map((jointId) => world.getComponent(jointId, CableJointComponent));
    const segments = joints.map((joint, index) => {
      const geometricLength = joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
      const subdivisions = joint.restLength > geometricLength + 1e-9 ? 16 : 1;
      const positions = new Float64Array((subdivisions + 1) * 3);
      writeSlackCablePositions(positions, joint.attachmentPointA_world, joint.attachmentPointB_world, joint.restLength, up, subdivisions);
      const force = Math.max(0, joint.constraintForceMagnitude || 0, joint.transferredConstraintForceMagnitude || 0);
      const direction = joint.attachmentPointB_world.clone().subtract(joint.attachmentPointA_world).normalize();
      return {
        name: entityName(world, path.jointEntities[index]),
        points: Array.from({ length: subdivisions + 1 }, (_, point) => Array.from(positions.subarray(point * 3, point * 3 + 3))),
        rest_length: joint.restLength,
        geometric_length: geometricLength,
        force_n: force, force_vector_n: xyz(direction.scale(force)), origin: xyz(joint.attachmentPointA_world),
      };
    });
    const guideLength = path.stored.slice(1, -1).reduce((sum, value) => sum + value, 0);
    const actual = segments.reduce((sum, segment) => sum + segment.rest_length, guideLength);
    const geometric = segments.reduce((sum, segment) => sum + segment.geometric_length, guideLength);
    let commanded = actual;
    if (joints.length > 0) {
      for (const [index, endpoint] of [[0, joints[0].entityA], [path.linkTypes.length - 1, joints.at(-1).entityB]]) {
        const motor = world.getComponent(endpoint, StepperMotorComponent);
        if (!motor) continue;
        const encoder = world.getComponent(endpoint, EncoderComponent);
        if (motor.torqueMode || !Number.isFinite(encoder?.angle)) {
          commanded = null;
          break;
        }
        const measured = encoder.angle - (motor.missedStepEncoderOffset || 0);
        const target = motor.commandedAngle - motor.deltaAngle;
        const targetStored = cableStoredLengthAfterRotation(world, path, index, endpoint, target - measured);
        commanded -= targetStored - (path.stored[index] || 0);
      }
    }
    const wraps = [];
    for (let index = 1; index < joints.length; index += 1) {
      const wrap = guideWrap(world, path, index, joints[index - 1], joints[index]);
      if (wrap) wraps.push(wrap);
    }
    return {
      machine: pathPart(machineId(world, id)), name: entityName(world, id),
      color: world.getComponent(path.jointEntities[0], RenderableComponent)?.color || '#ffff00',
      segments, wraps,
      lengths: { commanded, actual, geometric, error: commanded == null ? null : actual - commanded, stretch: geometric - actual },
    };
  });
}

export function captureFlightRecorderSnapshot(world) {
  return { frames: collectFrames(world), cables: collectCables(world) };
}
