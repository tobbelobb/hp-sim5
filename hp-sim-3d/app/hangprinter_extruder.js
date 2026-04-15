import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';
import {
  PositionComponent,
  MachineTagComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import { SpoolTagComponent } from './hangprinter_spools.js';

function isFiniteVector3(value) {
  return Boolean(value)
    && Number.isFinite(value.x)
    && Number.isFinite(value.y)
    && Number.isFinite(value.z);
}

function chooseReferenceTriangle(points) {
  if (!Array.isArray(points) || points.length < 3) {
    return null;
  }
  const epsilon = 1e-12;
  for (let i = 0; i < points.length - 2; i += 1) {
    const a = points[i];
    if (!isFiniteVector3(a)) continue;
    for (let j = i + 1; j < points.length - 1; j += 1) {
      const b = points[j];
      if (!isFiniteVector3(b)) continue;
      const edge = b.clone().subtract(a);
      if (edge.lengthSq() <= epsilon) continue;
      for (let k = j + 1; k < points.length; k += 1) {
        const c = points[k];
        if (!isFiniteVector3(c)) continue;
        const span = c.clone().subtract(a);
        if (edge.clone().cross(span).lengthSq() > epsilon) {
          return [i, j, k];
        }
      }
    }
  }
  return null;
}

function buildOrthonormalFrame(a, b, c) {
  const x = b.clone().subtract(a);
  if (x.lengthSq() <= 1e-12) {
    return null;
  }
  x.normalize();
  const span = c.clone().subtract(a);
  const z = x.clone().cross(span);
  if (z.lengthSq() <= 1e-12) {
    return null;
  }
  z.normalize();
  const y = z.clone().cross(x).normalize();
  return { x, y, z };
}

function quaternionFromRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22) {
  const trace = m00 + m11 + m22;
  if (trace > 0.0) {
    const s = Math.sqrt(trace + 1.0) * 2.0;
    return new Quaternion(
      (m21 - m12) / s,
      (m02 - m20) / s,
      (m10 - m01) / s,
      0.25 * s
    ).normalize();
  }
  if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1.0 + m00 - m11 - m22) * 2.0;
    return new Quaternion(
      0.25 * s,
      (m01 + m10) / s,
      (m02 + m20) / s,
      (m21 - m12) / s
    ).normalize();
  }
  if (m11 > m22) {
    const s = Math.sqrt(1.0 + m11 - m00 - m22) * 2.0;
    return new Quaternion(
      (m01 + m10) / s,
      0.25 * s,
      (m12 + m21) / s,
      (m02 - m20) / s
    ).normalize();
  }
  const s = Math.sqrt(1.0 + m22 - m00 - m11) * 2.0;
  return new Quaternion(
    (m02 + m20) / s,
    (m12 + m21) / s,
    0.25 * s,
    (m10 - m01) / s
  ).normalize();
}

function estimateRotation(referenceOffsets, currentCenter, entityIds, world) {
  if (!Array.isArray(referenceOffsets) || referenceOffsets.length < 3 || !isFiniteVector3(currentCenter)) {
    return new Quaternion();
  }
  const currentOffsets = entityIds.map((entityId) => {
    const pos = world.getComponent(entityId, PositionComponent)?.pos;
    return pos ? pos.clone().subtract(currentCenter) : null;
  });
  if (currentOffsets.length !== referenceOffsets.length || currentOffsets.some((offset) => !isFiniteVector3(offset))) {
    return new Quaternion();
  }
  const triangle = chooseReferenceTriangle(referenceOffsets);
  if (!triangle) {
    return new Quaternion();
  }
  const [i, j, k] = triangle;
  const restFrame = buildOrthonormalFrame(referenceOffsets[i], referenceOffsets[j], referenceOffsets[k]);
  const currentFrame = buildOrthonormalFrame(currentOffsets[i], currentOffsets[j], currentOffsets[k]);
  if (!restFrame || !currentFrame) {
    return new Quaternion();
  }

  const m00 = currentFrame.x.x * restFrame.x.x + currentFrame.y.x * restFrame.y.x + currentFrame.z.x * restFrame.z.x;
  const m01 = currentFrame.x.x * restFrame.x.y + currentFrame.y.x * restFrame.y.y + currentFrame.z.x * restFrame.z.y;
  const m02 = currentFrame.x.x * restFrame.x.z + currentFrame.y.x * restFrame.y.z + currentFrame.z.x * restFrame.z.z;
  const m10 = currentFrame.x.y * restFrame.x.x + currentFrame.y.y * restFrame.y.x + currentFrame.z.y * restFrame.z.x;
  const m11 = currentFrame.x.y * restFrame.x.y + currentFrame.y.y * restFrame.y.y + currentFrame.z.y * restFrame.z.y;
  const m12 = currentFrame.x.y * restFrame.x.z + currentFrame.y.y * restFrame.y.z + currentFrame.z.y * restFrame.z.z;
  const m20 = currentFrame.x.z * restFrame.x.x + currentFrame.y.z * restFrame.y.x + currentFrame.z.z * restFrame.z.x;
  const m21 = currentFrame.x.z * restFrame.x.y + currentFrame.y.z * restFrame.y.y + currentFrame.z.z * restFrame.z.y;
  const m22 = currentFrame.x.z * restFrame.x.z + currentFrame.y.z * restFrame.y.z + currentFrame.z.z * restFrame.z.z;

  return quaternionFromRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22);
}

export class ExtruderComponent {
  constructor() {
    this.extrusions = [];
    this.effectorCenterPos = new Vector3(0.0, 0.0, 0.0);
    this.centerPos = new Vector3(0.0, 0.0, 0.0);
    this.tipPos = null;
    this.coldEndPos = null;
    this.machineEffectorCenters = {};
    this.machineCenters = {};
    this.machineTips = {};
    this.machineColdEnds = {};
    this.centerSources = {};
    this.centerSourceOffsets = {};
    this.centerOffsets = {};
    this.tipOffsets = {};
    this.coldEndOffsets = {};
  }
}

export class ExtruderSystem {
  update(world, dt) {
    let extruderComp = null;
    for (const entityId of world.query([ExtruderComponent])) {
      extruderComp = world.getComponent(entityId, ExtruderComponent);
      break;
    }

    if (!extruderComp) {
      return;
    }

    const spoolEntities = world.query([SpoolTagComponent, PositionComponent]);
    const sumByMachine = {};
    const countByMachine = {};

    for (const entityId of spoolEntities) {
      const pos = world.getComponent(entityId, PositionComponent)?.pos;
      if (!pos) {
        continue;
      }
      const machineTag = world.getComponent(entityId, MachineTagComponent);
      const machineId = machineTag?.id || 'default';
      let sum = sumByMachine[machineId];
      if (!sum) {
        sum = new Vector3();
        sumByMachine[machineId] = sum;
        countByMachine[machineId] = 0;
      }
      sum.add(pos);
      countByMachine[machineId] += 1;
    }

    const centerSources = extruderComp.centerSources && typeof extruderComp.centerSources === 'object'
      ? extruderComp.centerSources
      : {};
    const sourceMachineIds = Object.keys(centerSources);

    const resolveAverage = (entityIds) => {
      if (!Array.isArray(entityIds) || entityIds.length === 0) {
        return null;
      }
      const sum = new Vector3();
      let count = 0;
      for (const entityId of entityIds) {
        const pos = world.getComponent(entityId, PositionComponent)?.pos;
        if (pos) {
          sum.add(pos);
          count += 1;
        }
      }
      if (count === 0) {
        return null;
      }
      return sum.clone().scale(1.0 / count);
    };

    const centerOffsets = extruderComp.centerOffsets && typeof extruderComp.centerOffsets === 'object'
      ? extruderComp.centerOffsets
      : extruderComp.extruderOffsets && typeof extruderComp.extruderOffsets === 'object'
        ? extruderComp.extruderOffsets
        : {};
    const centerSourceOffsets = extruderComp.centerSourceOffsets && typeof extruderComp.centerSourceOffsets === 'object'
      ? extruderComp.centerSourceOffsets
      : {};
    const tipOffsets = extruderComp.tipOffsets && typeof extruderComp.tipOffsets === 'object'
      ? extruderComp.tipOffsets
      : {};
    const coldEndOffsets = extruderComp.coldEndOffsets && typeof extruderComp.coldEndOffsets === 'object'
      ? extruderComp.coldEndOffsets
      : {};
    const machineEffectorCenters = {};
    const machineCenters = {};
    const machineTips = {};
    const machineColdEnds = {};
    for (const machineId of sourceMachineIds) {
      const entityIds = centerSources[machineId];
      let effectorCenter = resolveAverage(entityIds);
      if (!effectorCenter && sumByMachine[machineId] && (countByMachine[machineId] ?? 0) > 0) {
        effectorCenter = sumByMachine[machineId].clone().scale(1.0 / countByMachine[machineId]);
      }
      if (effectorCenter) {
        machineEffectorCenters[machineId] = effectorCenter.clone();
        const rotation = estimateRotation(centerSourceOffsets[machineId], effectorCenter, entityIds, world);
        let rootPos = effectorCenter.clone();
        const offset = centerOffsets[machineId];
        if (isFiniteVector3(offset)) {
          rootPos.add(rotation.transformVector(offset));
        }
        machineCenters[machineId] = rootPos;

        const tipOffset = tipOffsets[machineId];
        const tipPos = rootPos.clone();
        if (isFiniteVector3(tipOffset)) {
          tipPos.add(rotation.transformVector(tipOffset));
        }
        machineTips[machineId] = tipPos;

        const coldEndOffset = coldEndOffsets[machineId];
        if (isFiniteVector3(coldEndOffset)) {
          machineColdEnds[machineId] = rootPos.clone().add(rotation.transformVector(coldEndOffset));
        }
      }
    }

    if (sourceMachineIds.length === 0) {
      for (const machineId of Object.keys(sumByMachine)) {
        const count = countByMachine[machineId] ?? 0;
        if (count > 0) {
          const effectorCenter = sumByMachine[machineId].clone().scale(1.0 / count);
          machineEffectorCenters[machineId] = effectorCenter.clone();
          const rootPos = effectorCenter.clone();
          const offset = centerOffsets[machineId];
          if (isFiniteVector3(offset)) {
            rootPos.add(offset);
          }
          machineCenters[machineId] = rootPos;

          const tipOffset = tipOffsets[machineId];
          const tipPos = rootPos.clone();
          if (isFiniteVector3(tipOffset)) {
            tipPos.add(tipOffset);
          }
          machineTips[machineId] = tipPos;

          const coldEndOffset = coldEndOffsets[machineId];
          if (isFiniteVector3(coldEndOffset)) {
            machineColdEnds[machineId] = rootPos.clone().add(coldEndOffset);
          }
        }
      }
    } else {
      for (const machineId of Object.keys(sumByMachine)) {
        if (machineCenters[machineId]) {
          continue;
        }
        const count = countByMachine[machineId] ?? 0;
        if (count > 0) {
          const effectorCenter = sumByMachine[machineId].clone().scale(1.0 / count);
          machineEffectorCenters[machineId] = effectorCenter.clone();
          const rootPos = effectorCenter.clone();
          const offset = centerOffsets[machineId];
          if (isFiniteVector3(offset)) {
            rootPos.add(offset);
          }
          machineCenters[machineId] = rootPos;

          const tipOffset = tipOffsets[machineId];
          const tipPos = rootPos.clone();
          if (isFiniteVector3(tipOffset)) {
            tipPos.add(tipOffset);
          }
          machineTips[machineId] = tipPos;

          const coldEndOffset = coldEndOffsets[machineId];
          if (isFiniteVector3(coldEndOffset)) {
            machineColdEnds[machineId] = rootPos.clone().add(coldEndOffset);
          }
        }
      }
    }

    const machineIds = Object.keys(machineCenters);
    if (machineIds.length > 0) {
      extruderComp.machineEffectorCenters = machineEffectorCenters;
      extruderComp.machineCenters = machineCenters;
      extruderComp.machineTips = machineTips;
      extruderComp.machineColdEnds = machineColdEnds;
      const preferredOrder = sourceMachineIds.length > 0 ? sourceMachineIds : machineIds;
      let chosenId = preferredOrder.find((id) => machineCenters[id]) || null;
      if (!chosenId) {
        chosenId = machineIds[0];
      }
      extruderComp.effectorCenterPos = (machineEffectorCenters[chosenId] || machineCenters[chosenId]).clone();
      extruderComp.centerPos = machineCenters[chosenId].clone();
      extruderComp.tipPos = (machineTips[chosenId] || machineCenters[chosenId]).clone();
      extruderComp.coldEndPos = machineColdEnds[chosenId]?.clone?.() || null;
    }
  }
}
