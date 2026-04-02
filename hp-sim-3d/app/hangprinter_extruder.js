import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import {
  PositionComponent,
  MachineTagComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import { SpoolTagComponent } from './hangprinter_spools.js';

export class ExtruderComponent {
  constructor() {
    this.extrusions = [];
    this.centerPos = new Vector3(0.0, 0.0, 0.0);
    this.machineCenters = {};
    this.centerSources = {};
    this.centerOffsets = {};
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
    const machineCenters = {};
    for (const machineId of sourceMachineIds) {
      const entityIds = centerSources[machineId];
      let center = resolveAverage(entityIds);
      if (!center && sumByMachine[machineId] && (countByMachine[machineId] ?? 0) > 0) {
        center = sumByMachine[machineId].clone().scale(1.0 / countByMachine[machineId]);
      }
      if (center) {
        const offset = centerOffsets[machineId];
        if (offset && Number.isFinite(offset.x) && Number.isFinite(offset.y) && Number.isFinite(offset.z)) {
          center = center.clone().add(offset);
        }
        machineCenters[machineId] = center;
      }
    }

    if (sourceMachineIds.length === 0) {
      for (const machineId of Object.keys(sumByMachine)) {
        const count = countByMachine[machineId] ?? 0;
        if (count > 0) {
          const center = sumByMachine[machineId].clone().scale(1.0 / count);
          const offset = centerOffsets[machineId];
          if (offset && Number.isFinite(offset.x) && Number.isFinite(offset.y) && Number.isFinite(offset.z)) {
            center.add(offset);
          }
          machineCenters[machineId] = center;
        }
      }
    } else {
      for (const machineId of Object.keys(sumByMachine)) {
        if (machineCenters[machineId]) {
          continue;
        }
        const count = countByMachine[machineId] ?? 0;
        if (count > 0) {
          const center = sumByMachine[machineId].clone().scale(1.0 / count);
          const offset = centerOffsets[machineId];
          if (offset && Number.isFinite(offset.x) && Number.isFinite(offset.y) && Number.isFinite(offset.z)) {
            center.add(offset);
          }
          machineCenters[machineId] = center;
        }
      }
    }

    const machineIds = Object.keys(machineCenters);
    if (machineIds.length > 0) {
      extruderComp.machineCenters = machineCenters;
      const preferredOrder = sourceMachineIds.length > 0 ? sourceMachineIds : machineIds;
      let chosenId = preferredOrder.find((id) => machineCenters[id]) || null;
      if (!chosenId) {
        chosenId = machineIds[0];
      }
      extruderComp.centerPos = machineCenters[chosenId].clone();
    }
  }
}
