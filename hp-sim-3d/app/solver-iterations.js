import { CablePathComponent } from '../../src/js/cable_joints_3d/cable_joints_core.js';

// Apply an explicit world-wide override, or restore each path's machine default.
export function setSolverIterations(world, count) {
  if (count !== null && (!Number.isInteger(count) || count < 1 || count > 64)) {
    throw new RangeError('solverIterations must be an integer from 1 to 64, or null for machine defaults');
  }
  world.setResource('solverIterationsOverride', count);
  return syncSolverIterations(world);
}

export function syncSolverIterations(world) {
  const override = world.getResource('solverIterationsOverride');
  let maximum = 1;
  for (const id of world.query([CablePathComponent])) {
    const path = world.getComponent(id, CablePathComponent);
    path.defaultSolverIterations ??= path.solverIterations;
    path.solverIterations = override ?? path.defaultSolverIterations;
    maximum = Math.max(maximum, path.defaultSolverIterations);
  }
  world.setResource('machineSolverIterations', maximum);
  world.setResource('solverIterations', override ?? maximum);
  return override ?? maximum;
}
