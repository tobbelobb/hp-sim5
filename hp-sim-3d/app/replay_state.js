export function cloneExtrusionList(list) {
  if (!Array.isArray(list) || list.length === 0) {
    return [];
  }
  return list.map((entry) => ({
    ...entry,
    pos: Array.isArray(entry?.pos) ? [...entry.pos] : entry?.pos,
  }));
}

export function collectActiveExtrusionMachineIds(extruderComp) {
  if (!extruderComp || typeof extruderComp !== 'object') {
    return [];
  }

  const centerSources = extruderComp.centerSources && typeof extruderComp.centerSources === 'object'
    ? Object.keys(extruderComp.centerSources)
    : [];
  if (centerSources.length > 0) {
    return centerSources;
  }

  const machineCenters = extruderComp.machineCenters && typeof extruderComp.machineCenters === 'object'
    ? Object.keys(extruderComp.machineCenters)
    : [];
  return machineCenters;
}

export function filterExtrusionsForMachines(list, machineIds) {
  const extrusions = cloneExtrusionList(list);
  const active = Array.isArray(machineIds) ? machineIds.filter((id) => typeof id === 'string' && id.length > 0) : [];
  if (active.length === 0) {
    return extrusions;
  }
  const activeSet = new Set(active);
  return extrusions.filter((entry) => {
    const machineId = typeof entry?.machineId === 'string' ? entry.machineId : null;
    return !machineId || activeSet.has(machineId);
  });
}

export function restoreReplayExtrusions(extruderComp, extrusionSnapshot) {
  if (!extruderComp || typeof extruderComp !== 'object') {
    return [];
  }
  const activeMachineIds = collectActiveExtrusionMachineIds(extruderComp);
  const restored = filterExtrusionsForMachines(extrusionSnapshot, activeMachineIds);
  extruderComp.extrusions = restored;
  return restored;
}
