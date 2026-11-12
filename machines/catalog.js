// Machine catalogue: which scene, firmware configs and compiled commands belong together.
// machines/catalog.json is the single source of truth. Paths are relative to the repository root.
import catalog from './catalog.json' with { type: 'json' };

export const SCENES = catalog.scenes;
export const MACHINES = catalog.machines;

export function findMachine(name) {
  const key = String(name).trim().toLowerCase();
  return MACHINES.find((entry) => entry.name.toLowerCase() === key
    || entry.nicknames.some((nickname) => nickname.toLowerCase() === key)) || null;
}

export function machineForScene(scene, lineLayering) {
  return MACHINES.find((entry) => entry.scene === scene && entry.line_layering === lineLayering) || null;
}
