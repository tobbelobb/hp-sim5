import { compileCableSolverWasm, WasmCableSolver } from '../../src/js/cable_joints_3d/wasm_cable_solver.js';
import { buildPublicProjectUrl } from './machineCatalog.js';

export async function loadDefaultCableSolverBackend(fetchImpl = globalThis.fetch, fetchThis = globalThis) {
  const response = await fetchImpl.call(fetchThis, buildPublicProjectUrl('wasm/cable_solver.wasm'));
  if (!response.ok) throw new Error(`Unable to load cable solver WASM: ${response.status}`);
  return new WasmCableSolver(await compileCableSolverWasm(await response.arrayBuffer()));
}
