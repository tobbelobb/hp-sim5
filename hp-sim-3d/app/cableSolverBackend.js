import { compileCableSolverWasm } from '../../src/js/cable_joints_3d/wasm_cable_solver.js';
import { WasmPhysicsEngine, setDefaultPhysicsModule } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { buildPublicProjectUrl } from './machineCatalog.js';

export async function loadDefaultCableSolverBackend(fetchImpl = globalThis.fetch, fetchThis = globalThis) {
  const response = await fetchImpl.call(fetchThis, buildPublicProjectUrl('wasm/cable_solver.wasm'));
  if (!response.ok) throw new Error(`Unable to load cable solver WASM: ${response.status}`);
  const module = await compileCableSolverWasm(await response.arrayBuffer());
  setDefaultPhysicsModule(module);
  return new WasmPhysicsEngine(module);
}
