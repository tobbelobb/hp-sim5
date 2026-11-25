import { RustPhysicsEngine } from './rust_physics_engine.js';
import { NativePhysicsEngine } from './native_physics_engine.js';
export { RustPhysicsSystem } from './rust_physics_engine.js';

let defaultModule;
export function setDefaultPhysicsModule(module) { defaultModule = module; }
export function createDefaultPhysicsBackend() {
  if (globalThis.process?.versions?.node) return new NativePhysicsEngine();
  if (!defaultModule) throw new Error('Load the Rust physics module before registering simulation systems');
  return new WasmPhysicsEngine(defaultModule);
}

export class WasmPhysicsEngine extends RustPhysicsEngine {
  physicsEngine = 'rust-wasm';
  constructor(module) {
    const exports = new WebAssembly.Instance(module, {}).exports;
    if (exports.abi_version() !== 3) throw new Error('Unsupported Rust physics Wasm ABI');
    let handle = 0, buffer;
    super({
      create(bodies, paths, count) {
        const ptr = exports.alloc_buffer(bodies.length + paths.length);
        const memory = new Float64Array(exports.memory.buffer);
        memory.set(bodies, ptr / 8); memory.set(paths, ptr / 8 + bodies.length);
        handle = exports.engine_create(ptr, bodies.length / 160, ptr + bodies.byteLength, count);
        exports.free_buffer(ptr, bodies.length + paths.length);
        buffer = exports.memory.buffer;
      },
      entities: () => new Float64Array(exports.memory.buffer, exports.engine_entities(handle), exports.engine_entity_count(handle) * 160),
      options: () => new Float64Array(exports.memory.buffer, exports.engine_options(handle), exports.engine_options_len()),
      path: index => new Float64Array(exports.memory.buffer, exports.engine_path(handle, index), exports.engine_path_len(handle, index)),
      setCableSpoolParams(index, kr, strainedMode, initTension) {
        exports.engine_set_cable_spool_params?.(handle, index, kr, strainedMode, initTension);
      },
      exportSpoolState(index) {
        if (!exports.engine_spool_state_len || !exports.engine_export_spool_state) return new Float64Array(0);
        const len = exports.engine_spool_state_len(handle, index) >>> 0;
        if (!len) return new Float64Array(0);
        const ptr = exports.alloc_buffer(len);
        const written = exports.engine_export_spool_state(handle, index, ptr, len) >>> 0;
        const copy = new Float64Array(exports.memory.buffer, ptr, written).slice();
        exports.free_buffer(ptr, len);
        return copy;
      },
      importSpoolState(index, data) {
        if (!exports.engine_import_spool_state || !data?.length) return false;
        const ptr = exports.alloc_buffer(data.length);
        new Float64Array(exports.memory.buffer, ptr, data.length).set(data);
        const ok = (exports.engine_import_spool_state(handle, index, ptr, data.length) >>> 0) !== 0;
        exports.free_buffer(ptr, data.length);
        return ok;
      },
      spoolGeomStored(index, side) {
        return exports.engine_spool_geom_stored ? exports.engine_spool_geom_stored(handle, index, side) : NaN;
      },
      predictSpoolStoredAfterRotation(index, side, deltaAngle, layering) {
        return exports.engine_predict_spool_stored_after_rotation
          ? exports.engine_predict_spool_stored_after_rotation(handle, index, side, deltaAngle, layering ? 1 : 0)
          : NaN;
      },
      step(dt) {
        const invalidRow = exports.engine_step(handle, dt) >>> 0;
        if (invalidRow) throw new RangeError(`Non-finite physics position at entity row ${invalidRow - 1}`);
        return exports.engine_topology_version(handle);
      },
      destroy() { if (handle) exports.engine_destroy(handle); handle = 0; },
      refresh(regions) {
        if (buffer === exports.memory.buffer) return;
        buffer = exports.memory.buffer;
        for (const region of regions) region.data = region.read();
      },
    });
    this.exports = exports;
  }
}
