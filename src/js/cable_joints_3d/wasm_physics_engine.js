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
      step(dt) { exports.engine_step(handle, dt); return exports.engine_topology_version(handle); },
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
