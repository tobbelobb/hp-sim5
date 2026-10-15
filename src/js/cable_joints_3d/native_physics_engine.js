import { RustPhysicsEngine } from './rust_physics_engine.js';

// Keep native-only paths out of Vite's browser asset discovery.
const moduleUrl = import.meta.url;
const defaultArtifact = new URL('../../wasm/cable_solver/target/native/physics.node', moduleUrl);
const modules = new Map();

// Builtins are resolved only in Node so the shared pipeline remains browser-safe.
export function loadNativePhysicsAddon(addonPath = globalThis.process?.env?.HP_SIM5_NATIVE_ENGINE ?? defaultArtifact) {
  if (!globalThis.process?.versions?.node) throw new Error('Native physics requires Node');
  const { createRequire } = process.getBuiltinModule('module');
  const { fileURLToPath } = process.getBuiltinModule('url');
  const { resolve } = process.getBuiltinModule('path');
  const fs = process.getBuiltinModule('fs');
  const artifact = addonPath instanceof URL ? fileURLToPath(addonPath) : resolve(addonPath);
  if (!modules.has(artifact)) {
    if (!fs.existsSync(artifact)) throw new Error(`Native Rust physics is missing: ${artifact}. Run npm run build:native`);
    const { createHash } = process.getBuiltinModule('crypto');
    const sha = bytes => createHash('sha256').update(bytes).digest('hex');
    const build = fs.existsSync(`${artifact}.json`) ? JSON.parse(fs.readFileSync(`${artifact}.json`)) : null;
    const artifactSha256 = sha(fs.readFileSync(artifact));
    // A custom artifact is an explicit experiment choice. The default must
    // match this checkout; never silently profile stale code or fall back.
    if (artifact === fileURLToPath(defaultArtifact)) {
      const crate = new URL('../../wasm/cable_solver/', moduleUrl);
      if (!build || build.artifactSha256 !== artifactSha256 ||
          Object.entries(build.sources).some(([file, hash]) => sha(fs.readFileSync(new URL(file, crate))) !== hash) ||
          (build.pgo && build.pgo.trainerSha256 !== sha(fs.readFileSync(new URL('../../../scripts/train_native_physics.mjs', moduleUrl)))) ||
          build.buildScriptSha256 !== sha(fs.readFileSync(new URL('../../../scripts/build_native_physics.mjs', moduleUrl)))) {
        throw new Error('Native Rust physics build is stale. Run npm run build:native');
      }
    }
    const addon = createRequire(import.meta.url)(artifact);
    modules.set(artifact, { addon, artifact, info: { path: artifact, sha256: artifactSha256, build } });
  }
  return modules.get(artifact);
}

export class NativePhysicsEngine extends RustPhysicsEngine {
  physicsEngine = 'rust-native';
  constructor({ addonPath } = {}) {
    const { addon, artifact, info } = loadNativePhysicsAddon(addonPath);
    const { markAsUntransferable } = process.getBuiltinModule('worker_threads');
    const view = data => { markAsUntransferable(data.buffer); return data; };
    let host;
    super({
      create(bodies, paths, count) { host = addon.create(bodies, paths, count); },
      entities: () => view(host.entities()), options: () => view(host.options()), path: index => view(host.path(index)),
      step: dt => host.step(dt), destroy: () => { host?.dispose(); host = null; },
    });
    this.artifact = artifact;
    this.artifactInfo = info;
  }
}
