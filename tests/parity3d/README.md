# Headless differential 3D checks

The Rust engine checks run in CI through `scripts/run_ci_tests.sh`: fresh native
and Wasm builds, a shared-source contract, native smoke/lifetime checks and short
Wasm/native differentials. To run just those checks locally:

```bash
npm run build:native -- --no-pgo
npm run build:wasm
cargo test --manifest-path src/wasm/cable_solver/Cargo.toml --locked
node --expose-gc --test tests/benchmark/native_physics_engine.test.mjs \
  tests/benchmark/physics_engine_ui.test.mjs tests/benchmark/profile_headless.test.mjs
```

`tests/parity3d/oracle.mjs` loads a JSON fixture into the production JavaScript
scene layer and runs the production Rust system registration. It does not
contain a second physics implementation. Fixtures under
`tests/fixtures/engine_parity/` compare native and Wasm behavior for selected
mounted-rotor, open-loop torque and topology cases.

The browser-versus-native collection check is opt-in and requires a built
`RRF/build/rrf_simulator`, Chromium and Vite:

```bash
node tests/parity3d/browser_collection.mjs HEADLESS_DATASET NEW_OUTPUT_DIR
```

`hp-sim-3d/app/headlessWorld.js` is the shared construction path for tests,
research and benchmarks. `npm run test:physics` is the required bounded physics
gate; it rebuilds both artifacts and checks the production HP4 logo plus loaded
settling at browser defaults.
