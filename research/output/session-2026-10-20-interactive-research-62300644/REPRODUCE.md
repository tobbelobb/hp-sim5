# Reproduce the HP4 mechanics experiments

Run from the repository root at revision `c6f0b5a1bb501100706ecdc5e47294b74dac5ab8`, with the repository's `.venv` and Node installed. All worlds below are standalone and fresh. There is no RRF collection, autocal fitting, or live supervisor motion. Use a new result label for each repetition because the harness writes `<label>.json`, input USD and baked USD beside itself.

```bash
session=output/research/sessions/2026-10-20-interactive-research-62300644

# Frozen constitutive operation (overwrites the named E01 fixture result).
node "$session/mechanics.mjs" fixture

# Original native physics, 30 s held motors.
HP_SIM5_NATIVE_ENGINE="$PWD/$session/baseline-physics.node" \
  node "$session/mechanics.mjs" R01 stock native 1 hold 30

# No-slip native candidate, controlled 30 s pull, four substeps.
HP_SIM5_NATIVE_ENGINE="$PWD/$session/no-slip-physics.node" \
  node "$session/mechanics.mjs" R02 stock native 4 pull 30

# Faster motor plus no-slip native candidate, same pull.
HP_SIM5_NATIVE_ENGINE="$PWD/$session/motor-no-slip-physics.node" \
  node "$session/mechanics.mjs" R03 stock native 4 pull 30

# Two original pinholes plus an internal bridge; not a global gate solver.
HP_SIM5_NATIVE_ENGINE="$PWD/$session/no-slip-physics.node" \
  node "$session/mechanics.mjs" R04 gate native 4 pull 30

# JS alternatives do not require editing production source.
node "$session/mechanics.mjs" R05 no-rolling-slip js 1 hold 30
node "$session/mechanics.mjs" R06 motor js 1 pull 10 --no-slip
node "$session/mechanics.mjs" R07 accumulated-load js 4 pull 10 --no-slip
node "$session/mechanics.mjs" R08 accumulated-motor js 1 pull 10 --no-slip

# Analytic pulley comparison and synthetic two-port geometry/gradient checks.
node "$session/pulley-convergence.mjs"
.venv/bin/python "$session/gate_contract.py"
MPLCONFIGDIR=/tmp/hp-sim5-mpl .venv/bin/python "$session/analyze.py"
```

Archived native artifacts are explicit experiment choices; their SHA-256 and build-source hashes are in `*.node.json` and `provenance.json`. Using these binaries does not change the default addon. The model names of E29–E31 are resolved from the custom artifact, not their generic `stock` harness variant: these are **faster motor + no-slip** trials. E18–E27/E33–E35 originally ran while the no-slip source candidate was installed; the `--no-slip` flag reproduces its HP4 JS behavior after restoration. E36 provides that restored-source control. All HP4 rolling wheels have spool state, so the runtime exclusion of rolling redistribution is equivalent to the scoped candidate for this scene.

The harness dispatches A=-0.000334089 Nm and B=C=-0.037733 Nm at two simulated seconds, with D position held. `hold` dispatches no motor commands. Zero buildup, 30 mm spools and closed loop are explicit. Supported substeps are 1–16. E32's rejected 64-substep request did not advance physics.

`no-slip-candidate.patch` is the archived three-backend change plus regression tests and built Wasm; it is not accepted for production. `accumulated-feedback.patch` is the separate JS feedback experiment; its display-force fields still report the first visit. `candidate-torque.mjs` preserves the faster JS motor formula. `E17-candidate-core.mjs` preserves the passive-backdrive experiment; `accumulated-load-core.mjs` preserves the later feedback experiment. The default production tree was restored and the original binaries rebuilt.

The JSON rollouts sample complete path geometry/forces and wheel/motor angles every 0.1 simulated seconds, including the initial state. They do not contain RRDs. `solverTrace` captures only the last update of JS trials added for the multiplier audit; `multiplier-audit.json` computes the accumulated multiplier per timestep. No checkpoints restore a firmware state.

Re-running the named fixture/analysis programs overwrites their named result files. Preserve the original session or use a copy when extending the study. Use the frozen input/manifest hashes to distinguish repetitions from the original evidence.
