# HP4 physics substeps

Measured on 2026-11-12, starting at `9f4351b9`, after both firmware flex fixes.
The scene catalogue selects four physics substeps for HP4, with the existing
two cable solver iterations. HP3 and the other scenes retain one substep.
The firmware tick, command dispatch and recording cadence remain 2 ms; HP4's
physical integrations are 0.5 ms each.

[Small Steps](../../ai_docs/Smallsteps/Smallsteps.md) recommends spending solver
work on smaller integrations to reduce constraint error and numerical damping.
The local experiments support substepping, but HP4 still needs two iterations
per substep: one iteration with four or eight substeps leaves appreciable
pretension bias. Four substeps with two iterations is the first tested substep
setting to bring the static XY bias below 0.01 mm.

The shared scene loader reads `machine:catalog` from the authored scene root
and resolves `physics_substeps` from `machines/catalog.json` (default one).
This works for browser scenes, baked scenes and headless tools, including
copies at other file paths. Multiple machines use their largest default.
An explicit UI, remote or headless override survives scene rebuilds and resets.

## Static pretension

At home, with no layering and closed-loop motors, ramp firmware line positions
(-0.56587, -0.56587, -0.56587, -0.08332) mm over one second, then hold four
seconds. Convert to spool angles using MA (2,2,2,4) and 30 mm radius. This is
the MA²-corrected flex probe from the original research.

| Iterations | Substeps | XY bias (mm) |
|---:|---:|---:|
| 2 | 1 | 0.2280 |
| 2 | 2 | 0.0455 |
| **2** | **4** | **0.0057** |
| 2 | 8 | 0.0006 |
| 6 | 1 | 0.0287 |
| 10 | 1 | 0.0038 |
| 20 | 1 | <0.0001 |
| 1 | 4 | 0.0477 |
| 1 | 8 | 0.0166 |

ABC strand tensions rise from about 3.3 N at the old default to 4.96 N at
four substeps; eight substeps gives 4.99 N. The permanent native/Wasm
pretension gate requires XY bias below 0.01 mm and tensions within 2% of
the converged approximately 5 N ABC / 0.795 N D values.

## Complete logo

Current bundled unlayered CAN streams, closed-loop motors, production
QualityMonitor and G-code parser; 1,038,258 firmware ticks per replay.
Path values below are `contourRms`, `contourP95` and `contourMax` in mm.

| Machine | Iterations | Substeps | Path RMS / 95% / max (mm) | Mean Y (mm) | Physics wall time (s) |
|---|---:|---:|---|---:|---:|
| HP4 | 2 | 1 | 0.212 / 0.356 / 0.427 | -0.209 | 79 |
| **HP4** | **2** | **4** | **0.083 / 0.149 / 0.349** | **-0.005** | **282** |
| HP4 | 2 | 8 | 0.090 / 0.155 / 0.470 | -0.0003 | 559 |
| HP4 | 10 | 1 | 0.071 / 0.135 / 0.203 | -0.008 | 138 |
| HP3 | 1 | 1 | 0.097 / 0.143 / 0.225 | +0.099 | 23 |

All these runs had zero missed steps. Four substeps puts 99.79% of the HP4
path within 0.25 mm, compared with 74.68% at one substep. Its mean XYZ offset
is (0.025, -0.005, 0.002) mm.

Ten iterations at one substep is cheaper in this implementation and gives
lower path errors on this trajectory. Four substeps is the default because it
also reduces integration error and artificial damping, as recommended by
Small Steps. Extra iterations converge constraints within a 2 ms step while
retaining its integration error. Eight substeps brings little further static
benefit and exposes more high-frequency motion. These results support four
as a practical default; they do not establish timestep-independent dynamics.
The individual wall times are indicative, with some verification running on
other cores, rather than controlled repeated performance benchmarks.

## Stability and validation

The first 20 seconds of the layered, open-loop logo are gated in native and
Wasm at both the catalogue default and an explicit one-substep override.
At four substeps, less numerical damping raises the opening transient from
about 0.058 to 0.120 rad/s; a sixteen-substep reference reaches 0.196 rad/s.
Tilt stays below 0.002 rad (0.092 degrees observed) and motion decays to rest.
The four-substep gate bounds angular speed at 0.15 rad/s, adds the tilt bound,
and requires settling below 1e-6 rad/s and 1e-8 m/s. The old one-substep
0.1 rad/s bound remains. Loaded-checkpoint settling still covers one, two and
four substeps and the 0.5/4 Nm motor variants.

The complete layered, open-loop logo also finishes all 1,038,258 ticks with
zero missed steps and finite state. Peak speed is 0.373 m/s, peak angular speed
0.201 rad/s and maximum height change 0.672 mm. Its path RMS / 95% / max is
0.544 / 1.124 / 3.077 mm, compared with 0.644 / 1.317 / 3.368 mm at the old
one-substep default (also zero missed steps). This is a different winding and
motor configuration from the unlayered, closed-loop accuracy comparison above.

Passed: `npm run test:physics`, the same physics gates with the checked-in
Wasm binary, 326 Jest tests, 50 backend integration tests, native-header build
checks, Rust tests, 725 Python unit tests, 122 slow integration tests and all
six RRF HTTP integration groups. Browser checks confirmed HP4=4, HP3=1,
mixed-machine defaults and preservation of a manual override across resets.
The broad Python run also exposed obsolete research launch references, which
were removed before the clean run.

Numeric results are saved in [hp4_substeps_results.json](hp4_substeps_results.json).
Reproduce full replays with [logo_accuracy.mjs](logo_accuracy.mjs), for example:

```bash
node tests/benchmark/logo_accuracy.mjs --machine hp4
node tests/benchmark/logo_accuracy.mjs --machine hp4 --substeps 1
node tests/benchmark/logo_accuracy.mjs --machine hp4 --substeps 8
node tests/benchmark/logo_accuracy.mjs --machine hp4 --substeps 1 --iterations 10
node tests/benchmark/logo_accuracy.mjs --machine hp3
node tests/benchmark/logo_accuracy.mjs --machine hp4_w_line_layers --open-loop
```

The bounded static reproduction is `node tests/benchmark/hp4_pretension.test.mjs`.
All results are simulation only.
