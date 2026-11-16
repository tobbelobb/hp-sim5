# Five-anchor validation battery

Run from the repository root. These commands do not write `research/output/`.
Both scenes are unlayered. Printing uses catalogue defaults: open-loop motors,
one physics substep, one solver iteration for HP5 and ten for pulley-reduced
HP5. Autocal uses the catalogue's twenty solver iterations and closed-loop
motors. The autocal battery uses patience twenty so fit-score patience does not
stop acquisition before the two consecutive 0.5 mm stable-estimate checks.

Build firmware and regenerate both catalogue programs through the actual RRF
and Klipper planners:

```bash
cmake --build RRF/build --target rrf_simulator -j 4
.venv/bin/python scripts/compile_machine_commands.py --machine hp5 --machine hp5_pulley_reduced
```

The compiler writes the catalogue's `.can` and `.serial` files. Its temporary
RRF SD-card directory is isolated from interactive simulator sessions. Klipper
rebuilds its C helper automatically when that source changes. No Rust source
changes are needed for five anchors; the native and Wasm engines are exercised
below.

Regression checks:

```bash
.venv/bin/python -m pytest autocal/tests tests/python -m 'not slow' -q
.venv/bin/python -m pytest autocal/tests/test_ellipse_objective_jax.py -m slow -q
.venv/bin/python -m pytest tests/python/test_world_configuration.py -m slow -k disabled_layering -q
.venv/bin/python -m pytest tests/python/test_native_collector.py -m slow -k hp5_world -q
.venv/bin/python RRF/tests/test_flex_mode_position.py
.venv/bin/python -m pytest klipper/test/klippy/test_winch.py -q
npx jest --runInBand
node --test tests/benchmark/hp5_print.test.mjs tests/benchmark/hp5_pretension.test.mjs tests/benchmark/hp5_frame.test.mjs
```

The print regression plays complete `draw_squares_bigger` streams from both
firmwares through native and Wasm physics, checking all five decoded winches,
zero missed steps, tracking RMS below 0.4 mm and maximum contour error below
0.5 mm.

For measured full-logo runs (four commands; run sequentially for timings):

```bash
node tests/benchmark/logo_accuracy.mjs --machine hp5 --firmware rrf --open-loop
node tests/benchmark/logo_accuracy.mjs --machine hp5 --firmware klipper --open-loop
node tests/benchmark/logo_accuracy.mjs --machine hp5_pulley_reduced --firmware rrf --open-loop
node tests/benchmark/logo_accuracy.mjs --machine hp5_pulley_reduced --firmware klipper --open-loop
```

Each command prints JSON containing tracking/contour RMS, percentile and maximum
errors, missed steps, solver settings and physics wall time. Errors are in
metres; multiply by 1000 for millimetres. `--solver wasm` selects browser physics;
omitting `--open-loop` selects closed-loop motors.

Full physical autocal, including force tuning, sizing, rough bootstrap,
observability checks, equal-strain remeasurement, live acquisition and fitting:

```bash
mkdir -p /tmp/hp5-battery
HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=6000000 timeout -s INT 1000 .venv/bin/python autocal/autocal.py --machine-type hangprinter_5 --dataset /tmp/hp5-battery/hp5.json --headless-sim --recording-mode none --patience 20 --verbose
HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=6000000 timeout -s INT 1000 .venv/bin/python autocal/autocal.py --machine-type hangprinter_5_pulley_reduced --dataset /tmp/hp5-battery/hp5r.json --headless-sim --recording-mode none --patience 20 --verbose
```

Use fresh dataset paths for a complete bootstrap. Existing paths resume saved
data. Require `normal_completion: true` and `stop_reason: stable-estimate` in
`<dataset>.headless/manifest.json`; a timeout, exhausted budget, patience stop or
failed collection does not establish stable-stop. Read `wall_s` there for total
runtime and the `summary` row of `<dataset>.full_auto_log.jsonl` for fitted
anchors. Compare each fitted anchor to the configured firmware anchor, using the
Euclidean error per anchor without translation alignment.

Baselines used for comparison:

```bash
node tests/benchmark/logo_accuracy.mjs --machine hp3 --open-loop
node tests/benchmark/logo_accuracy.mjs --machine hp4 --open-loop
.venv/bin/python klipper/klippy/klippy.py public/klipper/hp3/printer-hp3-linux-mcu.cfg -i public/gcode/Hangprinter_logo6.gcode -o /tmp/hp3-logo.serial -d public/klipper/linux_mcu/klipper.dict
node tests/benchmark/logo_accuracy.mjs --machine hp3 --firmware klipper --commands /tmp/hp3-logo.serial --open-loop
HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=6000000 timeout -s INT 1000 .venv/bin/python autocal/autocal.py --machine-type hangprinter_3 --dataset /tmp/hp5-battery/hp3-baseline.json --headless-sim --recording-mode none
```

## Implementation and design checks

`generate_hp5_scenes.py` builds the direct and reduced scenes from HP3 and HP4
routing templates. Empty relationship targets remain unauthored: assigning an
empty target list exported USD `None`, which the browser parser turned into a
null scalar that the baker rejected. Both presets were selected independently
in a fresh browser tab with no bake/load errors on 2026-11-16.

All five owner design changes are implemented:

1. Each scene has exactly four `Attachment` prims at `(±0.25, ±0.25, 0.1)` m.
2. Direct low pairs use the two shared corners on their side; anchors add the
   unchanged firmware pivot vectors and the spools remain farther outward.
3. Both scenes have four top branches and twelve cable paths, with firmware
   parallel-line counts `2:2:2:2:4`. Guy lengths are 429.1 mm low and 352.3 mm top.
4. Reduced routing copies HP4 bearing/pinhole offsets and its low 2:1 / top 4:1
   mechanical advantages with 20:20 gears. Offset fixed/moving pivots preserve
   the specified firmware vectors, just as HP4's pivots preserve HP3's vectors;
   no additional firmware attachment-vector correction is needed. The reduced
   mover has four corner attachment points, 24 wheels and 16 pinholes (44 rigid
   members total); the direct mover has four members total. Both weigh 0.081 kg.
5. `rigidBodyBuilder.js` uses the authored frame-corner perimeter and connects
   every other member only to its own nearest corner. Bearings remain visible.
   HP3/HP4 have no new frame-corner relationship and retain their prior drawing.

RRF `HangprinterKinematics.cpp` and Klipper `kin_winch.c` now use their existing
bounded minimum-norm solver for the five-anchor force distribution. The obsolete
ridge active-set solver stalled on this layout and was deleted. Forward solves
and flex round trips are covered by real firmware tests. No Rust edits were
necessary; native and Wasm print streams agree numerically.

Autocal now keeps opposing unused low winches compliant, collects distinct fixed
circles, masks their slack encoder lengths during range fitting and planning,
and uses the HP3 gauge extended to four low anchors. The ordinary return path
uses low force for redundant machines, restores the carrying anchor to home
first, then holds it during low-axis returns. Holding a shortened top cable
while commanding multiple low home lengths was geometrically incompatible.
`configuration.py`, `session.py`, `experiments.py`, `validation.py`, `preflight.py`
and the collectors derive axes/counts from scene or firmware data. Catalog,
config resolution, raw-move workers and compiled CAN/MCU streams include I.

## Measured print results

Complete `draw_squares_bigger`, native physics, catalog defaults. Wall times are
physics replay times, measured while other validation jobs were running.

| Machine | Firmware | Tracking RMS mm | Contour RMS mm | Wall seconds |
|---|---|---:|---:|---:|
| hp5 | rrf | 0.2272 | 0.0933 | 1.37 |
| hp5 | klipper | 0.2609 | 0.0963 | 1.29 |
| hp5_pulley_reduced | rrf | 0.1278 | 0.1101 | 7.06 |
| hp5_pulley_reduced | klipper | 0.1385 | 0.1112 | 7.08 |

All eight native/Wasm × firmware × machine square replays passed: tracking RMS
below 0.4 mm, contour maximum below 0.5 mm, complete progress and zero missed
steps. Reproduce any row with:

```bash
node tests/benchmark/logo_accuracy.mjs --machine hp5 --firmware rrf --gcode draw_squares_bigger --open-loop
```

Substitute `hp5_pulley_reduced` and/or `klipper` for the other rows.

Complete-logo runs also had zero missed steps and contour maxima 0.176–0.228 mm.
RRF tracking RMS was 0.2148 / 0.2315 mm (direct/reduced), runtime 29.67 / 162.19 s.
Klipper tracking RMS was 1.2460 / 0.9775 mm, runtime 29.31 / 160.45 s; isolated
reference-tracking maxima of 633 / 605 mm inflate these RMS values. The HP3
Klipper baseline has the same isolated ~633 mm outlier. This pre-existing
reference/stream timing issue is not resolved here. Klipper 95th-percentile
tracking was 0.3707 / 0.4259 mm and the printed contour remained within 0.228 mm.

## Owner design checklist

1. Both movers have exactly four `Attachment` prims at `(±0.25, ±0.25, 0.1)` m.
2. Direct low pairs share their side's corners; anchors equal corner plus the
   unchanged firmware vector. Spools remain farther outward.
3. Both scenes have four I routes and twelve routes total. Both firmware configs
   use lines-per-axis `2:2:2:2:4`, with guy lengths `429.1:429.1:429.1:429.1:352.3` mm.
4. Reduced low wheels/pinholes retain the exact HP4 offsets relative to each
   corner, rotated with the axis; top bearings retain HP4's relative spread.
   Anchors move with those pinholes, preserving every configured pivot vector.
   Mechanical advantage is `2:2:2:2:4`, gear teeth `20:20` on all axes.
5. `rigidBodyBuilder.js` uses `rigidBody:frameCorners` for four perimeter edges
   and connects each remaining member only to its closest corner. Both scenes
   were visually checked in the browser. HP3/HP4 have no new frame rule.

Attachment counts are four in each scene. The direct mover has four total
members; reduced has 44: four corners, 24 wheels, and 16 pinholes. All wheels and
pinholes remain visible.

Final regression checks on 2026-11-16: 644 fast Python tests, 394 JavaScript tests,
8 JAX slow tests, 4 layering/baker tests, 11 RRF tests, 12 Klipper tests, and
23 HP4/HP5 native/Wasm print, pretension, and frame tests passed. No Rust source
changed, so no native/Wasm rebuild was required. Submodule-generated tags and
pre-existing untracked Klipper `In`, `done`, and `ai_docs/build/` are excluded.

Final review checks on 2026-11-16: 644 fast Python tests; 394 Jest tests; 23
native/Wasm HP4/HP5 physics tests; 11 RRF standstill/flex tests; 12 Klipper
kinematics tests; eight slow JAX objective tests; four production layering/bake
checks; and two full MCP five-axis collector checks all passed. No Rust source
was edited, so native/Wasm rebuilds were unnecessary. Their outputs agree in
the complete-square regressions. The HP3 full-autocal baseline reached stable
stop with maximum anchor error 1.037 mm in 369.57 s; a fresh full HP4 autocal
baseline was not rerun during this time-boxed pass. Both HP3/HP4 firmware tests
and the existing HP4 native/Wasm stability, pretension and logo checks passed.

## Autocal validation

The final return path restores the carrying I line first, then holds it while
returning low axes under tuned low force. Holding a shortened top line while
trying to restore multiple low lines to home imposes incompatible lengths.
The former generic return released I into 1 N torque mode; the intermediate
held-top patch retained the shortened I length and failed during rough return.
Focused tests now cover carrying-first ordering and both between-sweep/final
collector calls. HP3/HP4 retain their existing return behavior.

Final HP5 run (`/tmp/hp5-square-battery/hp5-final.json`): normal completion,
`stable-estimate`, 640.41 s. Euclidean errors for A/B/C/D/I without alignment:
`0.889, 1.653, 1.154, 1.037, 0.491` mm. Three initial rough sweeps plus two
additional rough sweeps passed the 11-parameter observability check; all five
were remeasured with equal-strain force profiles before adaptive acquisition.

Final reduced run (`/tmp/hp5-square-battery/hp5r-final.json`): **FAIL** for full
autocal. The 1000 s limit interrupted acquisition after 12 completed sweeps and
eight fits, without two consecutive stable hits. The latest fits still shifted
by more than the 0.5 mm threshold. No return-to-origin failure occurred with the
final carrying-first return rule. Elapsed time was 999.68 s. The selected
best-so-far estimate had A/B/C/D/I Euclidean errors
`1.980, 2.436, 1.156, 1.078, 0.961` mm; these are fitted, not applied parameters.
Interruption while collection was active also produced `HTTP Error 409: Conflict`
on attempted parameter application. Manifest: `status=interrupted`,
`normal_completion=false`, `stop_reason=Ctrl-C`, no applied parameters.
Stable-stop and interrupt-time application on the reduced machine remain open.

Exact bounded reproduction of the remaining failure (use a fresh dataset path):

```bash
mkdir -p /tmp/hp5-open-repro
HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=6000000 timeout -s INT 1000 .venv/bin/python autocal/autocal.py --machine-type hangprinter_5_pulley_reduced --dataset /tmp/hp5-open-repro/hp5r.json --headless-sim --recording-mode none --patience 20 --verbose > /tmp/hp5-open-repro/hp5r.log 2>&1
```

The same command with `hangprinter_5` and a separate fresh dataset reproduces
the passing direct-machine flow. Autocal uses the existing RRF headless backend;
headless Klipper autocal remains unsupported as before. No further tuning was
attempted after this bounded run. No files under `research/output/` were changed.
