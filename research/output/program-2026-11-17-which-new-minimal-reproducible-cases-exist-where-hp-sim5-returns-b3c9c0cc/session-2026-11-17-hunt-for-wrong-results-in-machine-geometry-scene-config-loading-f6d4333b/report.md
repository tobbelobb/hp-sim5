# Wrong results in machine geometry, scene/config loading and unit handling

**Question.** Find minimal, reproducible cases where `hp-sim5` (revision `c2450975`,
2026-11-17) returns genuinely wrong machine geometry, loads scene/config data with
the wrong meaning, or applies the wrong unit — concentrating on USD scene loading,
HP5 geometry, the catalog presets and the forward transform.

**Method.** I built small scripts that construct the same objects the simulator
uses (`OpenText` → `bakeCableSceneUsdaSource` → `readMachineSceneSpec` →
`buildEntityPlan` → `World`), or call the production forward driver in
`forward-transform-dev/hangprinter-forward-transform/build/`. Raw USDA transforms
are compared with `pxr`'s composed local matrices, which are the USD reference.
Repro scripts exit nonzero when the bug is present. `./run_all.sh` runs every
finding and control; logs are in `logs/`. No source file was modified.

The two axes of evidence are deliberately separated: **simulation evidence** is
what the JavaScript world and the C++ driver compute; **USD evidence** is what the
authored `.usda` file means to a USD-compliant consumer. There is no hardware
evidence in this report.

| # | Finding | Error size | Status |
|---|---|---|---|
| 1 | `xformOpOrder` ignored for translation (HP4 family, HP5-reduced wheels) | 0.20–3.09 m | genuine; shipped scenes affected |
| 2 | Missing `physics:angularVelocity` deletes a motor, leaves dangling joints | 2.54 m + first-step crash | genuine; in-domain optional attribute |
| 3 | `metersPerUnit` ignored | 1000× (2.23 m anchor error) | genuine; latent for shipped metre scenes |
| 4 | Forward dataset noise/bias unit conversion | 25.5× (A–D) / 51× (I) | genuine; benchmark-harness claim |

---

## Finding 1 — USD `xformOpOrder` is ignored for translation: HP4, HP4-worn-bearings and HP5-pulley-reduced wheels are 0.20–3.09 m from where USD puts them

**Repro.** `repro_xform_order_minimal.mjs` is the one-prim mechanism;
`repro_xform_order.py` measures the shipped scenes; `audit_transforms.py`
audits all tagged bodies, including orientation.

```bash
cd <session-dir>
node repro_xform_order_minimal.mjs         # exits 1; 1414.2 mm
.venv/bin/python repro_xform_order.py      # exits 1; 80 wheel prims
.venv/bin/python audit_transforms.py       # exits 1; 81 bodies
```

The minimal case uses a single Wheel with `translate = (1,0,0)`,
`rotateXYZ = (0,90,0)` and order `["xformOp:rotateXYZ","xformOp:translate"]`.
USD puts it at `(0,0,-1)`; the loader puts it at `(1,0,0)` — 1414.2 mm.

**Input.** The shipped `public/usd_scenes/hp4.usda`,
`public/usd_scenes/hp4_worn_bearings.usda` and
`public/usd_scenes/hp5_pulley_reduced.usda`. Every wheel is authored as

```usda
double3 xformOp:rotateXYZ = (0, 90, 0)
double3 xformOp:translate = (-0.34641, -0.22, 0.09275)
uniform token[] xformOpOrder = ["xformOp:rotateXYZ", "xformOp:translate"]
```

**Expected (USD).** The listed order composes as `R·T`, so the local position is
`R * translate` (for `WheelAL_top`: `(0.09275, -0.22, 0.34641)`).

**Actual (simulator).** `machineEntityBuilders.js` and `cable_scene_baker.js`
read `xformOp:translate` as the position and apply `rotateXYZ` only to the
orientation. `WheelAL_top` is placed at `(-0.34641, -0.22, 0.09275)`.

| Scene | Wheels | Min error | Median | Mean | Max |
|---|---:|---:|---:|---:|---:|
| `hp4.usda` | 24 | 329.2 mm | 661.1 mm | 1144.7 mm | **3090.2 mm** (`WheelD1_left_top`) |
| `hp4_worn_bearings.usda` | 24 | 329.2 mm | 661.1 mm | 1144.7 mm | **3090.2 mm** (`WheelD1_left_top`) |
| `hp5_pulley_reduced.usda` | 32 | 203.9 mm | 635.5 mm | 1091.0 mm | **3073.4 mm** (`WheelI2_left_top`) |

The audit checks 312 tagged bodies in 12 scenes; **81 disagree** (the 80 wheels
above plus one 20° orientation case, below). Position tolerance is 1 mm.

The error is not cosmetic: a cable span that should be a short pinhole-to-bearing
gap becomes a long diagonal. For `PinholeAL_top → WheelAL_top` in HP5-reduced,
the simulator gives **21.27 mm** and USD gives **372.71 mm** (351.4 mm error). The
mean error over each wheel's nearest pinhole is 258.8 mm, i.e. the bearing stack
is detached from its corner by roughly the size of the corner assembly itself.

**Secondary case (orientation).** `cubecorners.usda` `SpoolA` authors
`xformOp:rotateXYZ = (0,0,20)` but lists `xformOpOrder = ["xformOp:translate",
"xformOp:orient"]`. USD ignores the unlisted rotation; the loader applies it,
a 20° orientation difference. It is physically harmless here because the rotation
is about the spool's Z axis, but it shows the reader is not honoring the order
list.

**Why this is a bug, not a simplification.** The file explicitly declares the
order, and the numeric comments in the same file (`# 0.1-0.0065-0.00075` for
z = 0.09275) plus the pinhole at y = -0.24 show that the *intended* positions are
the raw translations. Under USD, the wheel stack no longer touches the corner
clamp, so a USD-native viewer/importer renders a broken machine. The generator
`scripts/generate_hp5_scenes.py` copies the same order, so the error propagates
to regenerated scenes. `public/usd_scenes/hp4_worn_bearings.usda` shares the HP4
layout and has the same 24-prim defect. The minimal asset fix is to swap the order to
`["xformOp:translate", "xformOp:rotateXYZ"]` on the 80 wheel prims; the simulator
would keep its current, intended geometry.

**Strongest alternative explanation.** "The asset is wrong, not the simulator."
I agree that the simulator's self-consistent geometry is the design intent. The
finding is the undetected disagreement: the scene is not valid USD, and the loader
silently substitutes a private interpretation for every USD consumer. If instead
the USD reading is authoritative, the simulator's cable lengths are wrong by
0.20–3.09 m.

---

## Finding 2 — A missing non-geometric spool attribute silently deletes a motor and leaves dangling cable joints

**Repro.** `repro_missing_attr.mjs`.

```bash
cd <session-dir>
node repro_missing_attr.mjs      # exits 1
```

**Input.** `public/usd_scenes/hp5.usda`, with only the line
`float3 physics:angularVelocity = (0, 0, 0)` deleted from the `SpoolC` block
(done in memory; the file is unchanged).

**Expected.** A zero initial angular velocity is the natural default, so the
scene should load with all five motors; alternatively the loader should reject
the scene at load time and name `SpoolC`.

**Actual.** `applySpool` treats `physics:angularVelocity` as mandatory and prints
one `console.warn("Skipping Spool prim SpoolC ...")`. `SpoolC` has no entity.
The two joints that start at that spool survive because `cableJointBuilder.js`
tests `entityA === null`, while `Map.get` returns `undefined`. The loaded world
contains:

```
SpoolC entity: undefined
CablePathCL joints=2 linkTypes=3 stored=[2,0,0] totalRestLength=4.348061
orphan JointCL_0: entityA=undefined attachmentA=(-0.0350, -0.0313, 0.0000)  distance_from_spoolC_centre=2.5380 m
orphan JointCR_0: entityA=undefined attachmentA=(-0.0410,  0.0230, 0.0000)  distance_from_spoolC_centre=2.4839 m
first physics step error: Missing cable endpoint for joint 29
```

So one of five motors disappears, two of 24 cable joints dangle, and the phantom
cable end is **2.538 m** from the spool centre (**2.506 m** from the intended
surface attachment; tolerance 1 mm). Loading itself
succeeds; the failure arrives later as an unrelated `Missing cable endpoint`
exception on the first `world.update(dt)`. Both the WASM backend and the default
headless backend throw the same way.

**Why this is a bug, not a simplification.** Angular velocity is not a geometric
property, and zero is the natural default. If the attribute really is required,
the correct behavior is a load-time validation error, not a warning plus a
half-built world. The silent propagation through `entityA === null` is clearly
accidental: `undefined` is the value the registry actually returns for a missing
body. The same file's `applyPinhole` already treats a missing
`physics:angularVelocity` as optional, so the spool requirement is inconsistent.

**Suggested fix.** Default a missing `physics:angularVelocity` to zero (or reject
at load), and treat `undefined` as well as `null` when resolving `entityA`/`entityB`.

**Strongest alternative explanation.** "The scene schema requires
`physics:angularVelocity`." No schema, test or documentation enforces that; every
USD attribute is optional by default, and the loader's own warning says
"missing attributes", not "invalid scene". Even under that reading, the bug is
the lack of a hard load-time rejection.

---

## Finding 3 — `metersPerUnit` is ignored: a valid millimetre-authored scene loads 1000× too large

**Repro.** `repro_meters_per_unit.mjs`.

```bash
cd <session-dir>
node repro_meters_per_unit.mjs   # exits 1
```

**Input.** Two USDA scenes that describe the same physical machine:
`tests/fixtures/engine_parity/usd_scene_minimal.usda` in metres
(`metersPerUnit = 1`), and the same scene with every linear length (translations,
radii and joint distance limits) multiplied by 1000 and `metersPerUnit = 0.001`.
Under USD both have the same world geometry.

**Expected.** Anchor at `(2.0, 1.0, 0.0) m`; wheel at `(0.5, 0.0, 0.2) m`; spool
radius 0.05 m; body radius 0.1 m.

**Actual.** The loader never reads `metersPerUnit` (`grep -rn metersPerUnit src
hp-sim-3d` finds no consumer). It loads the millimetre scene verbatim:

| Prim | Metre scene | Millimetre scene | Ratio |
|---|---:|---:|---:|
| Anchor | (2, 1, 0) | (2000, 1000, 0) | 1000.0 |
| Wheel | (0.5, 0, 0.2) | (500, 0, 200) | 1000.0 |
| SpoolA | (-1, 0, 0) | (-1000, 0, 0) | 1000.0 |
| Corner | (1, -1, 0) | (1000, -1000, 0) | 1000.0 |

The anchor x error is **1,998,000 mm** (Euclidean position error **2,233,831.9 mm**;
tolerance 1 mm). The world builds without warning; a subsequent physics run would simply be a 1000× larger machine.

**Why this is a bug, not a simplification.** `metersPerUnit` is part of the USD
scene contract, not decorative metadata: a consumer converts stage units to
metres by multiplying by it. The app accepts custom USDA files, so a user can hit
this with a perfectly valid millimetre scene. The repo documents simulator
geometry in metres (`research/More_on_hp-sim5_MCP.md`), so a deliberate
metres-only contract is plausible — but then the loader must reject
`metersPerUnit != 1`; it does neither.

**Suggested fix.** Either scale every linear length by `metersPerUnit` or reject
`metersPerUnit != 1` before building entities.

**Strongest alternative explanation.** "The simulator defines its own
metres-only scene format." That is only a defensible position if it is enforced
or documented; currently the value is silently ignored, and every position,
radius and distance is 1000× wrong.

**Scope caveat.** All 11 shipped catalog scenes declare `metersPerUnit = 1`, so
no shipped catalog result is affected. This is a latent unit bug for imported,
experimental or user-authored scenes.

---

## Finding 4 — Forward-transform noise/bias labels are 25.5× (A–D) and 51× (I) off

**Repro.** `repro_noise_units.py`.

```bash
cd <session-dir>
.venv/bin/python repro_noise_units.py   # exits 1
```

**Input.** `forward-transform-dev/hangprinter-forward-transform/run_tests.py` and
the shipped datasets, whose `config_used` is e.g. `spool_r_in_origin = 75 mm`,
`spool_gear_teeth/motor_gear_teeth = 255/20 = 12.75`, and
`mechanical_advantage = [2,2,2,2,4]`.

**Expected.** At the origin, one millimetre of line is
`gearing * ma * 360/(2πr)` motor degrees: **19.480565 deg/mm** for A–D and
**38.961130 deg/mm** for I.

**Actual.** `MM_TO_DEG = 360/(2π·75) = 0.763944 deg/mm` (a 1:1, zero-mechanical-
advantage spool). The harness therefore injects:

| Label | Injected | Actual line-equivalent (A–D) |
|---|---:|---:|
| noise σ = 1 mm | 0.764° | **0.0392 mm** |
| noise σ = 3 mm | 2.292° | **0.1176 mm** |
| noise σ = 5 mm | 3.820° | **0.1961 mm** |
| bias ±5 mm | ±3.820° | **±0.1961 mm** |
| bias ±2.5 mm | ±1.910° | **±0.0980 mm** |

Calling the harness functions on 200 real HP5 samples confirms it: `add_gaussian_noise(…, 5.0)`
produces a 3.969° motor-degree sigma, i.e. **0.204 mm** of line, and
`add_bias([…5, -5, 2.5, -2.5, 0…])` produces offsets `[3.820, -3.820, 1.910,
-1.910, 0]°` = `[0.196, -0.196, 0.098, -0.098, 0] mm`.

The same constant and helpers are duplicated in
`klipper/ai_docs/run_tests.py`.

**Why this is a bug, not an approximation.** The dataset config carries the
gearing and mechanical advantage needed for an exact conversion; ignoring both
changes the amplitude by 25.5× (A–D) and 51× (I). The printed sweep names claim
σ = 1/3/5 mm and bias ±5 mm, so the reported robustness numbers describe a much
easier problem than advertised. The independent regression corpus in
`test_forward.py` injects noise directly into line lengths and is **not** affected
by this constant.

**Suggested fix.** Convert per axis with `gearing * ma * 360/(2*pi*r)` (and use
the same per-axis factor for bias).

**Strongest alternative explanation.** "The labels mean millimetres at the motor
shaft." Even then the conversion from motor degrees to shaft millimetres must use
the gearing; 0.764 deg/mm is not a consistent physical unit for this drivetrain.
The sweep names do not say "shaft millimetres".

---

## Negative results (where I looked and found no bug)

1. **Inverse/forward round trip.** `probe_forward_roundtrip.py` fed 1200
   independently computed line-length vectors to
   `build/test_forward_driver` for HP5 (LastOnTop), HP3 (LastOnTop) and
   CubeCorners (CubeCorner). 849 converged; the script compiles the firmware's
   `hp_reachable()` and verified that all 351 non-converged probes are outside the
   documented pyramid/prism (0 reachable false negatives). Among converged probes:
   **max error 3.398e-4 mm**,
   median 1.29e-4 mm, against the 0.02 mm clean-case tolerance. `test_forward.py`
   passes all 5 tests (worst clean-case workspace error 0.0132 mm; 400
   motor→Cartesian round-trips max 0.0049 mm). No forward-transform defect found.
2. **Catalog presets vs scenes vs firmware.** `check_catalog_consistency.py`
   checks 11 scenes (the non-layered preset for each scene; layered presets
   intentionally use different R/Q), 72 frame-anchor offsets (anchor minus the
   attachment it feeds), 46 spool radii, and every Klipper `anchor_x/y/z` and
   `rotation_distance`. Everything agrees: anchor error < 0.02 mm, radius error
   < 0.01 mm, and Klipper `rotation_distance = 2πR/U` as expected. Mounted-motor
   slideprinter scenes were excluded from the anchor-offset check because their
   `M669` vector is the idealised mover-origin-to-anchor vector, not
   anchor-minus-spool.
3. **All catalogued scenes load complete cable paths.** `probe_scenes.mjs` finds
   authored and world cable-path counts identical for all 11 scenes
   (9, 9, 9, 8, 8, 6, 3, 6, 3, 12, 12), no duplicate sibling prim names, and no
   path with a joint-count mismatch.
4. **Boundary spool radii.** `radius = -0.03` throws a clear
   `Cannot derive rolling tangent without radius` error; `radius = 0` also throws
   when layering is disabled, which is the path the app forces for HP5. With
   layering enabled, zero radius is silently replaced by a layering-derived
   effective radius, but that is not the shipped HP5 load path. Negative/zero
   radius is therefore **not** a silent-wrong-geometry case for the default HP5
   machine.
5. **Forward workspace modes.** LastOnTop accepts the lower prism and the pyramid
   below the top anchor and rejects points above/outside it; AllOnTop has no
   height limit; CubeCorner encloses the cube and projects its lower face down.
   This matches the documented `forward_workspace.h` contract and the existing
   tests.

---

## Additional observations (not counted as simulator wrong results)

- **Slideprinter up-axis.** Four slideprinter scenes declare `upAxis = "Y"` but
  are authored in the XY plane with `physics:gravityDirection = (0,0,-1)`. The
  loader ignores `upAxis`, so simulation is unaffected, but a USD viewer would
  orient them 90° differently.
- **Dead `defaultPrim` fast path.** `machineSceneController.findScenePrimPath`
  reads `stage.ast.descriptor.defaultPrim`, but the parser stores it in
  `descriptor.assignments`. The traversal fallback still finds the correct root
  for all flat shipped scenes; a nested scene could resolve to a sub-group.
- **Unhonored transform ops.** `!invert!` is stripped rather than conjugated, and
  `xformOp:scale` is never applied. No shipped scene uses either. Latent.
- **Unsettled: Slideprinter Multi Unit.** The scene has six cable paths (two lines
  per motor) attached to three corners, while the paired
  `config_slideprinter.g` describes three lines and has no `M666 N`/`O`. The
  anchor offsets match exactly, so I did not count it as a catalog mismatch;
  whether the firmware model is intended to approximate the paired lines is
  undocumented. I did not run a motion comparison to settle it.

---

## Limitations

- The transform comparison uses `pxr`'s composed local matrices, not the three.js
  renderer. The local USD build lacks the `UsdGeom.Circle` schema, so
  `ComputeLocalToWorldTransform` returns identity for the `Circle` prims; the
  audit composes each prim's `GetLocalTransformation` down the parent chain,
  which is the same xform-stack computation a full USD build performs. The renderer follows the simulator's entity positions, so Finding 1 is
  a **USD-vs-simulator** disagreement, not a renderer-vs-simulator one.
- The millimetre scene in Finding 3 is synthetic (valid USD, not a shipped asset).
- Finding 1 is an asset/loader contract disagreement. Fixing the assets preserves
  the simulator's current, intended geometry; fixing only the loader would move
  HP4/HP5-reduced wheels in the simulation. I recommend fixing the assets and the
  generator, then adding a loader check that reports non-identity/unsupported
  transform stacks.
- Finding 2 was reproduced by patching a scene in memory; no source file was
  modified. It fails identically with the WASM and default headless backends.
- Finding 4 affects the dataset sweep harness and its Klipper copy; it does not
  affect the independent regression corpus in `test_forward.py`.
- No hardware evidence was collected. Only targeted tests were run (the two
  Python geometry/catalog test files and `test_forward.py`); the full Jest and
  Python suites were not run.
- The audit did not find any affected tagged body in hp3, hp5, cubecorners,
  skycam or the slideprinter scenes; Finding 1's positional defect is limited to
  the HP4-family and HP5-pulley-reduced wheels.

---

## Reproduce

The session directory is
`research/output/program-2026-11-17-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-b3c9c0cc/session-2026-11-17-hunt-for-wrong-results-in-machine-geometry-scene-config-loading-f6d4333b`.
Every command above is run from that directory. Repro scripts exit **1 when the
bug is present**; controls exit **0 when the expected behaviour holds**.

`./run_all.sh` runs all findings and controls and writes `logs/*.log`:

| Script | Expected exit | Purpose |
|---|---|---|
| `repro_xform_order_minimal.mjs` | 1 | Finding 1: one-prim mechanism (1414.2 mm) |
| `repro_xform_order.py` | 1 | Finding 1: USD vs simulator wheel positions |
| `audit_transforms.py` | 1 | Finding 1: all tagged bodies, position + orientation |
| `repro_missing_attr.mjs` | 1 | Finding 2: missing spool attribute |
| `repro_meters_per_unit.mjs` | 1 | Finding 3: metres vs millimetres |
| `repro_noise_units.py` | 1 | Finding 4: noise/bias unit conversion |
| `probe_forward_roundtrip.py` | 0 | Control: forward round-trip accuracy |
| `check_catalog_consistency.py` | 0 | Control: catalog/scene/firmware agreement |
| `probe_scenes.mjs` | 0 | Control: all catalog scenes load complete paths |
| `test_forward.py` | 0 | Control: production forward regression suite |
| `pytest tests/python/test_hp5_geometry.py tests/python/test_machine_catalog.py` | 0 | Control: authored HP5 geometry and catalog |

`results.json` contains the same measurements in machine-readable form.
