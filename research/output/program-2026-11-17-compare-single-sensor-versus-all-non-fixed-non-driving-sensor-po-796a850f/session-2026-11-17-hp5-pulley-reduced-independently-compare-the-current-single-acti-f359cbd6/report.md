# HP5 pulley-reduced: single active sensor vs both free lines as torque-controlled sensors

Investigation: `session-2026-11-17-hp5-pulley-reduced-independently-compare-the-current-single-acti-f359cbd6`
Program: `program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f`
Revision: `c2450975` (worktree `t3-ededea32`), simulation only.

## Independent review corrections

The [independent saved-data review](../session-2026-11-17-independently-review-reduced-hp5-matched-short-collection-eviden-748ceb19/report.md) supersedes the numerical claims below where they differ. The control loader incorrectly classified `prototype-control` as a sensor arm: its corrected four-range error is 9.87 mm, and the 1 N control is 48.8 / 21.2 mm for four/five ranges. The matched treatment remains 22.13 / 1.85 mm. Its extra-range error at nominal anchors is 0.015–0.019 mm RMS; the originally reported sub-millimetre bias came from weakly inferred poses. Same-recording local sigma improves 15.56 → 2.30 mm (6.8×). Held-out folds shared learned offsets, so they do not constitute fully independent validation. Encoder holding error does not bound frame elasticity, and no cogging ablation established causality. These are simulation findings, with no hardware or completed full-loop comparison.

## Question

On HP5 pulley-reduced, production collects `d+1 = 4` active lines: two fixed
(one low anchor + the top `I` anchor), one drive, one torque-controlled sensor.
The fifth low line is held at the compliant `forceLow` and its encoder is recorded
but discarded. Does promoting that redundant line to a measured, torque-controlled
range help, and what does pretension cost?

## What was run

Four short isolated headless collections (48 points each), matched in sweep
configs, pose targets, solver settings and fitter budget:

```
configs: [0,4]:drive 1/sensor 3   [1,4]:0/2   [2,4]:3/1   [3,4]:0/2
targets: -320,0 ; -180,-320 ; -320,-160 ; -180,-320     (per sweep, mm)
6 points per direction (2 directions per sweep), solver iterations 20, speedup 1
fixed sensor force 4.9914 N, force_mid 1.0318 N, force_max 2.8313 N
```

Every low anchor is the redundant line exactly once (2, 3, 0, 1). Two repeats
per arm were collected; the headless world is deterministic, and the repeats
produced identical simulated durations and analysis metrics, so they confirm
determinism rather than hardware repeatability.

| Arm | Collector | Redundant line torque |
|---|---|---|
| A `ctl-1/2` | control | `forceLow = 0.0516 N` (current policy) |
| B `twosensor-1/2` | prototype | 4.9914 N during preload + measurement only; `forceLow` during pullout |
| C `ctl-low1p0` | control | `forceLow = 1.0 N` for all idle lines |

All arms use experiment copies of the collector; the control variant keeps the
production single-sensor policy and only adds per-sweep fixed targets. The
prototype additionally measures the redundant line
([build_prototype.py](evidence/build_prototype.py), sources under
`evidence/prototype/` and `evidence/prototype-control/`); production files were
not edited. Per-sweep fixed-target groups are needed because one global target
keeps the top-anchor length constant and leaves the top-anchor height
unidentifiable — production's own rough sweeps vary it for the same reason.

## Result 1 — at production force the redundant axis is not a sensor

Encoder travel of the redundant line versus the geometric distance change that
the pose implies (pose trilaterated from the two fixed lines and the drive, using
the nominal `M669` anchors as a diagnostic only):

| Arm | Tracking slope, 8 sub-sweeps | Measured range | Geometric range | Encoder − geometry |
|---|---|---|---|---|
| A | `0.000` (every sub-sweep) | `0.0 mm` | `79–140 mm` | `+38.4 … +46.7 mm` |
| B | `0.995–1.000` | `78.6–139.3 mm` | `78.6–139.7 mm` | `−0.81 … −0.35 mm` |
| C | `0.975–1.004` | `3.0–8.0 mm` | `3.0–8.0 mm` | `−0.17 … −0.01 mm` |

At 0.0516 N the redundant spool is stalled: it holds 38–47 mm of extra paid-out
line (slack) and does not rotate as the mover moves. This is consistent with the
known motor-cogging finding: the cogging band of these 50-pole motors is
0.333/0.667 N, an order of magnitude above the 0.05 N command. A preliminary
single-config force sweep agrees: frozen at 0.05 and 0.5 N, taut but with a
collapsed accepted interval at 1.0 N, and a hard acquisition failure at 2.0 N
(`Measured sweep is too short to wind the sensor in half a spindle turn`).

A naive 5-line fit on arm A cannot reconcile the frozen range: the consistency
residual at the true anchors rises from 0.069 mm (4 lines) to 73.3 mm (5 lines),
and the robust fit is dragged 3.1 m away from the anchors. **A slack/stall
validity gate is a prerequisite for using this range.**

## Result 2 — measurement-only pretension makes it valid at unchanged coverage

Arm B preloads and measures both free lines at the normal sensor force, but
leaves `forceLow` untouched during pullout and idle. Then:

* redundant-line tracking is 0.995–1.000, with a constant −0.35…−0.86 mm offset
  per sub-sweep (home/routing offset plus ~0.4 mm friction hysteresis between
  directions);
* drive coverage is unchanged: 874/586/638/586 mm in both A and B (largest
  difference 0.4 mm), i.e. pretensioning only during measurement does not shrink
  the accepted sweep interval;
* fixed-anchor loaded error stays ≤0.0015 mm, so the extra 5 N does not slip or
  measurably move the position-controlled anchors;
* cost is +6.7% simulation time (1018 s vs 954 s per 48-point collection, both
  repeats identical). Wall time is not a usable discriminator here: the same
  simulation took 116–220 s depending on host load.

## Result 3 — the extra range is worth roughly 9× in anchor uncertainty

Same data, same starts, same optimizer, same home-offset prior; the only change
is whether the redundant line's length constrains geometry. Truth is the nominal
firmware `M669`, compared after translation alignment (the fit's own gauge is
canonical).

| Arm | 4-line max aligned anchor error | 5-line | max anchor σ (finite-difference Jacobian) | held-out point residual (median over 4 folds) |
|---|---|---|---|---|
| A current | **3.8 mm** | invalid (73 mm truth residual) | 16.4 mm | 0.03–2.13 mm |
| B two-sensor | 22.1 mm | **1.85 mm** | 15.6 → **2.3 mm** | 0.025–0.037 mm |
| C 1 N idle | 60.8 mm | 14.3 mm | 711 → 10.4 mm | 0.16–0.76 mm |

Perturbing every measured length by σ = 0.02 mm and refitting (6 seeds) shows
the same separation and is not a single-basin artefact:

| Arm | 4-line | 5-line |
|---|---|---|
| A current | 3.5–15.6 mm (median 9.5) | — |
| B two-sensor | 12.5–40.1 mm (median 18.3) | **1.23–2.98 mm (median 1.97)** |

Six explicit starts (truth, ±5%, top-anchor z ±250 mm, translated truth) give
1.85 mm for the 5-line fit on B in every case, versus 22.0–22.3 mm for the
4-line fit. At this 48-point budget the 4-line anchor estimate is limited by a
weakly determined direction; the second range removes that direction rather
than merely shifting the estimate.

Absolute errors of 2–22 mm are rough-bootstrap level, not calibration
certificates. The production fitter cross-check on arm A gives 11.1 mm versus
my 3.8 mm for the same data; both are stable basins at this budget, and the
matched comparison is the within-dataset one (22.1 → 1.85 mm).

## Result 4 — raising the idle/compliant force is not a substitute

Arm C shows what happens when the redundant line is pretensioned by raising
`forceLow` instead of only the measurement force: it becomes taut and accurate
per point (residual span 0.17 mm) but the accepted drive interval collapses about
4× (874 → 215 mm, 586 → 138 mm). The pullout/reserve acceptance now runs against
a line that is actively pulling, which is the likely cause (not isolated here). The 4-line fit degrades to 60.8 mm and is
start-sensitive (54.9–151.1 mm); the 5-line fit recovers to 14.3 mm but is still
7.7× worse than arm B. At 2 N the collector fails to acquire a sweep at all.

## Result 5 — integration hazard: the loader must know the force *and* use the range

If the two-sensor measurement is adopted but the production `d+1` loader is kept
(redundant line assumed at `forceLow`), arm B fits to 34.4 mm — worse than the
plain 4-line fit. The force error itself is small (a 5 N load-model mistake
changes stretch corrections by ~0.1 mm); the loss is the discarded geometry.
Conversely, using the extra range requires a second torque line in the static
solve (3 equilibrium + 2 known torques = 5 equations for 5 tensions) and the
per-point *physical* sensor (`source_sensor_anchor`), because reversed
sub-sweeps swap the drive/sensor roles while remapping `l_drive`/`l_sensor`.
Using the canonical sensor instead of `source_sensor_anchor` alone doubled the
truth residual (0.041 → 0.017 mm).

## Mechanism and strongest alternative explanation

The pattern is fully explained by cogging-dominated torque control: 0.05 N is
below the detent torque, so the spool never pays out and the encoder measures a
frozen spool angle, not a range. At 5 N the spool follows the line, and its
range is accurate to ~0.5 mm over 80–140 mm.

The main alternative — that the 5-line improvement is just "more observations"
regularising a weak solve rather than genuinely new information — is not
supported: the extra range has a physically predictable 79–140 mm span, its
residual against the pose is sub-millimetre, and both the Jacobian conditioning
and the held-out residual improve together. A second alternative, that the extra
5 N load biases the geometry through flex, is bounded: fixed-anchor loaded error
stays ≤0.0015 mm, and at matched commanded poses the sensor length changes by at
most ~0.3 mm, consistent with the modelled stretch.

## Limitations

* Simulation only (headless JS/Rust physics, stock pulley-reduced scene and
  `M666`), not hardware. Commanded versus actual tension still carries the known
  cogging error (0.333/0.667 N); at 5 N that is a 7–13% force error, at 0.05 N
  it dominates completely.
* Deterministic world: the two repeats are not independent noise draws.
  Robustness evidence comes from synthetic 0.02 mm length perturbations, which
  are a proxy for model/physics error, not a measured noise distribution.
* Nominal `M669` is treated as truth and truth is used only for diagnostics.
  Gauge/translation alignment follows prior work; no independent hardware
  reference.
* One scene, one pretension level (5 N), one sensor force, 4 sweeps × 48 points.
  The home-offset gauge places the two real offsets on lines 3 and 4, so a
  redundant line on another axis gets no offset of its own.
* No slack/validity detector is implemented or tested end-to-end. The frozen
  case shows that a fit alone will not reject a bad extra range.

## What would change the conclusion

* A measurement-only pretension arm at 1–2 N with the same protocol. If 1 N
  already gives sub-millimetre tracking and the same 9× fit gain, the 5 N load
  (and its cogging error) is unnecessary and the recommendation becomes cheaper
  and safer. This is the one small experiment I would add; it is not needed for
  the main conclusion.
* Hardware or a cogging-aware torque model: if the actual 5 N torque is much
  lower in some rotor phases, the range would intermittently stall and the gain
  would shrink. A slack/validity gate should be tested against such phases.
* A production-sized budget (more sweeps/poses): if the 4-line fit becomes
  well-conditioned, the relative value of the extra range should fall. The
  current result is specific to rough 4-sweep budgets.

## Recommendation

Do not use the compliant redundant axis as a range at `forceLow`; it is a slack
accumulator, and adding it blindly destroys the fit. If the extra range is
wanted, preload/measure it at the sensor force only, keep the pullout/idle force
unchanged, add a validity gate (e.g. require the redundant encoder's sweep
travel to match the pose-predicted travel within a few mm), and extend the
loader to the two-torque static model. Expect roughly a 5–9× reduction in anchor
uncertainty at rough-bootstrap budgets for about +7% simulation time and no
coverage loss. Raising the global compliant force is not an acceptable
substitute.

## Reproduce

```bash
# from the session directory; root interpreter and read-only shared artifacts
PY=/home/torbjorn/repos/hp-sim5/.venv/bin/python
$PY evidence/build_prototype.py
$PY evidence/collect_run.py --run-id twosensor-1 \
  --force-low 0.0516 --force-mid 1.0318 --force-max 2.8313 --sensor-force 4.9914 \
  --fixed-targets="-320,0;-180,-320;-320,-160;-180,-320" --points 6 \
  --configs 0,4:1:3 1,4:0:2 2,4:3:1 3,4:0:2 \
  --collector evidence/prototype/collect_sweep_data.mjs
$PY evidence/collect_run.py --run-id ctl-1 ... \
  --collector evidence/prototype-control/collect_sweep_data.mjs   # same args, control arm
$PY evidence/analyze.py --output analysis.json
$PY evidence/sensitivity.py runs/ctl-1 runs/twosensor-1 runs/ctl-low1p0 --output sensitivity.json
$PY evidence/noise_sensitivity.py runs/ctl-1 runs/twosensor-1 --output noise-sensitivity.json
```

Each `runs/<id>/` keeps `invocation.json`, `collector-command.json`,
`collector.log`, `run-summary.json`, the dataset and its `sweeps.headless/`
manifest, scene and firmware-config copies. See [manifest.json](manifest.json)
for code, scene and dataset hashes.

The worktree used the root checkout's read-only build artifacts through
temporary symlinks, which were removed after the runs:

```bash
ln -s /home/torbjorn/repos/hp-sim5/node_modules node_modules
mkdir -p RRF/build && ln -s /home/torbjorn/repos/hp-sim5/RRF/build/rrf_simulator RRF/build/rrf_simulator
ln -s /home/torbjorn/repos/hp-sim5/src/wasm/cable_solver/target src/wasm/cable_solver/target
```

## Artifacts

* [analysis.json](analysis.json) — per-run tracking, residuals, fits,
  conditioning, cross-validation, truth residuals.
* [sensitivity.json](sensitivity.json) — six-start fit spread.
* [noise-sensitivity.json](noise-sensitivity.json) — 0.02 mm perturbation study.
* `runs/_prelim/` — single-config force sweep (0.05 / 0.5 / 1.0 / 2.0 N) and
  constant-target 3–4-sweep runs that exposed the top-anchor degeneracy. Kept
  as negative/design evidence, not used in the main table.

## Isolation

All worlds were new headless worlds on free ports; no process, port or file of
`output/first-manual-autocal-of-hp5-pulley-reduced` was modified. Its
`sweeps.json` was copied read-only to `/tmp` for the fitter cross-check; that run
had already finished (11:35) before these experiments started. All processes and
listeners from this investigation were stopped; temporary links to the root
checkout's read-only build artifacts were removed. Nothing is committed.
