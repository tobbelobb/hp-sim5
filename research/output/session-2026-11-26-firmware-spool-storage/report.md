# Firmware spool flattening and strained storage

2026-11-26, simulation only, PR #132. Firmware: RRF `d820c0d11`, Klipper `ea7cd51a`. Simulator: native Rust solver from main after PR #131, closed-loop motors, catalogue physics settings.

## Question

The simulator stores rope on elastic spools at its winding strain and contracts the running radius with tension (`spool_rho` in `src/wasm/cable_solver/src/engine.rs`, default radial compliance `1.50e-8 m²/N`). Do the firmwares print more accurately when their flex compensation models the same two effects?

## Firmware model

Both firmwares take per-line tension `t = F/(U·N)` from the flex force solver. They add `linePos·(strain + sink)/(1 − sink)` to the flex offset, where:

- `linePos = d − d_origin + flex` is the origin-relative flexed line position;
- `strain = t/EA`;
- `sink = min(K·t/R², .5)`.

The term sits inside `FlexDistances`/`compute_flex`, so the inverse transform, Klipper's pretension transition and the forward transform share it.

The model is stateless. It assumes the rope on the drum was wound at the tension at which it is paid out. The simulator instead keeps a LIFO winding history, so a path that winds at one tension and pays out at another leaves an unmodelled residual. Both options exclude spool buildup.

## Comparisons

The run is headless Print Squares: `tests/benchmark/logo_accuracy.mjs --gcode draw_squares_bigger`, with concentric squares out to ±600 mm. The 0–100 score comes from `hp-sim-3d/app/quality-monitor.js`. Shape RMS and P95 are path deviation after removing the mean offset. Each machine has three command streams:

- **old:** the stream committed before this work;
- **base:** current firmware with the two options removed;
- **new:** current firmware with `M666 K0.015 V1` / `winch_spool_flattening: 0.015`, `winch_strained_spool_storage: true`.

Full table: [print_squares_scores.tsv](evidence/print_squares_scores.tsv), with columns score, shape RMS mm, shape P95 mm, contour max mm and missed steps. Reproduce it with [verify.sh](evidence/verify.sh).

| machine | RRF old / base / new | Klipper old / base / new | RRF shape RMS old → new (mm) |
|---|---|---|---|
| HP3 | 79.1 / 79.1 / 85.5 | 77.1 / 77.1 / 83.0 | 0.216 → 0.134 |
| HP4 | 85.9 / 85.9 / 95.2 | 86.0 / 86.0 / 94.7 | 0.129 → 0.042 |
| HP5 | 89.4 / 89.4 / 93.4 | 89.0 / 89.0 / 93.2 | 0.091 → 0.049 |
| HP5r | 90.6 / 90.6 / 96.2 | 90.7 / 90.7 / 96.3 | 0.090 → 0.028 |

No run missed a step.

### HP4 RRF ablations

- Strained storage alone scored 93.5; flattening alone scored 88.5.
- Flattening constants of 0.0075, 0.015 and 0.03 mm²/N scored 94.5, 95.2 and 93.8. The simulator's own constant scored best.
- Applying the correction only to rope wound in past home scored 90.5. Rope paid out past home and later rewound is stored strained, so a model that treats it as unstrained loses accuracy.

### COPacabana (exploratory)

On COPacabana, RRF squares with `M666 K0.015 V1` added scored 67.1 against 45.2 without it. Shape RMS fell from 0.685 to 0.213 mm. The COPacabana configuration is unchanged in this PR.

## Alternative explanations and limits

- **Model shared by construction.** The firmware model shares its constants with the simulator: EA from `M666 S`, and K equal to the solver default. The gain therefore measures model consistency, not hardware accuracy. Real line needs its own `K` and EA.
- **Nondeterministic RRF host.** The RRF host simulator is nondeterministic run to run, and its stream length grows when the host is loaded. Same-input score differences stayed within 0.02 points, and the squares streams stayed within 0.03 % of their length.
- **Truncated Klipper streams before the fix.** Before the Klipper fix in this PR, batch-mode `.serial` output intermittently lost its last ~5.6 s. In the affected runs this showed up as a 2.7 mm contour spike when motion stopped while the extruder continued. All Klipper numbers above come from complete streams.
- **One print only.** Only one print (squares around the origin) was tested. Paths that wind at one tension and pay out at a very different one were not examined separately.

## What would change the conclusion

The following would show the stateless approximation is too coarse:

- a print with large tension differences between winding and payout of the same rope;
- a hardware measurement of `K` well away from 0.015 mm²/N.

Tracking winding history in firmware would then be the next step.
