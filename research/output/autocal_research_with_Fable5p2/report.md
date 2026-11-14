# HP3 autocal: stretch bias and stretch-insensitive measurement design

Archived investigation, 2026-11-14. Simulation evidence only; filed from the owner's [conversation](evidence/conversation) and researcher's [summary](evidence/findings_summary.md). No new experiments were run during filing and no production implementation is claimed.

## Question and evidence scope

Why are HP3 autocal anchors about 7 mm off despite excellent fit scores, and can measurement design reduce that bias without knowing exact cable stiffness? The initial accepted browser result and the bootstrap headless comparisons are different datasets. The production-world design study uses direct motor commands and an independent least-squares fitter, bypassing firmware and production autocal filtering/selection.

The headless manifests preserve source revision `1172c62ac505ca0859db88571435f90d381c8d6e`, source/build hashes, scene, firmware, settings and timing. They stopped at the one-iteration budget (`max-steps`), rather than normal full-autocal acceptance. See [h1](evidence/headless_runs/h1/manifest.json), [h2](evidence/headless_runs/h2/manifest.json), and [h3](evidence/headless_runs/h3/manifest.json). The design study's exact source revision is not independently established by a corresponding manifest.

## Diagnosis and initial comparisons

The following findings and numbers are transcribed from the completed conversation:

Autocal is fitting a rigid-line model to data that contains line stretch, and the sweep geometry amplifies that small stretch into millimetres of anchor error. The optimiser and geometry model are not the problem.

I reproduced your `sweeps3` run headless with a Rerun recording. The first-iteration anchors match your browser run to within 0.2 mm (0.7 mm on D's Z).

## What the recording shows

- **The data disagrees with true geometry by 0.1–0.6 mm.** At each measurement point I compared the encoder length change with the true line length change computed from the recorded effector position. All four lines read short, and the amount follows the line tension (1–5.6 N during sweeps).
- **Stretch is the whole error.** Feeding the fit the true geometric lengths instead of the encoder lengths, with the same sweeps and points, gives 0.00 mm anchor error. With raw encoder lengths, my independent fit lands on the same anchors autocal does.
- **The fit cannot see it.** The stretch varies smoothly along a sweep, so shifted anchors absorb it almost perfectly. Residual RMS is 0.05–0.08 mm, which is why autocal reports "ideal quality" while the anchors are off.
- **The sweeps are ill-conditioned.** For the three bootstrap sweeps, the weakest parameter directions move 120–730 mm per mm of coherent length error. D's Z is the weakest, which is why it drifted from 1893 to 1879 in your run as sweeps were added (truth is 1900).

Your accepted result compared with the scene truth:

| Anchor | Found | True | Error |
|---|---|---|---|
| A (y, z) | −1907.51, −275.89 | −1900, −280 | −7.5, +4.1 |
| B (x, y) | 1651.62, 953.54 | 1645.45, 950 | +6.2, +3.5 |
| C (x, y) | −1651.94, 953.79 | −1645.45, 950 | −6.5, +3.8 |
| D (z) | 1880.34 | 1900 | −19.7 |

## How much has to be removed

Scaling the stretch correction on the bootstrap data shows how accurate a compensation would need to be:

| Stretch removed | Worst anchor coordinate error |
|---|---|
| 0 % | 8.4 mm |
| 50 % | 4.2 mm |
| 80 % | 1.7 mm |
| 90 % | 0.8 mm |
| 100 % | 0.0 mm |

So getting below 1 mm needs the stretch known to about 10 %, or sweeps that are far less sensitive to it.

## Two things I tried that do not fix it

- **Converged cable solver (`--solver-iterations 20`):** at the default single iteration, the A/B/C lines behave about twice as soft as configured (8.5–12.7 kN effective against 20 kN). With 20 iterations they match the configured value and the XY error halves to about 4 mm, but D's Z got worse (−26 mm). So roughly half the stretch in your run is a solver artefact and half is real line elasticity.
- **`--project-zero-tension`:** worse. Residual rose to 0.24 mm and D's Z went to +52 mm.

Both are single one-iteration runs (bootstrap sweeps only), not full autocal runs.

## What would fix it

The fit currently treats flex only as a noise term (0.237 mm sigma) and never corrects the lengths. The direct fix is to model stretch inside the fit: compute the line tensions from statics at each point (sensor force and mover weight are known) and add the stretch from `M666 S` and the guy-wire lengths before comparing to geometry. Sweeps that constrain D's Z better would reduce the amplification, but I have not tested either change.

One caveat for a statics-based correction: the realised sensor tension in the recording is up to 0.35 N above the commanded 2.23 N, so the commanded force alone is slightly off.

Nothing in the repo was changed. The three recordings and the analysis scripts are in `/tmp/claude-1000/-home-torbjorn-repos-hp-sim5/6be52c02-0bfa-4f95-a23e-a262242026a8/scratchpad` (`h1` default, `h2` 20 iterations, `h3` zero-tension), which is temporary; say if you want them moved somewhere durable or written up as a research report.


## Measurement design and production-world comparison

“Real simulator” below means the production physics world, not hardware. The first table row describes the earlier autocal dataset; it is contextual rather than a paired direct-collector baseline.

Yes, it is possible: in the real simulator a stretch-insensitive measurement scheme got the anchors to 0.1–0.6 mm without using the line stiffness anywhere. No choice of poses alone does it, though; it took three things together, and choosing poses higher up is only one of them.

## Results in the real simulator

All runs use 48–55 static poses on the HP3 scene, direct motor commands (no firmware), and my own least-squares fit rather than autocal's fitter. "Shape error" is the worst anchor coordinate after removing a common shift of all four anchors (explained below).

| Poses | Sensor force | Fit | Solver iterations | Shape error |
| --- | --- | --- | --- | --- |
| Your run (all on one sphere around D) | constant 2.23 N | rigid | 1 | 7–20 mm |
| Spread over height, z 0–1200 mm | constant 2.23 N | rigid | 20 | 8.2 mm |
| Same | constant 2.23 N | + offset | 20 | 2.6 mm |
| Same | equal-strain, 1.5 N per strand | + offset | 20 | 0.62 mm |
| Same | equal-strain, 1.0 N per strand | + offset | 20 | 0.12 mm |
| Same | equal-strain, 2.5 N per strand | + offset | 20 | 0.52 mm |
| Same | equal-strain, 1.5 N per strand | + offset | 1 (soft, uneven) | 1.20 mm |
| Same | constant 2.23 N | + offset | 1 | 4.2 mm |

Each row is a single deterministic run, so read the three equal-strain rows as a 0.1–0.6 mm range rather than as a trend.

## The theory

- **Stretch never averages out.** At any pose the line tensions have exactly one free degree of freedom (the pretension), and it stretches the lines in precisely the combination the fit measures. Every point is therefore biased in the same direction.
- **A uniform strain is harmless.** If every line were strained equally at every pose, the anchors would only scale by about 1e-4, which is 0.2 mm.
- **The harm comes from two things:**
  - The encoder zero is taken with A, B and C slack, while every measurement is under tension. That acts as a fixed length offset.
  - Tension varies between poses and between lines.
- **Scale is inherently weak.** Scale information only enters at second order in displacement over line length, so a 0.3 mm coherent error becomes about 6 mm at the 600 mm sweeps used today.

## The three ingredients

1. **Poses spread over height.** In your run D was always fixed at zero length change, so every pose lay on one sphere around the D anchor; that is why D's Z was so poorly determined. Shortening D to measure at several heights cuts the worst noise amplification from about 1300 to about 14 in my model, and makes stretch show up in the residuals instead of being absorbed.
2. **One extra fit parameter.** A single length offset on one line absorbs the slack-reference mismatch. Only one is needed, because the other three combinations are equivalent to moving the home position.
3. **Equal-strain sensor force.** Choose the torque-mode force at each pose so that the weighted mean tension per strand is the same everywhere. The stretch then looks like a uniform scale plus the offset. The rule uses the rough anchor estimate, the mover weight and the strands per anchor, but never the stiffness.

In my elastic model the scheme gives 0.14–0.34 mm for line stiffness from 20 kN down to 8.5 kN. It stays at or below about 1 mm when one line is 30–40 % softer than the others, when the planning anchors are 100 mm off, or when the mover mass is 25 % wrong.

## What you should know before building on this

- **Home shift:** the fitted anchors are relative to where the effector sits with tensioned lines at encoder zero, which is 0.8–2.2 mm lower in Z than the slack-line home. The lower lines are nearly horizontal, so 0.3 mm of stretch moves the effector about 2 mm vertically. All anchors shift together, so the machine geometry is unaffected; raw errors including this shift were 0.8–2.3 mm.
- **Current floor:** the torque-mode motor realises a tension up to ±0.36 N off the commanded value, regardless of force level. That is what separates the simulator's 0.6 mm from the model's 0.14 mm.
- **Solver iterations:** with one iteration the scheme still gives 1.2 mm, so it tolerates the soft, uneven lines, but 20 iterations in autocal runs is what gets it under 1 mm.
- **Not yet tested:** encoder noise, autocal's own fitter and filtering, the firmware path, and whether the active learner would pick such poses itself. I did not look at `--project-zero-tension`.


## Interpretation, limitations and next steps

The strongest alternative explanation was an optimiser or geometric-model defect. Replacing encoder lengths by recorded geometric lengths in the same sweeps removed the error to reported precision; independent raw-length fitting reproduced the bias. This supports the stretch diagnosis for these simulations, without proving every future autocal error has that cause.

The static elastic model is a design aid: its reported 0.003 mm agreement with the converged simulator and robustness sweeps are simplified-model evidence, not repeated production-world validation. Pose sets alone still gave roughly 6–12 mm in the exploratory model. Its gain reduction (about 1300 to 14) and stiffness/mass/planning-error robustness should not be interpreted as production autocal guarantees. The null-vector force rule holds the weighted mean per-strand tension constant; it does not require configured stiffness. Dataset interpretation must use `drive_anchor` as the torque-mode motor in the second sub-sweep, rather than `sensor_anchor`.

The design study does not repair or retest zero-tension projection; the earlier negative comparison remains evidence against that option alone. A stiffness-aware correction was an initial proposal, superseded as the preferred direction by the owner's requirement to tolerate unknown effective stiffness. Increasing iterations is proposed only for autocal runs.

Next steps are to integrate and validate the single offset and per-point force rule in the production fitter/collector, permit D shortening in sweep selection (D was mandatory fixed and selected delta zero), and test held-out poses, encoder noise, firmware behavior, repeated runs and hardware. Measure both absolute anchor error and translation-aligned shape error. Evidence of production-path failure, failure under heterogeneous compliance, or lack of held-out improvement would weaken the design conclusion. No such validation was performed here.

## Evidence and reproducibility

The evidence directory retains the original conversation, summary, diagnostic scripts (`resid.py`, `fit.py`, `rrd.py`, `an1.py`, `statics.py`, `val.py`), static-model/design scripts (`elastic.py`, `design*.py`), direct collector (`direct.mjs`), driver (`plan.py`), plans/results (`sim_results/`) and three headless datasets/logs/manifests. Scripts retain hard-coded scratchpad paths and are archival helpers, not portable commands from this directory. They were not rewritten or executed during filing. The conversation includes a corrected mode-switch ordering bug in the test collector; only the final reported results are used here.

[Raw artifact inventory](evidence/raw-recordings.json) records paths, SHA-256 and byte sizes. No durable destination was named in the consulted instructions. Only temporary copies of the large RRD/event artifacts exist; they are not committed and may disappear. Access requires the original local filesystem. Small copied evidence is retained in Git. No fresh independent scientific review or reproduction was performed during archival filing.
