# Research notebook

Objective (verbatim task): improve encoder-based CDPR autocalibration in hp-sim5.
The current algorithm can be slow, fragile, or choose worse anchors because its
selection score disagrees with actual geometrical accuracy. First reproduce and
measure those cases, then improve speed, robustness and/or UX. Priority interest:
robust results with **fewer or faster-to-collect** data points. Preserve the
one-click UX and automatic stopping. Ship the best-supported change.

## Constraints (from the user)

- `--buildup-factor 0`; simulator runs with Line Layering **disabled**.
- Do **not** search spool radii (`--find-radii off`).
- Use Hangprinter 3 (`hangprinter_3`), not HP4.
- Reference to beat: `output/extended-autocal/no-buildup-reference-003`.
- Keep collection routines similar/equivalent so the autocal regressions stay
  meaningful; otherwise compare new vs old explicitly.
- Repo rules: `.venv/bin/python`, small code changes, shallow stack traces,
  no commits/branches, no copyright headers, do not fix unrelated bugs.

## Budgets

- Fresh headless full-auto runs: 3 planned (E05, E17, E18). **3 used, all finished.**
- Bounded live collection experiments: <= 10 x <= 3 min wall. **0 used** (all
  collection questions were answered by replaying already-recorded traces).
- Offline replays of recorded/synthetic data: unbounded (cheap).

## Accepted steering (chronological)

1. Profile the reference `.rrd` first (per the user) to find where time goes,
   before choosing between "faster per point" and "fewer points".
2. E01/E01b: the encoder-quiet wait loop owns ~63 % of simulated time and the
   quiet window is its only safe knob -> ship the window change and validate it
   end-to-end at reference density (E17).
3. Reproduce the selection-score/accuracy disagreement (E14b/E15) because the
   task names it, then decide whether a selection fix is small enough to ship.
4. Test "fewer points per sweep" end-to-end (E18), since that is the user's
   headline interest and E13 only had offline replay evidence. Result: viable
   but inside run-to-run noise, so the default density is left alone.
5. Accepted mid-session: instead of replacing the ranking score (already a
   documented negative result in `autocal-ranking-bounded-updates-negative-results-b7deb1bc`),
   keep the ranking as the reported diagnostic and fix only the **applied
   geometry** with a bounded geometric consensus of the fitted candidates.
6. E39 (offline, free): tested the obvious follow-up to E38, a
   convergence-based early stop. Rejected, so no stopping logic changed and the
   one-click UX and automatic stopping are exactly as before.

## Environment facts

- HEAD = `ded5842e`. Native launcher attached; world reconfigured to HP3,
  layering disabled, base radii 30 mm, buildup 0 (`sys/config_hp3.g`,
  `public/usd_scenes/hp3_rigid_body.usda`).
- `research_preflight` passed collector/firmware/capabilities/recorder. The
  `settings` check reports only `python_requires_launcher_restart`, caused by a
  touched `scripts/research_agent.py` whose content already equals HEAD. This
  session's production changes are `autocal/*.py` and one `.mjs` constant, which
  the supervisor hash check does not hash, so the mismatch does not affect the
  runs.
- `scripts/research_socket_probe.py` was not run; supervisor connectivity is not
  command-socket access (see Limitations in `report.md`).
- Truth anchors (mm), the simulator's nominal HP3 geometry, never an input to any
  selector: `A=[0,-1900,-280] B=[1645.44826719,950,-280] C=[-1645.44826719,950,-280] D=[0,0,1900]`.
- Metric: **applied-anchor RMSE** = `sqrt(mean_i ||a_i - truth_i||^2)` over the 4
  anchors, parsed from the run's own final `Anchors (M669):` line. This is the
  geometry a user would measure, and it is what the shipped change targets.
- A sweep entry in `sweeps.json` holds **2 x `--sweep-points`** data points (out
  and back per step), one config per sweep. So the reference's 9 sweeps x 20 pts
  = `--sweep-points 10` (the collector default); E05 used 20; E17 used 10; E18
  used 6.

## Experiment log

| ID | Question | Method | Result | Status |
| --- | --- | --- | --- | --- |
| E01 | Where does reference time go? | RRD extract -> per-command simulated seconds | settle poll loop 63 %, moves 20 %, force tuning 10 %, noise sampling 6 % | done |
| E01b | Per-phase settle attribution | RRD phase table | 817 multi-poll waits = 3886 s; force-mode-change 2602 s, move 1285 s | done |
| E05 | HEAD baseline at 2x density | supervised full-auto, `sweep_points=20` | 6805 sim-s, 1345 s wall, 11 sweeps, 440 pts, applied 2.977 mm | done |
| E07 | Which quiet window is safe? | replay criterion over reference poll traces | window 1000 ms saves 490 s (7.3 %) at <= 0.04 deg p99; tolerance/vibration relaxations raise p99 to 7-20 deg | done |
| E16 | Wider criterion sweep | E07 extended with vibration-window and HEAD traces | window 1000 ms is the largest safe saving; vib 3000/2000 and tol 2.0/3.0 rejected | done |
| E16b | Same replay on HEAD's own trace | E05 RRD -> gcode events -> criterion replay | window 1000 ms saves **528.5 s / 7.8 %**; p99 0.12 deg, max 1.25 deg | done |
| E13 | Fewer points per sweep? | offline dense-thinning replay | 6/10 pts keeps last-iteration accuracy (2.64 vs 2.53 mm); 5/10 breaks; 4/10 diverges | done |
| E14b/E15 | Does the selection score agree with accuracy? | replay, per-iteration stage anchors + `history_select` table | selected iteration worse than the best *selectable* one by 0.09-1.8 mm; the newest iteration is structurally unselectable | done |
| E17 | Does HEAD+window1000 match reference accuracy at reference density? | supervised full-auto, `sweep_points=10` | 5807 sim-s, 1054 s wall, 11 sweeps, 220 pts, applied **2.580 mm** | done |
| E19-E32 | Which anchor-ensemble rule is safe? | offline rule matrix over persisted stage artifacts | naive ensembles blow up (mean of all 907 mm, mean of top-3 by rank 485 mm, 2x-best-rank band 291 mm); minimum-diameter pair is bracketed and safe | done |
| E33 | Shipped selector vs consensus, de-duplicated real datasets | production `consensus_anchor_estimate` over every persisted `history-*.json` | consensus better on 10/11, worse on 1; mean 4.273 -> **3.092 mm**, worst 10.060 -> **4.353 mm**, mean regret vs oracle 1.594 -> 0.414 mm | done |
| E35 | Does the shipped code reproduce the validated `md2` rule? | 11 real datasets, 5-point merge-tolerance sweep | max anchor deviation `0.0e+00` mm; identical RMSE at every tolerance 1e-3..1.0 mm | done |
| E36 | Applied-anchor RMSE of every fresh run | parse each run's own final M669 | reference 2.828, E05 2.977, E17 2.580 | done |
| E37 | Per-command sim-time profile, headless runs | `events.jsonl` send->next-send attribution | force changes ~50-64 % of sim time; E17 vs E05 at equal sweeps: 5807 vs 6805 sim-s | done |
| E18 | Does 40 % fewer points still calibrate at HEAD? | supervised full-auto, `sweep_points=6`, window 1000 + consensus selector | accepted, 10 sweeps, 120 pts, 5032 sim-s, 934 s wall, applied **2.192 mm** (best of the four runs) | done |
| E34 | Replay reference *data* with the consensus selector | offline `history-007` replay | applied 2.828 -> 2.623 mm on identical data | done |
| E38 | Does each extra sweep buy accuracy? | `fit-NNN` progression of every fresh run | no: E05 final (2.459) already at sweep 2, reference 2.809 at sweep 4/7, E18 2.2 at sweep 3; single fits diverge (2112, 2333, 1877, 1395 mm) while the consensus stays put | done |
| E39 | Can consensus stability stop collection early? | offline stopping-rule matrix over 40 candidate sequences | rejected: <=15 % sweeps saved for up to 2.627 mm accuracy loss (E17 would stop on a 5.110 mm plateau) | done |

## Hypotheses and verdicts

- **H1** "settle reuse never fires (bug)". **Rejected.** Revision artifact:
  reference-003 predates the reuse commit `536a2e36`; at HEAD M569.4 is sent once
  per point.
- **H2** "much more can be won by ending settles earlier". **Mostly rejected.**
  A causal oracle over move waits saves only 8.8 % of move-wait time.
- **H3** "the 1500 ms quiet window can shrink to 1000 ms". **Supported and
  shipped** (E07, E16b; 528 s / 7.8 % at p99 0.12 deg).
- **H4** "fewer points per sweep keeps accuracy". **Mixed.** E18 finished the
  end-to-end test at 6 points/sweep and it is the best run of the session
  (2.192 mm, 120 pts, 5032 sim-s), but the offline thinning corpus trends the
  other way (pts10 2.62 < pts8 2.83 < pts6 2.90; pts4 degenerates to 1220 mm),
  so the effect is inside run-to-run noise. No default changed.
- **H5** "relaxing tolerance or the vibration window is a safe extra saving".
  **Rejected** (E16: p99 7-20 deg).
- **H6** "the selection score disagrees with geometric accuracy". **Supported**
  (E14b/E15, and now E33 on 11 datasets): up to 1.8 mm of regret and the newest
  iteration can never win.
- **H7** "more points help". **Rejected** (E05: 2.4x points, no accuracy gain).
- **H8** "replacing the ranking score fixes selection". **Already rejected by
  prior sessions** (`autocal-ranking-bounded-updates-negative-results-b7deb1bc`:
  22 rank/accuracy mismatches; removing the age bonus cost +200.701 mm; whole-sweep
  prediction worse in 5/11). This session therefore does **not** touch the
  ranking; it keeps it as the diagnostic and fixes the applied geometry instead.
- **H9** "the geometric consensus of the fitted candidates is closer to the truth
  than the single best-ranked candidate". **Supported** (E33/E35): mean applied
  error 4.273 -> 3.092 mm, worst case 10.060 -> 4.353 mm over 11 distinct real
  candidate sets, and it improves all three fresh runs (reference 2.828 -> 2.623,
  E05 2.977 -> 2.459, E17 2.580 -> 2.483).
- **H10** "any ensemble of candidates is safe". **Rejected** (E19-E32): the
  unguarded mean, the rank-band mean and centrality-k means blow up to 291-907 mm
  when one candidate is a degenerate near-planar fit. Only a rule that stays
  *bracketed by real fits* (the minimum-diameter pair after near-duplicate merge)
  was safe on every dataset.
- **H11** "fewer points per sweep also cuts collection wall time proportionally".
  **Rejected as stated** (E37, confirmed by E18): most sim time scales with
  *sweeps* (force changes, pullouts, retraction), not with points. Cutting
  220 -> 120 points at ~equal sweep count saved ~13 % of sim time, not the 45 %
  the point count suggests.
- **H12** "a stable consensus estimate is a safe stopping signal". **Rejected**
  (E39): the applied estimate can sit on a plateau far from truth (E17 held
  5.110 mm for three consecutive sweeps), so the rule saves <=15 % of sweeps and
  can cost up to 2.627 mm. Stopping logic therefore stayed untouched.

## Ship set

1. `autocal/control/primitives/uncalibrated_actions.mjs`: quiet window
   1500 -> 1000 ms (H3). Production behaviour change, ~7.8 % of simulated
   collection time, one numeric constant plus its test expectations.
2. `autocal/history_selection.py` + `autocal/autocal.py`: apply the geometric
   consensus of the fitted candidates (H9/H10) while keeping the existing ranked
   candidate as the reported fit-quality diagnostic. New unit tests in
   `autocal/tests/test_history_selection.py`.

Deliberately **not** shipped: a point-density default (H4/H11 mixed, E18) and a
convergence-based early stop (H12 rejected, E39).

End-to-end evidence for the ship set: E17 (window, 10 pt/sweep) 2.581 mm vs
reference 2.828 mm; E18 (window + consensus, 6 pt/sweep) 2.192 mm; E34
(consensus on the reference's own data) 2.828 -> 2.623 mm. All better than the
run to beat, and the regression suites are green (351 Python, 106 JS).

## Next decision

None pending; the session is complete. E18 (accepted, 2.192 mm, 120 pts,
5032 sim-s) confirmed the shipped selector is robust at lower point density but
the replay corpus does not justify changing the default, so `--sweep-points`
stays a user knob. If this research is continued, the highest-value follow-up is
a hardware or higher-fidelity test of the two shipped changes, and next after
that an investigation of why the loop collects far more sweeps than E38 shows
the geometry needs - specifically whether the planner, not the stopper, is
what keeps spending sweeps.
