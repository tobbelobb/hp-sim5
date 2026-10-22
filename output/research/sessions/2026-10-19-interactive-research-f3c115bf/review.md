# Independent review: XPBD damping change and HP4 autocal run

## Decision

Imported the solver damping fix and simulated-time deadline fix in commit
`bf9f5399` (`Fix HP4 XPBD damping feedback and simulation waits`). The damping
change is supported as a targeted correction for the reproduced HP4 limit
cycle. Evidence does not show that the fix completes default one-substep
autocal, improves calibration accuracy, or fixes the separate curved/drifting
travel visible in the two-substep recording.

## Runtime review

The original E12 legacy/fixed JS timing had only three observations and did not
justify a precise percentage. I repeated the same 1,000-step loaded-checkpoint
workload five times per mode, alternating legacy then fixed, on the same JS
backend. Median times were 5.190 s legacy and 5.139 s fixed (fixed was 1.0%
faster). The observed ranges were about 4-5% of the median, so the defensible
conclusion is no measurable runtime change for this solver path. This is not a
full-autocal or recording benchmark.

The user's command used the browser JS backend with `--record`, at two physics
substeps and a requested 25x speed scale. Its finalized recording covers 892,890
steps / 1,785.78 simulated seconds over about 738 wall seconds (2.42x observed
simulation-to-wall ratio), contains 8.24 million rows in 83,830 chunks, and is
528 MiB on disk (1.3 GiB uncompressed). Rerun reports roughly 102 times more
chunks than its 2 MiB chunk-size optimization target. Recording and browser
workload dominate this run's elapsed time; the small damping cache is not a
plausible explanation for a 100% slowdown. The requested speed scale is not a
promise of 25x throughput on this workload.

## User run and trajectory

The full-autocal log shows force tuning selected `forceMax = 5.9763 N` after a
force edge at 5.9763 N and `d_max = 3196.260 deg`. Size tuning then applied
fractions of that force to a selected cable pair, measured 756.231 mm raw
travel, and scaled it to 604.985 mm. The displayed effector trace is visibly
curved and loops rather than following a straight segment. The size-tuning
code measures encoder-derived cable travel on the other motors while applying
force to a pair; it does not command a straight Cartesian trajectory, so that
visual trace is not itself evidence of an incorrect Euclidean distance
calculation. It does show the motion is not geometrically straight.

The first sweep timed out waiting for encoder stability. The recording
metadata says `autocal_complete: false`; it is a failed prefix, not a completed
calibration run. The prior headless experiments also report monotone payout
drift at one and two substeps, distinct from the reproduced 36 Hz limit cycle.
The report's four-substep success is useful as a control, but its fitted size
measurement differed materially from the two-substep fixed-force trial, so it
does not establish that the fix makes autocal generally accurate.

## Benchmark integrity and limits

The wobble A/B procedure is credible: E13 runs both arms in JavaScript from the
same loaded checkpoint and differs by `solverMode`; E14 records per-visit
displacement directly. Earlier E09 cross-backend evidence is weaker, but the
same-backend E13 pair addresses its main confound. Substep timing rows and
solver-iteration comparisons answer different questions and should not be
presented as the runtime cost of this damping fix.

The patch changes the damping displacement sampled by each constraint, keeps
the multiplier equation and iteration count, and mirrors the behavior across
JS, Rust/Wasm, Python and Warp. This is a targeted improvement for the tested
stiff HP4 state, not a general proof across all machines or solver states.
The first-visit sample can still depend on earlier constraints in the
Gauss-Seidel order; capturing the velocity quantity before projection would be
a more complete step-level treatment if broader cases expose order effects.

## Preserved artifacts

The session's reports, experiment scripts, numeric traces, small logs and
metadata are copied beside this file. The user's full-autocal log, bootstrap
configuration and recording JSON/recorder log are under
`output/extended-autocal/no-buildup-reference-010/`. RRDs, files over 5 MiB,
large event dumps, and private connection/viewer state were excluded.
