# HP5r top-anchor height bias and paired collection

All results are from simulation. The archived replays use the PR #122 fitter at `7a57be78`. The replay is a follow-up to the
[compliance probe](../session-2026-11-20-compliance-probe-where-stiffness-really-trades-against-the-top-a-62081729/report.md)
and [force-realisation study](../session-2026-11-21-force-realisation-error-in-autocal-data-replays-t3-test-complian-8252ab16/report.md).
Those studies found a positive I.z remainder after substituting measured or
simulator-derived forces, and motivated a test of effector rotation.

HP5r's remaining positive I.z bias after measured-force compensation comes
mainly from small effector rotations moving the low-line attachment points.
With the same recorded translation, freezing orientation takes the
geometry-only height error from +0.35–0.59 mm to +0.04–0.07 mm. There is no
production correction. The existing encoder records do not independently
measure orientation, and the frame-side and effector-side routing would also
need to be known for an attitude-based correction.

## Matched replay and geometry controls

Replay the native physics at the recorded commands and steps. Verify replay
encoder values against every recorded full encoder response by request ID.
Match the collector's sample windows by simulation time and check their mean
encoder readings against the stored means. Retain the collector's finite range
mask, including its extra-sensor coupling decisions. Use the scene EA and
convention-frame truth. Report signed I.z and the maximum absolute coordinate
error. Use the production robust point objective and only change the ranges or
the flex input under test. Initial anchors for the diagnostic refits come from
the original run's fit. The production-model comparison allows the standard
home offsets. The geometry ablations share zero offsets so an offset cannot
hide one intervention and amplify another.

In effector-distance units, combine the straight spans and internal wraps within
each route and divide by mechanical advantage, then average parallel paths.
Write the change in geometric cable length as Δg and the change in signed
straight-span extension as Δs. The encoder delta should agree with Δg − Δs.
The signed extension includes slack when a span is shorter than its paid-out
rest length. Compare this with positive extension alone to see the importance
of that assumption. The forward controls fit the generated geometric ranges
with the original accepted-range mask and no elastic model.

The rotation control uses the same effector positions throughout. It rotates
each rigid-body-side endpoint to its home orientation about the recorded body
centre, keeps the frame side fixed, and leaves the logged internal wraps in
place. Rotation contributes the difference of the observed and frozen chord
lengths. Separate controls change only the four low lines or only I. The pure
point control computes distances directly from the truth anchors and fixed
translations. The frozen chord model does not re-solve tangency or settle a
new trajectory. This matters chiefly for the wheeled I routes. The low
frame-to-effector spans run between fixed points and pinholes.

Downstream controls use the same points at the scene EA, and compare inferred
static tensions, replayed tensions, the additional 85.512 mm of low-line
reeving in the compliance model, the initial elastic extension, and gravity
aligned with the physical world or the tilted print frame. These are
diagnostic substitutions. Replayed orientation, per-segment force, and scene
EA are unavailable to the live fit.

### Results

The native replay reproduces 10,943, 10,776 and 10,376 full encoder
responses exactly, on seeds 1, 5 and 7 respectively. All 100 windows on each
seed match within 0.0047 degrees of the collector's rounded means. Median
attitude change is 0.0055–0.0069 degrees and the maximum is 0.013–0.030
degrees. Individual low-line corrections reach 0.023–0.096 mm. I's own
rotation-induced chord changes are much smaller.

All table entries are signed I.z errors in mm. Columns use the same measured
ranges where applicable and the same per-point measured-line mask.

| Control | Seed 1 | Seed 5 | Seed 7 |
| --- | ---: | ---: | ---: |
| Production force and offset model, true EA | +1.119 | +0.588 | −0.085 |
| Measured forces, true EA | +0.509 | +0.550 | +0.444 |
| Measured forces, remove rotation | +0.054 | +0.265 | −0.111 |
| Measured forces, remove all routed vs point geometry | +0.018 | +0.200 | −0.149 |
| Forward routed geometry with observed attitude | +0.491 | +0.351 | +0.593 |
| Forward routed geometry at home attitude | +0.036 | +0.066 | +0.038 |
| Forward rotation on low routes only | +0.511 | +0.363 | +0.617 |
| Forward rotation on I only | +0.016 | +0.054 | +0.013 |
| Forward reversal of the measured chord correction | −0.418 | −0.219 | −0.517 |
| Forward exact point ranges | <0.001 | <0.001 | <0.001 |

The low-line effect accounts for nearly all of the geometry-only I.z shift.
The sign reverses when the observed chord correction is mirrored. The
near-horizontal lows resolve vertical motion weakly, so their small range
changes propagate into the height attributed to I. Using measured force
reduces the force-model residual enough to reveal the common mechanical
error. It can also expose a height bias that commanded-force error happened
to cancel, as on seed 7.

The measured-force fit without the rotation correction has 0.44–0.55 mm
maximum coordinate error. With the correction it has 0.20–0.36 mm. The
remaining discrepancy matters. The recorded encoder deltas and Δg − Δs
close to within 0.041–0.048 mm on measured ranges, with RMS of about 0.006–0.026
mm across individual axes. Substituting signed replay extension and removing
rotation resolves much of I.z but leaves larger non-top errors than the
measured-force fit. Rotation accounts for the common height error, with other coordinate errors left over.

With measured force and the original geometry, adding the omitted low-line
reeving changes I.z to +0.572, +0.350 and +0.595 mm, a mixed outcome. The
home-extension substitution leaves +0.585, +0.956 and +0.473 mm. The gravity
convention changes I.z by at most 0.005 mm. Mean vertical force from the replay
is about 0.795 N, consistent with configured mass. Lower RMS after a modelling
change was therefore insufficient evidence for a geometric correction.

The 300 windows report four or five cable ranges each. Three per-point
coordinates are already needed for translation, leaving at most two scalar
range constraints for three possible attitude components, even given the
anchors and rig geometry. This check does not address a separately calibrated
one-axis or mechanical rotation model. The controls establish the effect of
measured rotation in the simulator. They do not establish a live estimator or
a hardware result.

## Paired collection

Fit both shapes with the same current robust point objective, seed-scale
schedule and home-offset rule. Compare each final dataset, then compare the
prefixes available by the earlier run's end time. The prefix includes entire
sweeps or complete mesh visits whose last saved measurement is within the
budget. The time test uses source acquisition clocks. Live stopping and mesh
height selection retain their respective production paths. Thus the same-model
comparison controls the final objective, while the live patterns still include
their different force schedules, planning and stopping.

The archived nominal inputs were collected on `e022c095`. Their refits use
`7a57be78`. This is a same-revision pair within each archived machine and a
single nominal scene per machine. Fresh HP5r pairs use the same machine and scene within each seed and true EA at 1.2× (seed 1) or 0.8× (seed 3) configured EA. The true-EA refit is an offline sensitivity check. These comparisons use the commanded-force fit.

### Archived nominal pairs

Maximum absolute coordinate error in mm. The common budget is the earlier
completion time, with complete visits only. Final errors use both final
datasets even when the runtimes differ.

| Machine | Budget (s) | Sweep at budget | Ellipsoid at budget | Sweep final | Ellipsoid final | Final sweep / ellipsoid time (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| HP3 | 4,587 | 0.55 | 2.93 | 0.55 | 2.74 | 4,587 / 6,068 |
| HP4 | 4,622 | 0.75 | 0.46 | 0.72 | 0.46 | 4,924 / 4,622 |
| HP5 | 3,778 | 1.25 | 0.92 | 1.25 | 1.24 | 3,778 / 6,094 |
| HP5r | 4,686 | 1.06 | 3.22 | 1.06 | 2.94 | 4,686 / 6,495 |

The archived comparison is mixed by machine. HP5's later mesh points make its
final fit worse than its budget prefix. HP5r's nominal meshes cost 39% more
simulated time and the common point fitter leaves almost three times the
final error, chiefly I.z. These are single nominal instances at the earlier
collector revision.

## Reproduction

Use `.venv/bin/python` from the checkout root. The compact fixtures hold
recorded datasets and run metadata. Reduced replay windows hold per-point
positions, orientation, cable totals, force and the accepted-window mask. Large
raw recordings remain available at the local locators in the fixture manifests;
the SHA-256 and byte counts identify them. No off-host storage was established.
The reduced evidence can reproduce the fits without the raw event files.

From the repository root, point `evidence` at the `evidence/` directory next
to this report.

```bash
export PYTHONPATH=. OPENBLAS_NUM_THREADS=1
.venv/bin/python "$evidence/analyze.py" "$evidence/replay-1" \
    --reduced "$evidence/replay-1/window-means.json.gz"
pair="$evidence/nominal/hangprinter_5_pulley_reduced"
.venv/bin/python "$evidence/compare.py" "$pair/sweeps" "$pair/ellipsoid"
```

The force-and-geometry controls for seeds 1, 5 and 7 are recorded in the
[reduced replay fixtures](evidence/). The seed 1 result was independently
reproduced from the reduced windows on the integrated branch. The archived
pair results are in the `pair-*.json` files in the same directory.
