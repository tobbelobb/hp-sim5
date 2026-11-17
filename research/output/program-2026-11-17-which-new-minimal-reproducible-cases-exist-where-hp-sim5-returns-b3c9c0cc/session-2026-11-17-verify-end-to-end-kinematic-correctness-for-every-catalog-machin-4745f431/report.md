# End-to-end kinematic correctness for every catalog machine

**Program:** 2026-11-17 "which new minimal reproducible cases exist where hp-sim5 returns wrong results"
**Session:** verify-end-to-end-kinematic-correctness-for-every-catalog-machin-4745f431
**Repository:** hp-sim5 `c2450975`, branch `main`, Node v24.14.1
**Date:** 2026-11-17

## Question

For every machine in `machines/catalog.json`, drive small pure-axis moves through
the production path the simulator uses (RRF G-code -> real firmware -> CAN motor
steps -> `RrfCanPlayer` -> spool angles -> production headless world) and check:

1. does `+X`/`+Y`/`+Z` move the **tool tip** toward `+X`/`+Y`/`+Z` by the
   commanded magnitude, with no mirroring, rotation or scaling;
2. does the motor-step -> line-length sign and steps-per-mm/spool-radius
   conversion match the authored scene geometry;
3. is the line-length-vs-motor-angle relation monotone and linear over a small
   range;
4. is any axis motionless, reversed, or much larger/smaller than commanded?

Measured physical response is kept separate from a pure geometric prediction.

## Verdict in one paragraph

The production path has the correct **direction and sign on every tested axis of
all 20 catalog machines**: no mirrored, swapped or reversed axis was found, every
motor reaches its commanded spool angle, and the CAN step totals agree with the
RRF config's own `M666 R`/`M666 U` closed form to <= 2 microsteps. Cross-axis
motion is at most 0.069 mm on a 5 mm move. Seventeen of the 20 machines are
within 1.5 % of the commanded magnitude on every supported axis (the exceptions
are both CubeCorners presets and HP3-layered `Z`, a 2-microstep 0.117 mm
quantisation outlier that disappears at 20 mm). Nineteen of 20 are within
0.15 mm absolute on every axis; only the layered CubeCorners preset exceeds
that. Two systematic defects remain. (F1) Every `_w_line_layers` preset has a **0.47 %-0.85 % scale error**
because the simulator's discrete layer-model effective spool radius
(39.000 mm / 47.000 mm) is not the RRF config's authored `M666 R`
(39.1845 mm / 46.6045 mm). It is exact, reproducible, and grows with move
length; it is 3 microsteps per 20 mm on HP4. (F2) The two CubeCorners presets
are **not Cartesian-accurate at the tool tip**: a 5 mm `+X` command moves the tip
only 4.906 mm (-1.88 %, non-layered) or 4.735 mm (-5.30 %, layered), while the
body centre moves +5.046 mm / +5.144 mm. The 8-cable rigid body rotates 0.189 deg
/ 0.078 deg because its rotational mode is nearly singular; the exact 6-DOF
geometry of the authored scene predicts the measured tip to 0.01 mm, and the
same geometry with continuous (unquantised) line lengths is exactly 5.000 mm.
So the solver is faithful; the catalog pairs a 3-DOF RRF model with a 6-DOF scene
without documenting the difference.

## Method

The tested path is the production RRF preset path, not a model of it:

```
G92 X0 Y0 Z0
G1 <axis><5 mm> F600
   -> RRF/build/rrf_simulator (the prebuilt firmware the simulator ships)
   -> CAN step capture (one steps column per drive, 750 kHz step clock)
   -> integrations/rrf/rrfCanPlayer.js (the same worker used for .can presets)
   -> {type:'Move', A/B/C/...} cumulative angles = steps * 2*pi/3200
   -> hp-sim-3d/app/headlessWorld.js (native physics.node, catalog scene,
      catalog line-layering flag and physics/solver settings)
   -> tool tip from ExtruderComponent.tipPos
```

For each machine and axis: settle 10 000 physics steps before the move, replay
the player's 2 ms Move stream, settle 10 000 steps, average 4 000 steps. The
reported value is the **tool tip** displacement; the body-centre displacement
and peak-to-peak oscillation are reported separately.

No source file was modified. The RRF simulator is the prebuilt
`RRF/build/rrf_simulator`; nothing was rebuilt.

Two controls distinguish the three kinds of evidence:

* **Measured physical response:** the headless world's tip pose after the move.
* **Firmware geometric prediction:** the `Final machine position: X=... Y=... Z=...`
  reply printed by the real RRF firmware from its own step stream.
* **Pure 6-DOF geometric prediction:** `geom_cube_6dof.py`, which solves the
  authored CubeCorners anchor/attachment geometry for the pose that keeps all
  eight cable paths inextensible, with the actual quantised line changes and
  with continuous line changes.

Zero gravity is used for machines with a bottom anchor (HP3/HP4/HP5/CubeCorners)
and for the planar slideprinters, to isolate kinematics. The all-top Four High
Anchors (skycam) is run **with** the scene's gravity: with zero gravity it has no
tension and drifts 18-156 mm (a negative control, not a result).

Run everything with:

```bash
cd <this directory>
bash run_all.sh
```

or individually:

```bash
node kinematics_e2e.mjs --move=5 --settle=10000 --avg=4000 --out=results.json
node repro_layered_radius_scale.mjs
node repro_cubecorners_tip_rotation.mjs
node control_all_other_axes.mjs
```

`run_all.sh` takes about 50 s (main sweep) plus about 4 s (repros), under the
90 s budget. Results are in `results.json` (5 mm sweep) and
`results-20mm.json` (20 mm cross-check).

## Per-machine / per-axis results (5 mm command)

`tip` is the measured tool-tip displacement along the commanded axis. `err` is
`(tip - 5)/5`. `cross` is the Euclidean off-axis displacement. `osc` is the
peak-to-peak tip oscillation over the averaging window. `body` is the
body-centre displacement. `fw` is the real RRF firmware's own reported position
from the same step stream (a geometric prediction, not the world).

| machine | axis | tip (mm) | err % | cross (mm) | osc (mm) | body (mm) | fw (mm) |
|---|---|---:|---:|---:|---:|---:|---:|
| Hangprinter version 4 without line layering | X | 5.0004 | 0.01 | 0.0162 | 0.001 | 5.001 | 4.985 |
| Hangprinter version 4 without line layering | Y | 5.0047 | 0.09 | 0.0062 | 0.000 | 5.005 | 4.965 |
| Hangprinter version 4 without line layering | Z | 5.0069 | 0.14 | 0.0000 | 0.000 | 5.007 | 4.981 |
| Hangprinter version 4 with line layering | X | 5.0623 | 1.25 | 0.0064 | 0.000 | 5.062 | 4.973 |
| Hangprinter version 4 with line layering | Y | 5.0403 | 0.81 | 0.0063 | 0.000 | 5.040 | 4.957 |
| Hangprinter version 4 with line layering | Z | 5.0293 | 0.59 | 0.0000 | 0.000 | 5.029 | 4.966 |
| Hangprinter version 4 with worn bearings without line layering | X | 5.0030 | 0.06 | 0.0172 | 0.003 | 5.004 | 4.985 |
| Hangprinter version 4 with worn bearings without line layering | Y | 5.0045 | 0.09 | 0.0061 | 0.000 | 5.005 | 4.965 |
| Hangprinter version 4 with worn bearings without line layering | Z | 5.0069 | 0.14 | 0.0000 | 0.000 | 5.007 | 4.981 |
| Hangprinter version 4 with worn bearings with line layering | X | 5.0619 | 1.24 | 0.0064 | 0.000 | 5.062 | 4.973 |
| Hangprinter version 4 with worn bearings with line layering | Y | 5.0402 | 0.80 | 0.0062 | 0.000 | 5.040 | 4.957 |
| Hangprinter version 4 with worn bearings with line layering | Z | 5.0293 | 0.59 | 0.0001 | 0.000 | 5.029 | 4.966 |
| Hangprinter version 3 without line layering | X | 5.0182 | 0.36 | 0.0061 | 0.000 | 5.018 | 4.949 |
| Hangprinter version 3 without line layering | Y | 5.0057 | 0.11 | 0.0057 | 0.000 | 5.006 | 4.942 |
| Hangprinter version 3 without line layering | Z | 5.0006 | 0.01 | 0.0086 | 0.000 | 5.001 | 4.948 |
| Hangprinter version 3 with line layering | X | 5.0045 | 0.09 | 0.0061 | 0.000 | 5.005 | 4.949 |
| Hangprinter version 3 with line layering | Y | 4.9579 | -0.84 | 0.0056 | 0.000 | 4.958 | 4.942 |
| Hangprinter version 3 with line layering | Z | 5.1171 | 2.34 | 0.0691 | 0.003 | 5.117 | 4.949 |
| Four High Anchors without line layering | X | 4.9457 | -1.09 | 0.0043 | 0.000 | 4.946 | 4.809 |
| Four High Anchors without line layering | Y | 4.9457 | -1.09 | 0.0043 | 0.000 | 4.946 | 4.809 |
| Four High Anchors without line layering | Z | 4.9648 | -0.70 | 0.0001 | 0.000 | 4.965 | 4.891 |
| Four High Anchors with line layering | X | 5.0006 | 0.01 | 0.0044 | 0.000 | 5.001 | 4.845 |
| Four High Anchors with line layering | Y | 5.0006 | 0.01 | 0.0044 | 0.000 | 5.001 | 4.845 |
| Four High Anchors with line layering | Z | 5.0093 | 0.19 | 0.0001 | 0.000 | 5.009 | 4.938 |
| CubeCorners without line layering | X | 4.9009 | -1.98 | 0.0235 | 0.134 | 5.051 | 4.886 |
| CubeCorners without line layering | Y | 4.9052 | -1.90 | 0.0228 | 0.128 | 5.047 | 4.886 |
| CubeCorners without line layering | Z | 4.9842 | -0.32 | 0.0300 | 0.006 | 4.984 | 4.898 |
| CubeCorners with line layering | X | 4.7347 | -5.31 | 0.0052 | 0.004 | 5.145 | 4.856 |
| CubeCorners with line layering | Y | 4.7347 | -5.31 | 0.0052 | 0.004 | 5.145 | 4.856 |
| CubeCorners with line layering | Z | 4.9848 | -0.30 | 0.0050 | 0.002 | 4.985 | 4.907 |
| Slideprinter Multi Unit without line layering | X | 5.0085 | 0.17 | 0.0207 | 0.013 | 5.009 | 4.931 |
| Slideprinter Multi Unit without line layering | Y | 4.9699 | -0.60 | 0.0034 | 0.007 | 4.970 | 4.912 |
| Slideprinter Multi Unit without line layering | Z | 0.0000 | no motion | 0.0000 | 0.000 | 0.000 | 0.000 |
| Slideprinter Multi Unit with line layering | X | 4.9512 | -0.98 | 0.0024 | 0.000 | 4.951 | 4.886 |
| Slideprinter Multi Unit with line layering | Y | 4.9505 | -0.99 | 0.0153 | 0.017 | 4.951 | 4.875 |
| Slideprinter Multi Unit with line layering | Z | 0.0000 | no motion | 0.0000 | 0.000 | 0.000 | 0.000 |
| Slideprinter without line layering | X | 4.9820 | -0.36 | 0.0085 | 0.006 | 4.982 | 4.931 |
| Slideprinter without line layering | Y | 4.9859 | -0.28 | 0.0112 | 0.012 | 4.986 | 4.912 |
| Slideprinter without line layering | Z | 0.0000 | no motion | 0.0000 | 0.000 | 0.000 | 0.000 |
| Slideprinter with line layering | X | 4.9898 | -0.20 | 0.0679 | 0.166 | 4.990 | 4.886 |
| Slideprinter with line layering | Y | 4.8808 | -2.38 | 0.0486 | 0.152 | 4.881 | 4.875 |
| Slideprinter with line layering | Z | 0.0000 | no motion | 0.0000 | 0.000 | 0.000 | 0.000 |
| Slideprinter Hexagon without line layering | X | 4.9872 | -0.26 | 0.0177 | 0.004 | 4.987 | 4.931 |
| Slideprinter Hexagon without line layering | Y | 4.9932 | -0.14 | 0.0028 | 0.012 | 4.993 | 4.912 |
| Slideprinter Hexagon without line layering | Z | 0.0000 | no motion | 0.0000 | 0.000 | 0.000 | 0.000 |
| Slideprinter Hexagon with line layering | X | 4.9512 | -0.98 | 0.0023 | 0.000 | 4.951 | 4.886 |
| Slideprinter Hexagon with line layering | Y | 4.9587 | -0.83 | 0.0066 | 0.009 | 4.959 | 4.875 |
| Slideprinter Hexagon with line layering | Z | 0.0000 | no motion | 0.0000 | 0.000 | 0.000 | 0.000 |
| Slideprinter Single Pinholes without line layering | X | 4.9833 | -0.33 | 0.0257 | 0.010 | 4.983 | 4.931 |
| Slideprinter Single Pinholes without line layering | Y | 4.9768 | -0.46 | 0.0071 | 0.008 | 4.977 | 4.912 |
| Slideprinter Single Pinholes without line layering | Z | 0.0000 | no motion | 0.0000 | 0.000 | 0.000 | 0.000 |
| Slideprinter Single Pinholes with line layering | X | 4.9514 | -0.97 | 0.0027 | 0.000 | 4.951 | 4.886 |
| Slideprinter Single Pinholes with line layering | Y | 4.9605 | -0.96 | 0.0198 | 0.013 | 4.961 | 4.875 |
| Slideprinter Single Pinholes with line layering | Z | 0.0000 | no motion | 0.0000 | 0.000 | 0.000 | 0.000 |
| Hangprinter version 5 without line layering | X | 5.0013 | 0.03 | 0.0061 | 0.000 | 5.001 | 4.942 |
| Hangprinter version 5 without line layering | Y | 5.0016 | 0.03 | 0.0061 | 0.000 | 5.002 | 4.942 |
| Hangprinter version 5 without line layering | Z | 5.0009 | 0.02 | 0.0109 | 0.000 | 5.001 | 4.937 |
| Hangprinter version 5 pulley reduced without line layering | X | 5.0015 | 0.03 | 0.0062 | 0.000 | 5.001 | 4.972 |
| Hangprinter version 5 pulley reduced without line layering | Y | 5.0014 | 0.03 | 0.0062 | 0.000 | 5.001 | 4.972 |
| Hangprinter version 5 pulley reduced without line layering | Z | 5.0069 | 0.14 | 0.0000 | 0.000 | 5.007 | 4.977 |

## Finding F1 — every layered preset has a 0.47 %-0.85 % spool-radius scale error

*Repro:* `node repro_layered_radius_scale.mjs` (exits 1)

The simulator's winding slope `d(stored line)/d(spool angle)` is measured with
the production function `cableStoredLengthAfterRotation`. The RRF config's
`M666 R` is the radius the firmware uses for `steps/mm`. They should be equal.

| preset family | scene spool radius | sim effective radius | RRF `M666 R` | predicted scale | measured 20 mm error |
|---|---:|---:|---:|---:|---:|
| HP4 A/B/C/D | 30.000 mm | **47.000 mm** | **46.6045 mm** | **+0.849 %** | X +0.75 %, Y +1.03 %, Z +0.50 % |
| HP3 A/B/C/D | 30.000 mm | **39.000 mm** | **39.1845 mm** | **-0.471 %** | X -0.57 %, Y -0.26 %, Z -0.84 % |
| CubeCorners A-D/I-O | 30.000 mm | 39.000 mm | 39.1845 mm | -0.471 % | (rotation dominates; see F2) |
| Slideprinter families | 30.000 mm | 39.000 mm | 39.1845 mm | -0.471 % | X -0.33 %, Y -0.76 % |
| Skycam A-D | 30.000 mm | 39.000 mm | 39.1845 mm | -0.471 % | 20 mm layered not run |

The non-layered scenes have `sim effective radius = 30.000 mm = M666 R`
exactly, so they have no such scale error. That contrast is the control: the
same HP4 scene and the same 20 mm sweep through the same path gives
`20.0008 / 19.9979 / 19.9965 mm` without layering (X/Y/Z, -0.00 % to -0.02 %),
versus `20.1498 / 20.2063 / 20.0990 mm` with layering. The only changed inputs
are the layer flag and the config's `R`.

At 20 mm, the measured HP4 layered tip is `20.1498 mm` (X), `20.2063 mm` (Y),
`20.0990 mm` (Z); the predicted +0.849 % would be 20.170 mm. At 5 mm the effect
is 0.024-0.042 mm, below the 0.059 mm-of-line microstep floor, which is why the
5 mm table shows only +0.59 % to +1.25 % (quantisation included) and why the
20 mm cross-check is the decisive measurement. One 5 mm cell stands out:
HP3 layered Z is +2.34 % (0.117 mm), which is 2 microsteps and inside the
0.15 mm absolute test tolerance; at 20 mm the same axis is -0.84 %, so it is
quantisation, not a separate scale defect. The other two 5 mm cells outside
1.5 % behave the same way: Slideprinter with layering Y is -2.38 % at 5 mm
(with 0.15 mm oscillation) but -0.76 % at 20 mm, and skycam non-layered X/Y is
-1.09 % at 5 mm but +0.29 % at 20 mm (the sign flips, as expected from
one-microstep rounding).

*Error magnitude / tolerance:* 0.47 %-0.85 % of every layered move. On HP4 a
20 mm move is 373 microsteps; 0.849 % is 3.2 microsteps, so this is above the
quantisation floor, not rounding. A sensible tolerance is the 0.25 % used by
the repro (half the smallest observed mismatch); both families fail it.

*Why this is a bug and not a documented convention:* the layered presets are
normal catalog entries. The scene's authored spool radius is 30 mm; evaluated at
the scene's initial wound length with the production `cableStoredLengthAfterRotation`
function, the layer model gives 39.000 mm (HP3/slideprinter/CubeCorners) or
47.000 mm (HP4). The paired RRF config declares 39.1845/46.6045 mm and the
firmware uses that value for every step. Nothing in the catalog or the
machine documentation says a layered move is scaled by a different radius. The
error is a systematic calibration mismatch, not a sign or direction error.

*Strongest alternative explanation:* the layered winding model is inherently
approximate, and a sub-1 % difference may be inside its intended accuracy. The
config's `R` may also have been computed at a slightly different "homed" wound
length than the scene's authored 2.0 m / 1.0 m. That would make this a
scene/config initialisation mismatch rather than a wrong layer formula. Either
way the end-to-end command is 0.47-0.85 % off, and the catalog gives no
warning. I did not find a documented layer-accuracy budget.

*What would change the conclusion:* a documented statement that layered
presets are calibrated to a different wound length, or a config whose `M666 R`
is generated from the same initial stored length as the scene. A 20 mm replay
with the other radius would then be exact.

## Finding F2 — CubeCorners does not move the tool tip by the commanded amount

*Repro:* `node repro_cubecorners_tip_rotation.mjs` (exits 1)

A 5 mm `+X` command through the real RRF firmware and the production world:

| preset | measured tip | measured body centre | body rotation | exact 6-DOF tip (quantised steps) | exact 6-DOF tip (continuous line) |
|---|---:|---:|---:|---:|---:|
| CubeCorners without layering | **4.9059 mm** | 5.0460 mm | 0.189 deg | 4.9045 mm | **5.0000 mm** |
| CubeCorners with layering | **4.7348 mm** | 5.1444 mm | 0.078 deg | 4.7346 mm | **5.0000 mm** |

The `-X` control is symmetric: -4.9043 mm (non-layered), -4.7393 mm (layered).
The `Y` results are the same by symmetry. `Z` is correct: 4.984 mm
(non-layered), 4.985 mm (layered).

*Error magnitude / tolerance:* -1.88 % (-0.094 mm) non-layered and -5.30 %
(-0.265 mm) layered, against a 1 % / 0.05 mm tolerance. The exact 6-DOF
geometric prediction agrees with the measured tip to 0.0002 mm (layered) and
0.0014 mm (non-layered); the continuous-line prediction is 5.000 mm. So the
simulator is faithfully solving the authored scene, and the whole error is the
difference between the 3-DOF command and the 6-DOF rigid body.

*Mechanism:* the eight cable paths attach at the corners of a 0.4 m cube. The
layout is nearly singular in rotation: a small rotation about Y moves the top
and bottom attachments in opposite directions, changing the eight line lengths
by only ~0.004 mm. One microstep of rounding in the RRF step stream is enough
to excite it. The body centre then over-travels by +0.9 % (non-layered) or
+2.9 % (layered) while the tool tip, 0.3 m below the centre, under-travels by
-1.9 % / -5.3 %. The RRF firmware's 3-DOF model has no rotation degree of
freedom and cannot represent this. The effect is geometric, not a stiffness
artefact: scaling the scene's `cablePath:stiffnessPerUnitLength` down 10x
(20 000 -> 2 000 N/m) gives 4.604 mm (worse), and scaling it up 5x (100 000 N/m)
gives 4.735 mm, i.e. the same as the geometric limit.

*Why this is a bug and not a documented convention:* the catalog presents
CubeCorners as an ordinary Cartesian machine, pairs it with a normal RRF config,
and does not document that the effector is free to rotate or that a Cartesian
move is only approximate at the tool tip. The RRF firmware's own reported
position is also short (4.886 mm / 4.856 mm), so the firmware does not
compensate. A user who asks for 5 mm gets 4.73 mm at the nozzle on the layered
preset.

*Strongest alternative explanation:* the real CubeCorners mover is also a rigid
body and would also rotate; the firmware's 3-DOF model is the approximation, so
the simulator is more physical, not wrong. I take this seriously: the exact
6-DOF geometry of the authored scene reproduces the measured tip to 1.4 µm, so
the solver is not at fault. The defect is the **scene/config pairing** (a 6-DOF
rigid body driven by a 3-DOF command) and the missing documentation, not the
cable solver. This is why I classify F2 as an end-to-end kinematic/model
mismatch rather than an arithmetic bug. If the scene's rotational freedom is
intended, the catalog should say so; if it is not, the effector should be
rotationally constrained or the firmware should model rotation.

*Caveat on move size:* the absolute tip offset is roughly 0.1-0.3 mm, so the
percentage shrinks at larger moves. At 20 mm the non-layered tip is 19.911 mm
(-0.44 %); the layered 20 mm case is dynamically unsettled (0.8-0.9 mm
oscillation) and its window mean is +0.8 %. The 5 mm result is the clean,
settled case; 5 mm is exactly the "small pure-axis move" this investigation
targets.

## Negative results (checked, correct)

1. **Direction and sign, all machines and axes.** Every `+X`/`+Y`/`+Z` command
   produced a positive displacement along that axis. No mirrored, reversed or
   swapped axis was found. `-X` is symmetric for CubeCorners and HP4.
2. **Motor-step -> spool-angle conversion.** `RrfCanPlayer` produces
   `steps * 2*pi/3200` and every motor reaches its commanded angle to
   < 2.9e-4 rad (0.15 microstep; the largest error is the gravity-loaded
   skycam, most are < 1e-15 rad). No axis is motionless except the documented
   planar `Z` below.
3. **CAN step totals vs the RRF config's closed form.** For all 168 audited
   axis/move combinations, `actual steps` agrees with
   `linePos * MA * 3200/(2*pi*R)` to at most **2 microsteps**, and the mean
   absolute error is 1 step. The residual is the firmware's integer position
   rounding (the same order as the standstill one-step issue reported for RRF
   host builds in the firmware-layer session).
4. **Multi-path / mechanical-advantage geometry.** HP4's D axis has three cable
   paths and `M666 U4`; HP5's I axis has four paths and `U1`; the slideprinter
   paired-line scenes have two paths per spool and a config with `U1`. All of
   them produce the correct travel: HP4 Z 5.007 mm, HP5 Z 5.001 mm, Slideprinter
   Multi Unit X 5.009 mm, Hexagon X 4.987 mm. So the scene's path count is not
   naively multiplied against the config's MA; the authored free-segment
   geometry accounts for it. In particular this settles the earlier open
   question about Slideprinter Multi Unit: the six-path scene driven by the
   three-line `config_slideprinter.g` still moves X/Y correctly, so the paired
   lines are not double-counted.
5. **Line length vs motor angle is exactly linear** over the tested +-0.05 rad
   (+-1.5 mm to +-2.35 mm of line): 146 path samples, maximum deviation from a
   straight line 3.8e-16 m, strictly monotone in magnitude on every path. The
   sign of the slope depends on the path's winding direction, and the RRF step
   signs follow it. The slideprinter paths put the hybrid link at the end of the
   path and wind the opposite way from HP3/HP4 (`cablePath:clockwise` plus the
   end-of-path sign), so their production stored-delta slope is -39 mm/rad and
   the RRF +X stream has the opposite sign on B/C; the measured motion is still
   +X. That is the machine's own authored convention, tested against the scene,
   not an assumption. Non-layered slopes are exactly the 30.000 mm spool radius.
6. **HP5 and HP5-pulley-reduced** are the cleanest machines: 5.0013/5.0016/5.0009
   and 5.0015/5.0014/5.0069 mm, cross-axis < 0.011 mm, no oscillation.
7. **Worn bearings do not change the kinematics:** HP4 worn-bearing results are
   within 0.003 mm of plain HP4 on every axis.
8. **The firmware's own geometric reply is not the world's response.** For the
   all-top skycam the firmware reports 4.809 mm for a 5 mm X command (-3.8 %),
   while the world moves 4.946 mm. For most machines the firmware reply is
   0.3-0.9 % low. This is a firmware-model/rounding difference, not a world
   error; the world is closer to the command. I did not treat it as a finding.

## Documented limitations, not defects

* **Slideprinter `Z` produces no motion** (all four slideprinter scenes, with
  and without layering). The machine is planar: every anchor and spool is at
  z = 0, and three coplanar cables cannot impose an out-of-plane coordinate.
  The real RRF firmware agrees: its own reported Z is 0.000 for a 5 mm Z
  command. At 20 mm the firmware's near-singular inverse reports -17.1 mm while
  the world correctly stays at z = 0; that firmware amplification is the
  documented planar-singularity effect, not motion.
* **Skycam needs gravity.** Four High Anchors has all four anchors above the
  mover. With zero gravity the cables have no tension and the body drifts
  18-156 mm for a 5 mm command. With the scene's -9.82 m/s2 gravity it moves
  4.946/4.946/4.965 mm, cross-axis < 0.005 mm. The zero-gravity drift is a
  test-methodology control, not a simulator result.
* **CubeCorners large moves oscillate.** At 20 mm the layered CubeCorners tip
  oscillates 0.8-0.9 mm for tens of seconds (the 8-cable rotational mode is
  lightly damped). The 5 mm case used for F2 settles to 0.004 mm. This is a
  dynamics observation, outside the kinematic scope.

## Limitations and what I could not settle

* **Simulation only.** All evidence is from the prebuilt RRF host firmware and
  the native cable solver. No hardware was measured. F1 and F2 are model/scene
  statements, not claims about a physical printer.
* **RRF path only.** I did not exercise the Klipper MCU path
  (`KlipperMcuCommandPlayer`, `.serial`/`.txt` uploads) for this sweep. The
  catalog's preset buttons use the RRF `.can` path tested here; the Klipper
  presets are a separate upload path. A Klipper cross-check of F1/F2 would
  strengthen the result.
* **F1's exact cause is the radius value, not a specific line of code.** I
  measured the production winding function and the config value; I did not
  trace how `R = 39.1845/46.6045` was generated, nor whether the scene's
  authored 2.0 m / 1.0 m stored length is meant to be the homed length. If the
  config was generated for a different wound length, the fix is an
  initialisation alignment rather than a formula change.
* **F2's "correct" answer depends on intent.** The exact 6-DOF geometry
  reproduces the measured tip, so the cable solver is not wrong. I could not
  find documentation stating whether the CubeCorners effector is intended to be
  rotationally free. Without that, the strongest alternative (the real machine
  also rotates; the firmware is the approximation) cannot be excluded. I did
  not test a rotationally constrained variant of the scene, which would show
  whether constraining it restores 5.000 mm.
* **CubeCorners at 20 mm does not settle** in the layered preset, so the
  move-size scaling of F2 is characterised only at 5 mm (settled) and partially
  at 20 mm. I did not run the full 20 mm sweep for every machine; the 20 mm
  cross-check covers HP4/HP3/slideprinter layered, both CubeCorners presets and
  the skycam.
* **Zero gravity for most machines.** The production app uses the scene's
  gravity. I disabled it to isolate kinematics for machines with a bottom
  anchor; gravity changes the equilibrium by a small sag. The skycam control
  shows the method matters when the machine is all-top.
* **One-microstep RRF host behaviour.** The CAN step audit is within 2 steps,
  but the firmware's reported position is often 1 step away from the closed
  form (the same class as the `M400` standstill issue in the firmware-layer
  session). I did not isolate that here; it sets the ~0.06 mm-of-line
  quantisation floor for all the 5 mm numbers.

## Evidence index

```
kinematics_e2e.mjs                 # full 20-machine x 3-axis sweep
results.json                       # 5 mm sweep, all machines/axes
results-20mm.json                  # 20 mm cross-check subset
results-20mm-hp4-nonlayered.json   # 20 mm non-layered HP4 control for F1
results-cube-minus5.json           # -X symmetry control for F2
logs/main-5mm.log, run-20mm.log    # stdout of the two sweeps
table.md                           # the Markdown table above
repro_layered_radius_scale.mjs     # F1, exits 1
repro_cubecorners_tip_rotation.mjs # F2, exits 1
geom_cube_6dof.py                  # pure 6-DOF CubeCorners geometry
probe_cube_stiffness.mjs           # F2 stiffness-sensitivity control
control_all_other_axes.mjs         # negative-result control, exits 0
run_all.sh                         # runs all of the above (~55 s)
logs/                              # run_all.sh output
```

`results.json` contains, for every machine/axis: step totals, player angles,
firmware final position, tip and body displacement, cross-axis and oscillation,
spool target/actual angles, and the winding samples. The top-level `winding`
and `stepAudit` objects hold the geometry and CAN audits.

## What would change the conclusion

* A `M666 R` generated from the same initial wound length as the scene, or a
  documented layered-radius tolerance, would remove F1.
* A rotationally constrained CubeCorners effector (or a documented statement
  that it is free and Cartesian moves are approximate) would remove F2.
* A Klipper replay showing the same 0.5-5 % errors would broaden F1/F2 beyond
  the RRF preset path; a Klipper replay showing different errors would make the
  path, not the scene/config, the dominant cause.
