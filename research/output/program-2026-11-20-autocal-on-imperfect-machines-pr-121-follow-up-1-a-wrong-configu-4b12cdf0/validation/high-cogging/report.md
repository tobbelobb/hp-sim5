# HP5 autocal under elevated motor cogging

These are headless simulations. Anchor error is the largest absolute coordinate error in the print frame, with home at the origin, the top anchor on the z-axis and A.x = 0. Time is simulated machine time.

## Tension cap

The matched uncapped HP5r runs at 3× cogging, seeds 5 and 6, completed rough geometry and failed during the second profiled bootstrap sweep. Both missed a drive tracking target by about 0.088 mm against a 0.079 mm tolerance. The cap at 0.75 × M666 X let both repeat runs finish all four profiled bootstrap sweeps with the same tuned forces, rough fixed targets and simulator inputs.

Rebuilding the force tables on the same rough fits reduces the 3× strand target from 7.078 to 4.955 N and the predicted peak line force from 46.5–48.8 N to at most 30 N. The cap also changes the supported arc. The comparison measures their combined effect. The extra sensor remains at 7.769 N. The earlier 18 nominal bootstrap profiles were byte-identical with and without the cap.

## Stopping policy

On HP5 and HP5r with spare sensor ranges, keep the two-fit fast path at RMS ≤ 0.05 mm. The stable-estimate path now requires two successive new measurement groups for which every anchor coordinate moves at most 0.5 mm. Both updates must have at least four measurement groups, adequate spare range coverage, full information rank, a stretch-corrected fit, estimated anchor standard deviation ≤ 1 mm, range RMS ≤ 0.12 mm, and sweep jackknife ≤ 2 mm. This covers the raised residual floor without accepting a flat series of weakly constrained fits.

Re-evaluating the original HP5r 2× seed 6 fit history with the collected sweep prefixes yields a stop at 8 sweeps and 4,234 s. That fit has 1.539 mm error. The original run continued to 23 sweeps and 8,614 s and applied a fit with 1.543 mm error. Six paired nominal replays on the same sweep data selected identical anchors and used five sweeps under either policy. The gate controls observed stability and sensitivity to individual sweeps. A systematic bias shared by all measurements can still pass it.

## Live stress runs

All four HP5r runs completed rough and profiled bootstrap, collected
refinement sweeps and applied a calibration on `f09bc8c6`. The scenes retain
the benchmark harness's randomized anchor positions and stiffness. Seeds 5 and
6 have true EA of 0.963 and 1.070 times configured EA, respectively.

| Seed | Cogging | Total sweeps | Stop | Simulated time (s) | Largest coordinate error (mm) |
| --- | ---: | ---: | --- | ---: | ---: |
| 5 | 2× | 10 | stable-estimate | 4,585 | 1.60 |
| 6 | 2× | 10 | stable-estimate | 4,779 | 0.94 |
| 5 | 3× | 20 | patience | 6,356 | 1.69 |
| 6 | 3× | 18 | stable-estimate | 5,996 | 1.03 |

The 3× seed 5 fit never met the 1 mm estimated standard-deviation threshold
and used the existing patience stop. Its 12-sweep fit had just 0.41 mm error,
while seed 6 was still at 2.22 mm after 12 sweeps. Collecting longer improved
seed 6 and worsened seed 5, so the two seeds do not support a uniformly
earlier 3× stop. The cap prevented the previously observed bootstrap failure
on both seeds.

The [live fixtures](evidence/live/) contain each selected fit, learning curve,
collected ranges and manifest. Complete local run logs are under the checkout's
ignored `output/research/pr122` directory.

## Profiled slip recovery

The collector reduces positioning and pullout forces after a small slip and retries the whole direction with the same measurement force profile. If repositioning reaches idle preload or the retries run out, the collector can reject an active sweep and the planner can choose another one. Bootstrap still requires a complete set of measurements. A drift larger than one revolution still stops acquisition with the motors held. The collector unit tests cover both bounded rejection and the gross-drift stop.

## Reproduction

Use a short output path for headless simulation. This example runs the HP5r 3× seed 6 case. For the paired HP5r runs use seeds 5 and 6 at each cogging level.

```bash
.venv/bin/python -m autocal.tools.imperfect_machine /tmp/ac122-repro \
    --machine hangprinter_5_pulley_reduced --seed 6 --cogging-scale 3
```

The archived uncapped controls are under `evidence/runs/cog*-hp5r-*` in the rough-bootstrap investigation at Git commit `cb086259`. The [2× seed 6 fixture](evidence/archived-2x-seed6/) contains the exact archived dataset, the previous fit history and the short current replay log. The full-auto CLI replay independently stops at the same eight-sweep prefix. For reproduction, set `evidence` to the `evidence/` directory beside this report.

```bash
replay=$(mktemp -d /tmp/ac122-replay.XXXXXX)
gzip -dc "$evidence/archived-2x-seed6/sweeps.json.gz" > "$replay/sweeps.json"
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m autocal.autocal \
    --sim --no-collect --machine-type hangprinter_5_pulley_reduced \
    --dataset "$replay/sweeps.json" --patience 8
```
