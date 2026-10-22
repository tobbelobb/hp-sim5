# Encoder-only local endpoints: HP3 improvement, HP4 incomplete

Status: complete bounded evaluation — experimental candidate; not validated as an improvement across both machines.

## Abstract

We replaced global edge-force tuning with local position-controlled endpoint discovery using encoders and existing low-preload commands. Rust Native headless production trials requested10–25x speedup. HP3 completed24 points across three configurations and both directions, with canonical cable spans approximately996–1020mm versus364–524mm for the preceding continuous-force candidate, using686.644 versus1023.390 physical seconds. Requested10x/25x repeats produced identical raw encoder arrays. HP4 origin-only collection completed eight points with a708.71mm canonical span and also repeated exactly at10x/25x. Full HP4 trials exposed oscillation around a free-encoder plateau: absolute step response allowed excessive winding. An eight-move net-response check corrected that observed search failure, but full collection remained expensive. The final HP4 trial reached21 of24 points before its360-second wall budget, with no final dataset. Thus an overall HP4 improvement is not established. No M569.8, motor-current sensing or load bounding is used. Encoder endpoints are empirical usable intervals, not certified geometric boundaries or strain limits. Hardware behavior and calibration accuracy remain untested.

## Topics

encoder-only, local endpoint, HP3, HP4, pullout, position control, autocalibration, preload, oscillation, max force, Rust Native.

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Local endpoints improve HP3 full collection range and physics work | E24,E29,E31,E34 | Supported for tested datasets | [Final HP3](E34-hp3-net-response-bootstrap/result.json), [previous candidate](E24-hp3-continuous-10x/result.json) |
| Single-step free response is sufficient on HP4 | E28,E30 | Rejected: origin test succeeds, full test winds through oscillating plateau | [Failure log](E30-hp4-local-bootstrap/trial.log) |
| Net-response candidate completes HP4 full collection quickly | E33,E35 | Not demonstrated:16/21 points at240/360wall budgets; no final datasets | [E33 outcome](E33-hp4-net-response-bootstrap/outcome.json), [E35 outcome](E35-hp4-local-final-bootstrap/outcome.json) |
| Requested speed changes numerical endpoint results | E29/E31,E28/E32 | Rejected for these repeats; HP4 repeat predates net correction | [Exact raw-encoder comparisons](endpoint-repeat-checks.json) |
| Encoder endpoint guarantees low strain or improved calibration | None | Not tested | No material or calibration-accuracy validation |

## References

- Implementation commit `899aa4f9`: position endpoints, separate preload tuning, metadata and replay handling.
- [Report](report.md), [notebook](research.md), [comparison plot](local-endpoint-comparison.png), [comparison JSON](endpoint-results.json).
- [HP3 dataset](E34-hp3-net-response-bootstrap/sweeps.json), [HP4 origin dataset](E28-hp4-local-origin/sweeps.json): completed collections; sibling manifests retain world/native artifact identity.
- [Final partial HP4 point journal](E35-hp4-local-final-bootstrap/points.jsonl), [log](E35-hp4-local-final-bootstrap/trial.log): preserved incomplete trial, not a completed dataset.
- [Endpoint audit](endpoint-first-audit.md): rationale; proposed load constraints were superseded by explicit user steering.
- [Force-fit report](force-fit-report.md): preceding duration-dependent F80 approach.
- [Earlier continuous evaluation](continuous-evaluation-report.md): preceding candidate and baseline limitations.

## Implementation and scope

Global tuning now detects only useful drive/free-encoder motion with two-second increasing-force preload probes, captures positions and returns home. No global edge pullout or asymptotic force fit is performed. The legacy force_max_n is a measurement-preload cap (five times measured motion preload, capped by the preload-search ceiling); it is not used to choose the automatic pullout boundary. Existing manual-force collection remains available.

Each local search position-controls the driven motor(s), holds fixed anchors and leaves observation motors at measured low preload. It starts with10mm steps at1200mm/min, reducing steps as free/drive response increases, down to0.5mm. Stops include response ratio5, tracking error above0.5mm, instantaneous loss of free response, or net free response below0.1 times driven travel across eight moves spanning at least20mm. Search bounds are2500mm,300steps,180physical seconds and90wall seconds. Accepted travel requires10mm drive and0.25mm free movement. The drive returns to90% of accepted travel; existing held-drive preload settling and inward position sampling follow. All exits capture motors. Fixed-anchor drift remains an error handled by existing recovery.

No M569.8, current sensing, motor-load monitoring or load-based constraints were added. Position control alone does not prevent high supporting forces. The response thresholds and backoff are empirical; encoders cannot uniquely distinguish rigid-body travel from stretching, and quiet encoders need not imply a quiet body.

Paired-drive fixed-offset discovery explores its own path. Each sweep direction discovers a local interval at actual fixed lengths. Metadata stores samples, settings, endpoint and stop reason. Automatic pullout_force_n is null and pullout_control is position. Python replay no longer converts these datasets into manual force-only pullouts. Legacy replay and explicit user overrides retain their behavior.

## Completed and incomplete production trials

| Trial | Points | Physical seconds | Observed wall seconds | Canonical cable spans (mm) |
| --- | --- | --- | --- | --- |
| E24 HP3 continuous-force candidate |24|1023.390|23.55|380.59,523.87,364.12|
| E34 HP3 final local endpoint |24|686.644|18.90|1005.96,1020.08,995.81|
| E23 HP4 continuous-force candidate |24|936.396|216.39|540.09,554.52,491.54|
| E28 HP4 initial local origin-only |8|269.946|54.01|708.71 (one configuration)|
| E35 HP4 final local full attempt |21 of24|1364.242|360 budget|No final dataset|

E34 uses about33% fewer physics seconds than E24 and larger sampled spans. These runs use automatic size selection, so fixed target lengths differ between methods; this compares whole collection strategies, not identical arcs. E20 original HP3 baseline achieved1323mm spans but used2423.044physics seconds. Local endpoints do not exceed that original baseline range. Canonical cable spans are not Cartesian workspace fractions or calibration-accuracy measurements. Wall times are observations with varying concurrent workload, not a controlled throughput benchmark.

HP4 full completion is unvalidated. E30's first implementation collected12 journaled points before its240wall-second timeout; its second sweep included winding through an oscillating plateau to a2009.67mm time-limited endpoint. Net-response checking stops that observed failure path at758.95mm in E33/E35. E33 then collected16 points before its240wall budget, entering the third configuration. E35 increased only the outer bounds to360wall/1800physical seconds; it collected21 points before timeout. Its final direction selected1893.20mm travel and required further preload settling. Large intervals increase search and subsequent inward movement work. The final attempt is slower than the preceding completed HP4 candidate, with no final three-sweep comparison available.

E29/E31 HP3 requested25x/10x full runs and E34 net-response version match raw encoder arrays for all24points exactly. E28/E32 HP4 requested25x/10x origin-only runs match all eight raw arrays and269.946physical seconds. That HP4 repeat predates net-response correction and does not establish final-version full repeatability.

All new physics uses Rust Native, fixed2ms steps, closed-loop motors, layering disabled and numerical recording without RRD. Production collectors exercise real RRF, three configurations/two directions/four points per direction for full trials,16 noise samples and collection feed1400. The attached supervisor remained source-stale; no attached-world movement was used. Command-context networking probe passed. Fresh isolated worlds preserve manifests, source diffs, native artifact identity, logs and partial journals.

## Verification and decision

123 JavaScript behavior/primitive/CLI tests and three Python replay-metadata tests pass. These cover persistent oscillation, tracking failure, fixed-anchor drift, independent wall bound, useful free travel, endpoint return, new/legacy replay and explicit overrides. Python tests do not run a Python physics backend. Final naming/metadata and every-read fixed guards do not alter accepted movement trajectories relative to the recorded searches.

The result supports continuing the endpoint formulation, especially on HP3. It does not support claiming a reliable faster full replacement across HP3 and HP4. No more live trials are pending. The candidate is committed for review/testing; HP4 needs a tighter useful-interval policy and/or reduced full-collection work before promotion as a demonstrated improvement. Proposed follow-ups are NOT TESTED: back off before the rolling plateau window, test different position-step rates, and limit exploration by marginal useful encoder span rather than waiting for an almost-flat response. No further constant-force F80 fitting is recommended from this evidence.

No final-version HP4 repeat, load replay, hardware test, HP5/Cube experiment or calibration solve was run. No strain or hardware-safety claim follows from these results.
