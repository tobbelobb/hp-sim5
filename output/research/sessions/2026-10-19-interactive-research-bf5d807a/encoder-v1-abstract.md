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
