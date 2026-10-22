# HP3 comfortable force tuning and PJRT cache warnings

Status: complete.

## Abstract

This investigation tested why a no-buildup HP3 reference run selected 12.8849 N and why autocal printed native PJRT compatibility warnings. Offline replay of the saved force curve and one fresh isolated headless HP3 tuning run both selected 2.3957 N using a decline in travel gained per added newton. The minimum moving force remained 0.2273 N. The fresh run used 976.542 simulated seconds and 54.06 wall seconds. A separate JAX 0.9.2 reproduction emitted three warnings on a warm disk cache, versus none with the revised default; numerical output was identical. Autocal now leaves disk caching opt-in, uses equal-duration edge probes, and removes the logistic extrapolation and force-cap success fallback. Ninety-five collector tests and ten JAX tests passed. These are simulated tuning and software checks; no new full calibration or hardware accuracy improvement was measured.

## Topics

HP3, autocalibration, force autotuning, comfortable workspace, incremental compliance, simulation timing, JAX, PJRT, persistent cache.

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| A plateau-seeking definition permits excessive maximum force | E1, E2 | Supported for the measured curve | [Replay](reference-replay.json), [fresh tuning](tuning.json) |
| Recording delays alone explain the changed force curve | E2 | Inconclusive; a fresh unrecorded headless run reproduced it closely | [Tuning log](tuning.log) |
| Persistent-cache hits cause the exact native warning | E3 | Supported | [Reproduction](jax-repro.json) |
| Revised default and force logic pass regressions | E4 | Supported for tested cases | [Checks](checks.md) |
| Lower force improves full calibration accuracy | none | Not tested | User will repeat manual collection |

## References

- [Report](report.md): decisions, results and limitations.
- [Notebook](research.md): objective, budget and steering.
- [Supplied reference](../../extended-autocal/no-buildup-reference-001/sweeps.json): original force curve.
- [Recorder manifest](../../extended-autocal/no-buildup-reference-001/hangprinter-20261010T135406.255382Z-2af430aa.json): supplied recording provenance. The large RRD was preserved and was not decoded for this focused fix.
- [JAX issue #36294](https://github.com/jax-ml/jax/issues/36294): upstream report of native warning spam on persistent-cache hits.

Follow-up: [full cache timing investigation](../autocal-jax-cache-20261010/report.md) measured a 43.64% warm-cache saving and restored caching by default with overridable native logging. The earlier opt-in decision described here was provisional and is superseded.
