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

## Force criterion and measured result

The former tuner searched for two successive relative travel changes within 10%, then chose the lower force of the last plateau pair. It did not require settled trials. In the reference, every edge probe after 0.2273 N remained unstalled; it selected 12.8849113355 N and extrapolated 1785.461 degrees despite observing at most 1249.35 degrees. This measures finite-time excursion, not equilibrium reachability.

The new definition tracks incremental compliance Δtravel/Δforce. Two consecutive intervals below 50% of the greatest observed compliance select the force before that decline. The 50% threshold is an explicit conservative operational choice, not a derived physical boundary. A plateau/reversal or fixed-anchor drift over 1.5 degrees stops ramping. No detected knee yields a marked failed tune and a default maximum bounded by the configured cap and minimum moving force. It never treats a force cap as a measured knee. Each edge trial observes 30 seconds on the selected clock, without ending early on a stall. Trial options now reach runForceTrial; final observed travel is retained rather than a possibly stale stall position. The obsolete logistic fit was removed.

E1 replay selected 2.39574867159 N after ten probes, with the highest probe 4.69566739632 N. The reference required fourteen probes up to 18.0388758697 N. E2 used production headless JS, sys/config_hp3.g, closed-loop motors, no layering, drive anchor 1, held anchor 3, and speed scale 1 so the headless selected clock matches physical simulation seconds. It selected the same 2.39574867159 N and 0.227271514624 N moving threshold. The selected excursion was 535.76 degrees versus 529.61 degrees in the supplied curve. Maximum observed excursion during confirmation was 836.69 degrees. No held-motor drift was observed. These measurements are in [tuning.json](tuning.json) and [service manifest](isolated.headless_1/manifest.json).

The initial E2 attempt [manifest](isolated.headless/manifest.json) failed because sandbox permissions prohibited sockets. It is preserved; the approved isolated retry succeeded. No existing browser or firmware service was commanded.

Historical comparison: sixth_hp3_dataset_no_buildup.full_auto.log selected 7.1716 N and ended at score 640.2; its other saved fit logs also failed the score threshold. second_hp3_dataset.full_auto.log ended at score 0.6, but used a 0.636619 buildup model and global radius fitting, so it is not a matching successful no-buildup reference. These comparisons do not establish timing as the sole cause of the current failed calibration.

## JAX warning reproduction

E3 used the local JAX and jaxlib 0.9.2. A cold explicitly enabled cache emitted zero compatibility warnings; its warm repeat emitted three. Two default fresh processes after removing automatic JAX_COMPILATION_CACHE_DIR and minimum compile-time overrides emitted zero warnings. All four returned 100.0. No native warnings were globally suppressed. Explicit user cache settings remain respected, and in-process JIT caching remains active. Cross-launch compilation can now take longer unless users opt in to disk caching; on this installed version opting in can restore the upstream warning.

## Validation and limitations

E4: 19 collector suites / 95 tests passed, including recorded-curve replay, synthetic asymptotic and linear curves, false knee rejection, reversal and held-motor slip handling. Ten Python JAX/cache/objective tests passed. git diff --check passed. Final source is the uncommitted working tree based on bdc2ac1fa4b92c77df68d374e5947c21e8a5d532; [diff](changes.diff) preserves the implementation and new tests.

Full calibration quality remains untested. The new criterion is finite-duration and will depend on inertia, damping and the chosen observation window; equal simulation-time windows control recorder-throughput effects without asserting equilibrium. The isolated run used no recorder and did not test new manual browser collection. Reload the simulator and use a fresh dataset/output directory for the next reference. Existing data retains its original high-force commands and cannot be repaired by changing the tuner.

Follow-up: [full cache timing investigation](../autocal-jax-cache-20261010/report.md) measured a 43.64% warm-cache saving and restored caching by default with overridable native logging. The earlier opt-in decision described here was provisional and is superseded.
