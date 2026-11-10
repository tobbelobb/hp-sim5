# Trial-004 upper-sweep motor slipping

Objective: diagnose and fix the two squiggly upper sweeps, and compare fits against known simulator geometry.
Accepted steering: user suspects lost motor steps in the sweeps after full-auto iterations 1 and 4. Read finalized trial-004 artifacts. Do not modify the user browser or live services.
Additional steering: real machines cannot supply simulator cable tensions. Runtime protection must use existing encoders and commanded values only. Simulator force measurements are diagnostic evidence, never a required collector input.

Hypotheses:
- E1: held B changes during these sweeps (recorded commands, encoders, dataset).
- E2: excessive default forces exceed held B torque (isolated full command replay in production JS physics).
- E3: lower collection forces preserve fixed B/D and clean geometry (own RRF/production JS collector, both directions, 20 points).
- E4: patched production collector completes those sweeps and rejects stable slipped fixed encoders (integration and unit regressions).
- E5: fits 4/5 are closer to known geometry than selected fit 1 (existing regression parser and ground truth).

Movement/compute bounds: one replay to 3,200 simulation seconds; isolated per-sweep collector experiments with 300-second wall subprocess limits. New local RRF/physics ports only, no user UI or firmware connections. Preserve baseline failure evidence and datasets. No complete numerical optimization rerun required to diagnose the collector.

E1 supported: sweeps 004 and 007 have B ranges 193.04 and 200 degrees; clean bootstrap fixed encoders span about 0.2 degrees. Collector repeatedly corrects B by 30-132 mm between sampled points.
E2 supported: production JS replay at sim=2796s has B=253.37 degrees versus target .225, 35 magnetic detent cycles. B branch tensions 7.58+5.85 N imply about .526 Nm at .03918 m, exceeding .5 Nm holding torque. Torque/free-mode transitions make accumulated diagnostic counts unreliable in other parts of the recording; encoder/target divergence while B is held is the relevant evidence.
E3 supported: original collector completes with B range 193.03 degrees and fixed B metadata=60.319 mm instead of zero. Setting pull/sensor forces to 1.13636 N completes both directions with B range .16 degrees and D .14 degrees.
E5 supported: existing regression true-anchor error sum drops from 315.77 mm in fit 1 to 101.74 in fit 4 and 73.91 in fit 5. The fit score chooses fit 1 despite this; optimizer ranking is not changed in this task.

E4 supported: final policy caps pullout at 20 times start force and measurement at five times start force, both bounded by the tuning maximum. Both upper configurations complete 20 points with B spans .50 and .18 degrees. Bootstrap completes all three configurations and six directions, 60 points, with fixed spans at most .18 degrees. A deliberate high measurement override is rejected at 8.010 degrees drift before a final dataset is saved, and all four position-hold commands are acknowledged. Unit/integration regressions pass: 18 suites, 89 tests.

E6 mixed: using five or ten times start force for pullout keeps the upper motors held but fails the bootstrap's 120-second active travel deadline. Those failed trials are preserved. Twenty times start force completes both upper sweeps and the bootstrap. This is an empirical default for the tested geometry, not a guarantee that every machine's commanded force is safe.

Status: complete. No hardware measurements, browser retry, full optimization rerun or replacement reference capture. The original bad data remains available. Next decision for the user: capture a fresh visual run with the corrected collector, then separately investigate score ranking using clean data. Do not infer improved calibration accuracy from these collector checks.
