# Roller propagation investigation

User asks to confirm whether slow line propagation across HP4 rollers causes sluggishness and suggest inexpensive alternatives to whole-world substeps. Also investigate reference-006's remaining 30 s timeout and inspect the independently started two-substep reference-007 run. Do not interfere with user processes.

Budget: offline, fixed inputs, no recording, 30 s JavaScript reference runs for baseline, exact frictionless redistribution, two substeps, and stiffness-only motor counterfactual. Two 3 s coefficient probes; one baseline rerun to restore the 30 s artifact after the short probe reused its filename. No full-autocal, live browser or physics changes. Previous evidence: ../2026-10-16-hp4-substep-options/report.md.

E01: exact redistribution produces only ~1% more travel than baseline. E02: suppressing dt²*K/I while keeping implicit damping produces ~10x more travel. Initial zero-K trial bypassed the damping branch too; preserved but excluded. E03: four redistribution passes per macro tick at both one and two substeps. E04: K doubles with solver iteration count,36->72 Nm/rad. History confirms earlier global-path and non-slip coupling experiments and identifies May6 implicit denominator introduction.

All hypotheses and limitations in report. No production source edits, no tests required for diagnostic-only artifacts. Source status clean. Recommendations are proposals, not deployed or energy-validated fixes. No ongoing experiment remains.
