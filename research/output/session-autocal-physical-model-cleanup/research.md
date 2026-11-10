# Integration notebook

Date: 2026-10-22. Status: complete.

Objective: integrate lane 05's supported physical-model cleanup and make a small
independent investigation where useful. User authorized implementation and small
investigations; no request to rerun the lane matrix or perform collection.

Baseline: HEAD `00375083` (lane 04 integration). Initial tracked checkout clean.
Read lane 05's report, proposed changes and probe outputs in worktree
`/home/torbjorn/repos/hp-sim5-worktrees/research-eaee3cb5`; copied selected evidence
and frozen artifacts to this integration's output session with origin hashes.

Decisions: remove ellipse/autocal flex correction and both switches/plumbing,
retain separate point/plotting consumers, fail explicit zero-tension requests when
projection/fields are unavailable, retain origin and plane policies. Preserve
flex-disabled old replay artifacts; reject old enabled policies and new obsolete
overrides explicitly. No changes to filtering/stopping/solver policy.

Experiment budget: zero new optimizer tuning fits, zero collection/movement or
hardware trials. Two full no-collection reference replays for verification;
72 fit-free objective comparisons across 12 fixtures. Saved-geometry probe of four
production artifacts with sweep filtering on/off. First raw-length probe was
misconfigured; retained it and reran with modeled lengths. No timings accepted as
performance evidence.

C05-01 through C05-05 are recorded in [report.md](report.md). Full Python suite
passed after socket access and updating one obsolete CLI test; later migration
checks passed on affected modules. Collector checks passed. Both reference replays
matched exactly. The sentinel probe found all-invalid geometry and flat cost in
two saved production failures even with rejection disabled. It does not reconstruct
the failure trajectory or all lane-05 alternative fits.

Next decision: solver trajectory diagnosis using feasible/cold starts, logging
geometry validity before testing different rejection settings. No automatic default
change is justified by saved failure states alone.

Evidence directory: `output/research/sessions/2026-10-22-autocal-physical-model-cleanup/`.
[Final report](report.md), [verification source](verify.py), and the report's linked
raw results, logs, manifest and source diff provide provenance. Research index
regenerated once by the integrator after this turn's work.
