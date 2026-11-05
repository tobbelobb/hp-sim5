# Finish the HP3 autocal robustness program

Continue the existing research program and deliver a supported, usable autocal
improvement with concrete before/after results. The human wants you to finish
the work, not produce another plan or re-summarize known negative findings.
Work autonomously; the interview is finished. Commit and push recoverable work
and checkpoints. No further permission is needed for that. Preserve the human's
new commits and other work.

## First reconcile the repository

The human pushed new commits from elsewhere during the previous turn. Fetched
`origin/main` was `fbd2c744` at handoff. It already contains the recovery commits
through `ca0217c1`, plus newer changes including prompt portfolio triage, manager
role changes, pinned/sequential cloud execution and non-fatal recording failures.
Fetch again, inspect current HEAD, status and upstream, and read current AGENTS.md,
research/AGENTS.md, the current cloud-research-client skill and returned role
instructions. Follow current upstream instructions; the old queued prompt is
historical context, not authority to undo new workflow code. Do not force-push,
reset away the human's commits, or overwrite main with the older research branch.

Latest handoff/setup artifacts are on remote branch
`research/autocal-originals-2026-11-05-0a82df7f` (also mirrored to
`research/autocal-recovery-2026-11-05-0a82df7f`). Base the implementation on current
upstream; import the handoff/artifacts without reverting intervening source
changes. A clean continuation branch/worktree is appropriate. Avoid staging the
many apparent deleted output files in launcher worktrees: output/dependencies
are shared, and that status is not a source change to commit.

## Existing program and honest outcome

Program: `program-0a82df7fd4b34b8181716630492a1af4`.
Read this directory's README.md, the first worker report at
`workers/attempt-939c073f49004eb6949910600200be51/report.md`, and
`continuation-setup/README.md` and `policy-frozen.json`. Use the current
`research/prompts/autocal_parameter_variation_focus.txt` and registered prompt
portfolio; audit remaining work rather than rediscovering completed studies.

The first study tested bounded-preload prototypes in an isolated harness.
Nominal max error was3.841mm default versus3.873mm candidate; matched noise-only
default max was4.349mm. All three completed calibrations had140points and every
anchor below5mm. Four active-guide trials failed before fitting. The prototype
did NOT improve collection and was rejected for production adoption. A constructed
native probe exposed a guide transport/load-model concern, not hardware failure
or a proved root cause. Do not repeat this matrix as if untested or change physics
to make collection pass. No production improvement has been adopted so far.

The exact scoped report-only result has effective admission
`admission-22c5d2de62e841e9bd8b77c2b3ae4811`, finding supported/decision keep,
code readiness report_only. Submission:
`submission-eee989ceef1947c39f94515078da2af2`. Original407files, including raw
journals, all fits, failed trials and recordings, survive in `recovery-originals/`.
The access outage did not delete them. Restore/verify with its restore_session.py.
Do not rerun the already completed review because of its earlier acknowledgement
timeout; actual original and replacement responses plus applied admission are saved.

## Human goals and thresholds

Priorities: fewer calibration failures across machines, then better accuracy,
then lower runtime, then explanation. ANY anchor Euclidean placement error above
20mm is a practical failure. Good performance means EVERY anchor below5mm.
An incomplete collection is also a failure, recorded separately from a bad
accepted geometry. Up to20% extra collection time is acceptable for a substantial
robustness gain. Emphasize friction and noisy encoders. Simulated truth can build
and evaluate plants; it must not choose measurements, forces, thresholds, seeds
or the applied answer. A small simulation matrix does not prove hardware reliability.

## Interrupted continuation: setup exists, calibration results do not

Question record: `question-0e6d4ed320fe44a793f3bc5eb35f9c34`.
Assignment: `assignment-a64d3f287c374d4e8b0e697ee33afe68`.
Cancelled attempt: `attempt-11d12ea8033e4ba291598eb250d6e416`.
Approach: `approach-1ae49e51d47342ae9cc9f973962a14f9`.
Rival: `approach-4efeaafb10e946d68b589da1124f50ad`.
Previous allocation: `runtime_allocation-aaf676ff4d7b4d67824723d28ba8a61b`.

The worker hit an account usage limit before its first calibration collection.
The human then requested handoff. Launcher state is cancelled, returncode130,
cleanup launcher_finally_completed; bridge request
`5cc53a6391bf4a67a4a6437bf7bf783a` is acknowledged cancelled. No live numerical
job is intentionally left running. The old exit file says wall_expired=true as
part of cancellation; do not report that as spending100minutes of research.
The continuation had zero collection jobs and zero fits. A1001-step-per-world
motor probe and preflight/socket outputs exist; inspect them before relying on
physical intervention validity. They are not reviewed calibration results.

Local session, if still present:
`/workspace/hp-sim5/output/research/sessions/attempt-11d12ea8033e4ba291598eb250d6e416`.
Local worktree:
`/workspace/hp-sim5-worktrees/bounded-research-assignment-b6928f45`.
Controller attachment, if still live:
`/workspace/hp-sim5/.research-programs/managers/program-0a82df7fd4b34b8181716630492a1af4/connection.json`.
The program is paused for handoff; resume explicitly after reconciliation.
Inspect controller state, capabilities and runtime readiness; refresh an old
controller if it is running pre-update code. Never adopt/signals historical PIDs.
If local program state is absent, use committed program.json/report.md and exact
IDs as provenance when recreating a bounded continuation under the current tool.
Do not represent that as restoration of an active physics world.

`continuation-setup/` preserves59originalfiles in verified gzip archives, plus
readable candidate.patch, policy-frozen.json, budget ledger and partial reports.
Run its restore_session.py --verify-only or --destination /fresh/path. Saved
harness/default and harness/candidate differ only in measurement multiplier5→4.
The patch has NOT been applied to production. Copied scripts retain absolute
old worktree imports: rebase them carefully to the new checkout, preserving ECS
component class identity. Do not run old prepare.py or mutate original evidence.
Reconcile latest headless/recording changes and hash fresh source/native artifact.

## First candidate to finish testing

Use encoder-tuned sensor measurement force `min(forceMax,4*forceMid)` instead of
current `min(forceMax,5*forceMid)`. This fixed candidate came from earlier
independent force-valley evidence near3.8times midpoint; do not retune its
multiplier using new truth. Keep explicit sensor-force overrides, pullout and
preload forces, physical guards, solver, selection, planning and automatic
stopping unchanged. Inspect every helper call site. The isolated candidate
changes a generic helper: any adopted production default must respect the
HP3-only evidence and avoid silently changing untested machines.

Fresh declared worlds, A/B/C/D order, all guide friction zero, known30mm radii,
zero buildup, layering off, otherwise stock HP3:

| World | Coulomb factors | Stiction factors | Gaussian read sigma | Seed |
| --- | --- | --- | --- | --- |
| nominal | 1,1,1,1 | 1,1,1,1 | .03deg | 4001 |
| development | .5,2,1.5,1 | 2,.5,1,1.5 | .03deg | 4101 |
| evaluation | .5,1.5,2,.75 | 1.5,2,.5,1 | .05deg | 4201 |

Use loaded effective defaults before multiplying (.001Nm Coulomb and .0015Nm
stiction expected). Verify ECS AND packed native fields and a physical torque-mode
response. These hypothetical variations are not hardware-characterized. The
motor law uses smoothed friction, not ideal static breakaway. Run fresh paired
default/candidate nominal and development runs, then the held-out evaluation pair.
Freeze policy before any new primary run and disclose any required harness fix.

Finite proposed continuation:100worker minutes including last10for reports,
15review minutes plus10manager/startup,190s cleanup; max7collections,80fresh fit
calls,28Mtotal physics steps,2000points; per job4Msteps/8000sim seconds, at most6
automatic iterations. Six primary runs; seventh only a justified confirmation.
Reallocate explicitly under the current controller; do not inherit an old deadline
or fake a completed worker response. The human permits a wide budget, so choose
additional finite work only when it can change a concrete decision.

Use .venv/bin/python, native Rust, headless, speedup25, Node23.11.1 through the
current env.sh. Jobs sequential: cgroup has2CPUs. Before long collection run
supervisor research_preflight AND separate command-context research_socket_probe.py.
No manual engine.prepare; inspect after normal first update. Recording none is
appropriate for this numerical matrix, with full command/noise journals and
passive initial/terminal snapshots. Follow current handling of recorder failures.

## Delivery and durability

After EACH completed or failed collection, preserve compact results, input/spec
and source/native hashes, dataset and fit/selection state, candidate diff and raw
journals. Commit and PUSH before starting the next long run. Do not rely on a
single end-of-turn save. Exclude live connection/auth/control files. Hash-verified
gzip archives and readable small artifacts are already the recovery convention.
The saved checkpoint helper scripts are examples; adapt their hardcoded state,
not missing /tmp files. Keep failures and interrupted work.

Report all four per-anchor Euclidean errors, maximum, RMSE definition, final
applied versus intermediate candidate results, accepted/stopped or aborted,
points/sweeps/fits, steps/simulated time and wall time with host-load caveats.
Adoption needs meaningful applied-accuracy improvement that transfers to the
evaluation pair, no new failure and at most1.20times paired baseline collection
effort. Do not sell an intermediate-fit improvement as a final-anchor improvement.
Do not claim fewer >20mm failures if the baseline never has one.

If supported, implement the minimal scope-safe production change, run relevant
collector/autocal checks, obtain the required fixed-evidence review under current
workflow, integrate the reviewed candidate against explicit current upstream,
and commit/push the concrete code and results. If this candidate fails, preserve
the counterexample and make a reasoned next decision; do not adopt it to satisfy
an improvement claim. Avoid another broad infrastructure detour or an ending
that merely offers to continue while useful authorized work remains.

Maintain research.md, abstract.md, report.md and the short results README, update
the original prompt/portfolio to remove settled questions, regenerate the catalog,
finish cleanup and save. Final answer must clearly state what changed in autocal,
the measured before/after result, remaining limits and links to pushed commits.
