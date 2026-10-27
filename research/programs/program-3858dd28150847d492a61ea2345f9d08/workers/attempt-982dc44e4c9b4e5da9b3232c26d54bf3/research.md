# Paired native HP3 bootstrap collection

Status: in progress. No native collection outcome yet.

Objective: test whether the earlier analytical core-fit fragility survives ordinary native collection and production fitting/recovery.

NP-E01 freezes two session-local HP3 scenes differing only cable stiffness 20000 versus8000N, all halfWidth0, fixed30mm radii, buildup0, layeringoff. Three production representative configs and10 points per direction target60points/world. Matched feed1400, speedup25, noise16samples at40Hz, manual forces .01/.05/.5N and sensor preload1N, recording none. The sensor setting and forces follow research/native-collection.md before collection. Earlier draft .01N preload was amended before any movement after parent identified documented stationary-sensor failure.

Hard budget: two production collections, two million total physics steps, eight high-level fit stages,30minutes ending09:01UTC. No failed-world retries or tuning. Finish experiments by08:56UTC.

Acceptance: complete3-sweep/60point paired datasets, finite sensor records, actual native engine and baked stiffness/halfWidth/config verified. Failure blocks pair inference and preserves partial evidence. Fitting uses ordinary lbfgsb/trf stages and production restoration recovery only if underconstrained and within eight calls. Known scene-home effective anchors evaluated offline only. No one-click stopping or collection savings claim.

Previous evidence: ../attempt-3b2e360ec1834eee9bab4e06f6f880fd/handoff.json and ../2026-10-27-autocal-parameter-variation-manager/review-first.json. Do not rerun analytical matrix.

Preflight: authenticated research_attach.py MCP connection, runtime_status and research_preflight retained. Command socket probe succeeded. Attached stock layered supervisor world is not used for collection. Standalone HeadlessSimulation owns each copied scene before entry and correct manifest.scene.

Reference caveat: softer cables can change settled encoder-zero pose and rotation. Standalone status lacks effector pose; conventional scene-home truth is not exact pose-relative truth. Never use pose or known geometry for fitting.

Next decision: inspect complete counts or preserve failure, then fit only complete data.

## NP-E01 environment startup failure

Both collections aborted at the first physics update under the unsourced command shell's Node24.19.0. Physics logs show napi_detach_arraybuffer fatal V8 abort, matching scripts/cloud/README.md's documented Node24 cloud incompatibility. Each manifest reports total_steps0, collector exit1 and physics returncode-6. Actual engine/input construction succeeded but no measurements exist. This is a command-environment setup fault, not a stiffness outcome. Original files remain at nominal/ and soft/ unchanged.

Corrected driver asserts sourced Node23.11.1 and writes a new pinned/ directory. Awaiting controller-recorded bounded replacement allocation before running. No source/build changes.

Controller decision decision-98f76c05c2fc474abde86f7fe3e331f0 authorizes exactly two replacement launches under pinnedNode23, four total startups including two0-step failures, two complete collections max,2M actual steps total,8 fit stages max and original09:01UTC deadline. No native rebuild or physics source changes.


## Manager closure after interruption

The actor hit its model usage limit after completing two pinned collections and eight fit-stage calls. No worker final message or handoff was written. The owner deadline cancelled the launcher at09:01UTC, with confirmed cleanup. Parent later acknowledged actual actor cancellation and wrote an explicitly manager-authored report in ../2026-10-27-autocal-parameter-variation-manager/native-report.md. No further fitting/physics ran.
