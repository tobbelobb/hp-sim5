# Existing-login cloud research pilot

Source revision: `8acde0c21d5b9db8893115e3e79d82d9e41f98ca`.

The standalone Codex CLI could not reuse this task's supplied authentication.
An explicit local client transport connected the existing research manager,
controller and launcher to real authenticated cloud subagents. No additional
login or model API key was supplied. The parent cloud chat handled dispatch and
completion acknowledgements.

## Actual outcomes

- Two managed investigations completed, each followed by a separate cloud review
  and a `reviewed/keep` admission. Both findings have `report_only` code readiness.
- Production worker `attempt-d35a0ef8b1b54fc4a14bc9a1ba28aa9f` ran HP3 experiment
  `8f977de942c64d7391f63c3198ba0c9d` through the real launcher-attached MCP. It ran
  100 native steps at 2 ms and verified all 101 telemetry and RRD samples, a
  readable blueprint and 1,494,557 RRD bytes.
- Saved-evidence worker `attempt-6aabde30a2df4f20a0e259cc25c52b4b` did no new
  numerical work. Its separately reviewed finding limits the evidence to small
  tool/runtime checks. It establishes neither calibration improvement nor an
  unattended research manager release.
- Controlled worker `attempt-e7bb4a742d144b1eaf33c223c8428d23` did no numerical
  work. Its actual cloud actor reached `interrupted`, the controller recorded
  `cancelled`, and the launcher returned 130 with `launcher_finally_completed`.
- A new manager host and a fresh authenticated cloud manager context reconstructed
  two admissions, three historical attempts and zero active workers. No physics
  world or model conversation restoration was claimed.

Two parent MCP probes preceded the managed pilot. Each ran 100 HP3 steps; one
wrote an RRD. Aggregate new physics across parent probes and the managed worker
was 300 steps and 0.6 simulated seconds. There were no fits, collections or
hardware measurements.

## Evidence layout

`submissions/` contains the original fixed manifests and all selected assets,
including the managed native recording and telemetry. Their hashes still match
the manifests. Original absolute source paths remain provenance; resolve the
assets locally by their IDs in each submission directory.

`reviews/` retains all three actual review responses. The first response was
rejected because it cited `candidate.diff` as an asset ID. That metadata file
was not a selected evidence asset. The failed response was preserved, and a
separately funded genuine correction passed through the unchanged guard.
`admissions/` contains the two resulting admissions.

`parent-probes/` retains the two earlier native experiments and recording.
`checks/` contains the focused-test JUnit, cancellation observation, fresh-host
reconstruction and fresh-cloud-manager report. `inventory.json` records sizes
and SHA-256 hashes for the captured evidence files.

`program-export.json` preserves the program's graph, assignments, decisions and
review history. It is an audit export, not an executable live store. Archived
process and actor IDs must never authorize signals or redispatch. Private live
connection descriptors, control tokens, SQLite client state and dependency caches
are outside this archive.

## Failures and boundaries

The focused program suite produced 106 passes and one reproducible failure in
`test_unopened_program_saved_attempt_is_not_adopted_or_signalled`. Its implicit
allocation allows only 0.6 seconds beyond a one-minute worker and five-minute
review. Intervening recovery calls exhaust that margin. The pilot used explicit
allocations with startup margin and did not change the assertion or controller.

The old review allocation also rejected both retry and further funding. It was
ended with no other active work before a new bounded review handoff was funded.
Those guard refusals were not bypassed.

The transport requires an active parent cloud chat with authenticated subagent
tools. It is not a background model daemon after that chat closes. Reviewer
read-only behavior is instructed, with fixed-evidence integrity checks, rather
than a separate OS read-only sandbox. Only single-worker production was tested.
This pilot does not complete the original task 016 release contract.

The live pilot used the original adapter preserved inside its fixed submission.
The versioned adapter in `scripts/cloud/` relocates that implementation and adds
request-ID validation. Its process/transport checks are separate from this live
evidence. See [cloud research setup](../../../scripts/cloud/RESEARCH.md).
