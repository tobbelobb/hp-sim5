# Existing research worker dispatch with observed ownership and cleanup

Task 009 implements a single-worker adapter over the actual ProgramStore. It
passes an exact assignment/allocation prompt file to the existing research
launcher, records birth-verified ownership, returned paths and partial artifacts,
and supports responsive start/status/cancel. Fifty-seven final adapter/launcher
checks passed, followed by ten passing adapter checks after stricter attempt
import validation. Seventeen earlier store/graph checks are reused. Four real
launcher dry-runs verify Codex/Claude batch/worktree arguments without starting
clients or physics. Actual-launcher interruption checks use fake services.
No scientific experiment, accepted claim, live worker or manager release was
established; missing runtime/client prerequisites remain explicit.

## Implemented behavior

`ProgramStore.workers(root)` returns the retained controller. Start reserves one
active worker across the store, requires a ready assignment and passes its
allocation/constraints/IDs plus unchanged additional instructions in a UTF-8
prompt file. It uses an argv list and inherits existing authentication. It starts
only `hp-sim5-research-agent`, with its supported backend/provider/model, machine,
physics, viewer/recording, worktree and forwarded-client options. The manager
creates no second physics world and no model API dependency.

Attempt records retain the exact prompt hash, contract, argv/settings, manager
and process identities, source revision/dirty flag, start/exit observations,
session and actual returned worktree paths, launcher/runtime session IDs, logs,
raw events, final response and launcher manifests. Status reads bounded metadata
and byte counts rather than growing telemetry. Partial or malformed manifests
remain diagnostics. Historical/imported sessions are inspectable but never
adopted or signalled. Birth/boot/UID/group mismatch prevents signalling a reused
PID. An active historical reservation blocks new dispatch pending task 013.

Cancellation signals the verified launcher first, allowing its existing cleanup
path to retire services. Inspection found two startup ownership gaps: runtime
ownership was assigned after `start()` returned, and Viewer readiness interruption
could skip retiring its local process. Both are fixed with focused regressions.
Cleanup errors are written to `exit.json`; remaining services still receive their
cleanup attempts. A missing exit manifest leaves cleanup unverified. Forced group
termination explicitly reports failed cleanup and unverified detached services.
Polling is required after cancellation; the default 190-second grace allows the
existing bounded runtime shutdown path. Independent timers, restart recovery and
parallel workers belong to tasks 013/014.

A zero launcher exit records `submitted`, with assignment execution `completed`.
Review stays `unreviewed`, knowledge stays `unassessed` and graph claims remain
unchanged. An existing terminal assignment state is preserved during subsequent
cleanup. Task 010 owns fixed scientific submissions and review admission.

## Verification and evidence attribution

[checks.json](checks.json) retains exact commands, results and check versions.
The first adapter run had eight passing tests and one failing assertion: its graph
comparison accidentally included the program revision, which correctly advances
on dispatch. The assertion was corrected to compare knowledge and separately
check the increased revision. This was a test expectation error, not admission.

The first combined check passed 69 tests (19 deselected, 30 subtests), including
10 existing store tests and 7 existing graph tests. After the startup ownership,
cleanup-error and artifact/import changes, the focused adapter/launcher check
passed 57 tests (19 deselected, 17 subtests) in 4.27 seconds. Later stricter import
field validation passed all 10 adapter tests and 17 subtests in 0.82 seconds.
These checks cover 74 distinct tests when the 17 earlier store/graph checks are
included. Passing checks were reused where later edits did not affect them.

The adapter fixtures use real subprocesses and actual store/graph APIs. Existing
launcher regressions use the real launcher, fake coding-client executables and
fake services; they cover prompt/report preservation, inherited auth handling,
provider/model forwarding, sidebar/goal settings, worktree/shared output,
new-session refusal, both clients and runtime/Viewer interruption. Physics-only
imports in that test module and service configuration are now lazy so argument
and lifecycle checks do not require USD/Rerun. No unavailable dependency was
faked. The parent staged released pytest sources locally; their supplied
[provenance](test-tool-provenance.json) is preserved.

[dry-run-records.json](dry-run-records.json) contains four separately retained
actual `hp-sim5-research-agent` invocations: Codex/Claude batch and worktree
dry-runs. Their exact [prompt file](dry-run-prompt.txt) includes literal quotes/
shell characters and zero numerical allocations. The calls ran from `/tmp`,
preserved forwarded resume flags, returned zero, and created no requested session,
worktree or world. They verify launcher argument compatibility, not a client
conversation or live tools. The client model was left at its configured default.

The parent's [prerequisite observations](prerequisites.json) report missing USD,
Rerun, native physics build, Cargo and node_modules, plus an existing Codex login
whose nested execution fails with read-only writable-state initialization. Claude
was reported absent in the dispatch briefing. Installed MCP 1.29.0 is below this
repository's required interface/version. These observations were copied rather
than re-probed; no credentials, HOME/CODEX_HOME or network permissions changed.

## Decision and limits

Task 009 acceptance criteria pass at its fixture/dry-run boundary. Numerical
development budget and actual use were zero fits, physics trials/steps,
collections and movements. Runtime budgets are preserved contracts, not enforced
wall/movement/spend guarantees yet. No physics/motor/cable/scene dynamics code
changed, so the physics delivery gate does not apply.

Same-UID shared output is not adversarial isolation. The controller must remain
alive and be polled; it deliberately does not adopt saved processes or continue
an unfinished conversation after restart. Program exports include attempts but
cannot admit scientific knowledge. Task 010 is the next eligible development
task. Production client/tool/review/graph/replanning evidence is still mandatory
in task 016 before manager readiness can be claimed.

Sources: [adapter and lifecycle guide](../../../../research/program-workers.md),
[adapter tests](../../../../tests/python/test_research_program_workers.py),
[actual launcher regressions](../../../../tests/python/test_research_agent.py),
[manager contract](../../../../research/program-tasks/MANAGER.md), and the preceding
[typed graph verification](../2026-10-23-program-evidence-graph/report.md).
