# Durable research handoff, task 006

Objective: retain the report, scoped decision, necessary selected inputs and
saved job outcomes after worktree/session turnover. User instruction: read the
program prompt and complete one task. Selected the lowest-priority-number ready
task (006, priority 50) after inspecting the clean tree and current archive/job
implementation. Task 005 is outside this invocation.

Workspace preparation: the initial checkout was 79e53cbb and lacked the requested
program queue. Found the prompt on origin/main, fetched through the configured
network and fast-forwarded the clean work branch to 166f0352. No user changes
were present. The local interpreter was absent; created .venv and installed
pytest 9.1.1 and pytest-asyncio 1.4.0 for focused tests.

Source observations: archive.py catalogs abstracts and retains links but does not
copy evidence. AutocalJobs.start writes running job.json; run finally writes the
terminal self.status result. AutocalJobs.status rejects IDs outside its in-memory
table. Terminal states are accepted, failed, cancelled and budget_exhausted.
Historical running/cancelling records cannot establish current liveness. The
old tooling wish list and task 004's copied inputs support a lightweight offline
handoff, without changing ownership/cancellation or starting a world.

H1: an explicit file selection plus portable manifest can preserve the useful
scoped result and byte identities when original paths no longer exist.
H2: historical job evidence can retain lifecycle outcomes without asserting
current liveness or scientific success.

Predeclared budget: zero physics runs/steps, fresh fits, numerical selection or
prediction evaluations, collections and movements. At most one real CM-E1
packaging/relocation demonstration; focused synthetic fixture tests for missing,
corrupt and stale evidence. No supervisor, browser, Viewer, doctor or preflight.
Queue wall ceiling: 45 minutes. Relevant delivery checks are focused archive/CLI
tests, evidence/link inspection, catalog generation and diff formatting. No
physics/motor/cable/scene dynamics code is changing.

Implementation boundary: extend archive.py and add scripts/research_handoff.py
with export/show/job operations. Always copy abstract/report/notebook; select
only additional claim-specific evidence. Record source revision, scope, decision,
original-to-relative file mapping, expected/copied hashes and limitations. Leave
documents unchanged. Show reads only the relocated directory, preserves partial
results and reports integrity separately from science. Saved job reading never
consults a PID or adopts/signals a process. Document operator responsibility for
evidence selection, old absolute links and external code/runtime dependencies.

Next: exercise one copied CM-E1 handoff after deleting its temporary source tree,
review the source and results, then record the decision and queue completion.

Completed DH-E1 once: nineteen selected files, eleven compressed inputs,
4,509,141 bytes; every temporary source path absent at recovery and every copied
hash verified. The scoped CM-E1 decision survived. No numerical analysis was
rerun. Raw export/recovery snapshots and commands/results are saved beside this
notebook; temporary source/bundle trees were retired. The initial context
limitation overstated missing prior reports; inspection found October 15 present
and the initial screen absent. The report explains this correction and the script
uses the corrected declaration for future demonstrations.

DH-T1: the first test collection found a missing closing list bracket; fixed it.
The focused archive/handoff run then passed all 21 tests, including CLI
relocation, partial/corrupt evidence, lifecycle distinctions and no PID signalling.
Reviewed implementation after that run: no lifecycle ownership or dynamics
changes, only offline archive operations. No additional numerical or duplicate
producer checks were needed. Operator documentation and source identities are
recorded with the result. H1/H2 supported within the tested portability/fixture
scope; no calibration or hardware claim. Mark task 006 done after the final
catalog/link/format checks. Next decision: task 005's living research agenda.

Catalog regeneration exposed absent prior untracked sessions: a full rebuild
deleted their previously committed catalog-only abstracts. Retained the earlier
entries and inserted the newly generated session entry to preserve that history;
no missing report was regenerated or inferred. This is a scoped catalog update,
without changing catalog parsing or adding an evidence registry.
