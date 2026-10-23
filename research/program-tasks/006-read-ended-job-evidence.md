# 006: Make research handoffs survive worktree and session turnover

Outcome: another researcher can recover a useful completed/interrupted result
and its necessary inputs without the original process or worktree. This combines
the old tiny job-status card with the actual evidence-portability gap exposed
by the sanity check. Solve the whole handoff, not just a JSON formatting command.

The initial screen's three histories existed only beneath another worktree's
output, while this checkout retained checks.json referring to them. They have
now been copied into the [sanity session](../../output/research/sessions/2026-10-23-program-task-sanity/report.md).
Historical reports also contain old absolute paths. The tooling wish list records
lost multi-hour runs; job.json already persists terminal evidence, whereas live
AutocalJobs.status looks in its in-memory jobs table. Recheck these facts in
current source before choosing an implementation.

Provide and exercise a lightweight durable handoff route in the existing research
archive/status tooling. It should locate the report, scoped decision, necessary
input/evidence references and saved job outcome where present. A small explicit
export/bundle or existing-manifest-based recovery is preferable to a new registry.
Copy only the evidence needed to reproduce the claim; avoid duplicating every RRD,
entire runtime, dependency environment or credential file. Preserve source/input
identity and missing-artifact limitations. Do not regenerate expensive evidence
just to repair a reference.

Demonstrate the result in a relocated temporary fixture or copied real handoff
whose original worktree paths are unavailable. It must distinguish terminal job
states from a stale historical `running` observation and from scientific success.
Do not infer ownership from a reused PID, signal old processes, restore a world
from its final snapshot, or claim calibration accuracy from CLI acceptance.
Missing/corrupt inputs need a useful diagnostic and a partial recoverable record.

Choose a compact implementation boundary and carry it through the relevant
fixtures/tests and a usable command or documented procedure. The existing archive
and autocal-job modules are the starting points. Include hashes and relative
references needed for the selected evidence, not a universal provenance schema.
No scheduler, live-job adoption, checkpoint/resume engine or cancellation redesign.

Budget: queue wall allowance, zero physics/fits/collections. Use saved artifacts,
temporary trees and fake child processes. The real world need not start to prove
that a handoff and an ended-job record remain readable after relocation.
