# 010: Review fixed submissions before admitting knowledge

Outcome: a completed worker investigation produces a versioned submission that
a separate review assignment can assess; only the controller can apply a scoped
assessment to the graph. Use 008/009 and the existing evidence-first reviewer.

Capture the claimed finding, source/base/candidate revisions, actual diff when
present, configuration/input identities, report/abstract, selected evidence and
producer checks. Snapshot the selected submission under a distinct ID/hash and
recheck its integrity before assessment/admission. Reuse explicit handoff export
where it fits; do not bundle environments, credentials or every RRD.

Dispatch a separate reviewer context with the exact target and relevant evidence.
Reuse the existing coding-client/auth integration; a saved-evidence review need
not start a physics world or introduce another model API.
Keep findings (supported/rejected/mixed/inconclusive/not tested), execution state,
practical value and code readiness separate. Store reviewer identity, contract
version, actual checks, verdict/reason, limitations and any unresolved discrepancy.
Reading producer measurements is inspected evidence; independent numerical
reproduction is a separate observation and is required only for a named gap.

Done when:

- A worker cannot use its submission fields or exit code to self-admit a claim.
  A review applies only to its fixed claim/evidence/candidate versions. Changed
  bytes/candidate require a new submission or explicit invalidation, not reuse
  of a stale favorable verdict.
- Known fixtures distinguish a valid scoped negative finding, a lower-fit-cost
  overclaim, missing evidence and an actual physics patch missing its required
  stability gate. They yield usable keep/revise/block decisions.
- Graph queries show pending versus reviewed evidence and withdrawal history.
  A report-only negative outcome can close useful work without a code patch.
- Tests document the actual controller/reviewer/worker write boundary. Same-UID
  shared worktrees are not described as protected evaluation or adversarial
  isolation; do not invent a new deployment requirement to hide that limitation.

Use saved data and lifecycle fixtures; do not reproduce numerical experiments by
default. Real independent review dispatch is exercised in the final live pilot.
