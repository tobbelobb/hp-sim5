# 015: Keep research acceptance separate from tested code integration

Outcome: reviewed worker patches can become a reviewable combined candidate
without silently changing the human's main branch or treating favorable science
as passing engineering checks. Build on 010/013/014 and existing worktree tooling.

Record candidate base/head/diff and source/input identities on submission.
Prepare integration in a separate worktree/branch, detect conflicts and stale
bases, combine compatible candidates explicitly, and run the checks relevant
to the final combined revision. A no-patch negative finding remains useful
without an integration task. Tool/evaluator changes follow the same review path
and version their contracts instead of retroactively upgrading old claims.

Done when:

- Conflicting patches, changed-after-review candidates and individually passing
  patches whose combination fails checks get explicit outcomes. Reviewed findings
  remain in the graph even when a patch is discarded or integration is blocked.
- Test records identify the exact combined revision/configuration. Physics/motor/
  transport/dynamics changes require the real `npm run test:physics` gate;
  mocks, startup, parity or extra-substep success cannot replace it. Test fixtures
  can verify missing-gate rejection without editing production physics.
- The manager can explain claim acceptance versus patch readiness and link the
  candidate/diff/checks from chat. No automatic merge/push occurs without the
  human's existing authorization; preparing a candidate needs no extra approval.
- Fixture branches exercise compatible, conflicting, stale and failing-combination
  cases with focused meaningful checks. Reuse passing scientific evidence when
  unaffected, and name why any new empirical check is necessary.

Implement the path through candidate preparation and checks, not a full hosting
or release service. Preserve all unrelated human work and failed candidates.

Implemented 2026-10-24. The [integration guide](../program-integration.md) documents
manager prepare/check/status/release/recovery operations, exact sealed sources,
isolated explicit-base worktrees, polled checks and durable exclusive capacity.
Version-2 review keeps scientific admission separate from engineering blockers;
historical version-1 semantics remain unchanged. Reviewer requirements and actual
combined physics delivery gates cannot be cleared by unrelated checks.

Verification uses real Git/binary worktrees and finite child-process fixtures.
The [retained record](../../output/research/sessions/2026-10-24-tested-integration/report.md)
contains exact commands, passing/failing outputs, immutable receipts, fixture Git
bundles, hashes and limits. Affected fixes passed 61 cases in 11.47 seconds;
unchanged passing cases and 34 subtests from the broader run are reused, without
summing duplicates. No actual physics gate or live coding-client review is claimed.
Development allocation was 90 minutes with zero numerical work. Task 016 remains
the actual manager release gate.
