# 008: Track findings, evidence and their consequences

Outcome: program memory answers real knowledge and uncertainty questions through
queryable typed relations, rather than requiring the manager to reread every
report. Extend 007's store/CLI; use [MANAGER.md](MANAGER.md) for graph semantics.

Represent scoped claim versions, evidence artifacts with hashes/source/config,
derivations, assessments, questions and routes. Add support/challenge/scope-limit,
required-premise, supersession and decision relations. Required-premise dependencies
are acyclic; evidential debate need not be. Multiple premises in a derivation are
AND requirements; distinct sufficient derivations are alternatives. Do not treat
shared data or copied reports as independent confirmation.

Add source/citation-preserving queries for search, support/challenges, upstream
premises, downstream affected conclusions, route rationale and open questions.
Withdrawal/change creates a dated review event, preserves old scope and evaluates
surviving support. Acceptance still requires the assessment/admission path in 010;
this task must not auto-certify imported claims.

Seed a small auditable program from roughly 10–20 useful findings in a few saved
sessions: CM-E1, earlier mixed ranking evidence, an attribution correction and a
pending/incomplete result. The current agenda links the relevant sources. Import
only records actually inspected, preserve missing-artifact limitations and link
to existing reports rather than copying the full corpus.

Done when:

- Queries reconstruct why blanket finite-prediction membership was retired,
  which relevant claims remain unresolved, and whether a result covers HP4 or
  hardware. Answers return sources, scope and assessment state.
- A withdrawn necessary premise flags affected derivations; an alternative valid
  derivation survives. Unrelated claims stay unaffected. Source/config changes
  distinguish new applicability from retracting the historical measurement.
- Cyclic required premises, missing IDs, malformed artifacts and duplicate input
  identities get useful diagnostics; discussion cycles remain representable.
- Focused graph/store/CLI tests and the curated real-data query transcript are
  saved. Report the extent of inspection; no independent science replay is needed.

Deliver usable graph behavior now. A graph database, embeddings, automatic
whole-corpus extraction and visualization are unnecessary for this outcome.

## Delivered

Implemented on ProgramStore as `store.graph()` with append-only versioned claims,
derivations, reported assessments, typed relations and review events. Existing
program/record identity, transactions and revision guards apply. Six new CLI
commands provide mutations and citation-preserving queries. Required premises
are acyclic AND inputs within a derivation; separate derivations are alternatives.
Withdrawal recomputes support and review needs without editing scope; new source/
configuration applicability stays unestablished while old measurements remain.
Unreviewed reports cannot admit claims; admission remains task 010.

The [guide and curated seed](../program-graph/README.md) contain 16 unassessed
findings from nine inspected sources, with eleven CM-E1 compressed/raw hash
pairs checked. The [saved transcript](../program-graph/queries.json) records
twelve successful fresh-process CLI calls including retirement/open-scope queries,
withdrawal with surviving alternative support, unaffected attribution and source
change. Missing raw historical artifacts remain explicit. No numerical replay.

Verification: seven graph tests and ten existing store tests pass, covering
cycle/missing/duplicate/malformed input diagnostics, immutable history, revision
guards, source/config scope, alternative support and full export/import. The
[task report](../../output/research/sessions/2026-10-23-program-evidence-graph/report.md)
and [check log](../../output/research/sessions/2026-10-23-program-evidence-graph/checks.txt)
state the inspection boundary. This completes 008, not the manager release.
