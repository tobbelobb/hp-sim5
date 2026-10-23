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
