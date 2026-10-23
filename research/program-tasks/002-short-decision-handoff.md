# 002: Put a short decision handoff in the existing abstract

Benefit: the next researcher sees what to do with a finding without opening a
new database or maintaining a second report.
Budget: zero simulations, fresh fits or collections.

Read [the abstract template](../abstract-template.md),
[archive.py](../../src/python/hp_sim5_research/archive.py), and
[its tests](../../tests/python/test_research_archive.py).

Extend the existing abstract convention with a compact decision handoff: the
finding's applicable machine/data/model scope, the extent of review, the current
decision, and the next cheapest observation that could change it. Cite the
existing hypothesis/evidence entries instead of restating the whole result.
Aim for a few lines, not a second notebook. Negative or inconclusive outcomes
can have a useful next decision; missing evidence stays explicit.

Update `initial_abstract` only as needed to keep new sessions honest and useful.
The existing index includes arbitrary abstract Markdown already; preserve that
simple behavior. Do not migrate all old abstracts, add a mandatory claim schema,
parse new fields into another store, or automatically assign acceptance status.

Done when:

- A new session clearly has no result or research decision yet.
- A completed example can communicate scope, review extent and next decision
  briefly, with links to the original evidence.
- The current index/search displays those lines and preserves relative links;
  legacy abstracts still work without edits.

Run `.venv/bin/python -m pytest -q tests/python/test_research_archive.py` once if
archive behavior changes. Add a focused behavioral case only if needed to expose
a real gap. For template-only edits, a temporary rendered catalog is sufficient.

## Completed handoff example

This example uses saved simulation evidence; no new fit or collection was run.

- Scope: [H1](../autocal-filtering-lane02-integration.md#bounded-investigations),
  the recorded HP3 layered fixture's individual fit and final consensus;
  broader solver equivalence is untested.
- Review: task 002 inspected the [comparison](../../output/research/sessions/2026-10-22-autocal-filtering-integration/summary-comparison.json),
  [revision ancestry](../../output/research/sessions/2026-10-22-autocal-filtering-integration/fallback-history.json)
  and [H1 analysis](../autocal-filtering-lane02-integration.md#the-layered-reference-comparison).
  No independent numerical reproduction.
- Decision: retain H1's rejection of the inferred fallback regression; correct
  the original report's comparison of an individual fit with final consensus.
- Next: inspect any saved comparison showing a gap at the same output stage,
  with matched inputs and attributed revisions, before reopening H1. None was
  identified here; correcting this attribution needs no new fit.
