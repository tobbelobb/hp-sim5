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
