# 005: Follow explicit dependencies only if ordinary search proves awkward

Activate only when task 004 identifies a repeated, concrete question such as
"which decisions relied on this suspect measurement?" that current lookup handles
poorly. Otherwise park this task.
Budget: zero simulations, fresh fits or collections.

Read the pilot, [archive.py](../../src/python/hp_sim5_research/archive.py), and
[the graph proposal](../../research-program-design.md#make-the-claim-ledger-an-evidence-graph).

Add the smallest useful explicit relation to the existing Markdown evidence
records, for example a stable link labeled `Relies on` or `Challenges`. Use it
for the pilot's few findings. If a lookup command is actually needed, extend
`scripts/research_index.py` and the existing archive module to follow just these
relations and show the relevant statements, scope, review extent and links.
Keep plain search working. Treat ordinary citations as references, not automatic
logical dependencies or independently verified premises.

Done when the pilot's specific question can be answered from explicit records,
missing targets remain visible, and repeated/cyclic links cannot cause unbounded
traversal. A withdrawn input should identify affected reasoning for review;
alternate evidence may still support the claim. No automatic confidence arithmetic.

Use temporary Markdown fixtures and focused archive tests for changed lookup
behavior. Keep traversal bounded and use files/IDs already present. No graph
database, vector service, UI, whole-corpus extraction, immutable-store rollout or
general graph framework belongs in this task. Split a larger demonstrated need
into a later card rather than expanding this one.
