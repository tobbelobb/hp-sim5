# Strategy in the manager conversation

The manager compares approaches and records its reasoning through the existing
[chat tools](program-manager.md). It starts no separate strategist or physics
world. `overview` returns `strategy`, `next_allocation` and a concise cited
`brief`, also saved as `brief.md` in the manager session. Another chat reconstructs
these from the store. `query` with `rationale` follows decisions through strategy
records, claims, separate reviews, fixed submissions and artifact sources.

## Record a question, compare routes, fund a discriminator

Use `approach` to name each route, then `route_plan` to record its hypothesis,
exact scope, priority reason, practical tradeoffs, objections, estimated cost,
uncertainty, novelty, expected decision value, discriminating observation and
reopening condition. Supply `dependency_ids` for supporting claims or artifacts
and `citation_ids` for other records actually used. A lower integer `priority`
allocates first. Each version is append-only; append order chooses the current
plan even if timestamps tie or the clock moves backwards.

`commission` creates a ready assignment and its fixed investigation contract.
It requires a recorded plan for at least two distinct rival approaches,
`explanations` with their competing explanations, and `outcomes` containing the
observations and decisions that distinguish them. Each outcome has a `finding`,
`observation` and `actions` list. An action names `approach_id`, `target`, `reason`,
exact funded `scope` and `reopening_condition`. Broader approaches need their own
plans; a cheap failed method cannot silently retire a wider hypothesis.

For example, a narrow saved-data scout can use:

```json
{
  "approach_id": "common-folds",
  "objective": "Inspect four named sources for compatible common unseen sweeps",
  "kind": "scouting",
  "allocation": {"wall_minutes": 12, "fresh_fits": 0, "collections": 0},
  "explanations": [
    {"approach_id": "common-folds", "explanation": "Training overlap hides a useful selector difference"},
    {"approach_id": "model-adequacy", "explanation": "Model inadequacy dominates even on a fair fold"}
  ],
  "outcomes": {
    "found": {"observation": "A compatible common unseen sweep exists", "finding": "supported", "actions": []},
    "absent": {"observation": "None exists in the named sources", "finding": "inconclusive", "actions": []},
    "failed": {"observation": "A reader failure prevents inspection", "finding": "tool_failure", "actions": []}
  },
  "execution": {"source_limit": 4}
}
```

The route plans must already exist. Full runnable calls, including state-changing
outcomes, are in the [saved cases](program-graph/strategy_cases.py) and
[tests](../tests/python/test_research_program_strategy.py). `kind` also accepts
`experiment`, `literature` and `tooling`. All use `worker_start` and the existing
`hp-sim5-research-agent`; the worker prompt receives the investigation, route
plan and accepted human inputs. Funding itself starts no child.

Every allocation requires a positive finite `wall_minutes`; other explicit
counters are finite and nonnegative. The zero numerical counters above belong
to that saved-source scout. Fresh fits, native/headless physics and simulated
collection remain available. Choose workload-specific limits using the
[recording costs](autocal-recording-performance.md), rather than copying the
development queue's caps. A funded collection defaults its execution guidance
to simulated autocal speedup 25 and event recording. Override guidance only with
settings supported by the actual workload. Headless physics already runs at full
speed. Allocations remain recorded worker contracts here; task 013 owns runtime
enforcement and recovery.

## Interpret results without converting execution into science

After the worker ends, submit its fixed scoped evidence, run a separate review
and apply that review through the existing [admission tools](program-review.md).
Call `strategy_result` with the assignment, a predeclared outcome, reason and
cited admission or review. Scientific outcomes require an effective `keep`
admission for that exact assignment and matching finding. Unrelated findings,
withheld reviews, invalidated submissions and withdrawn evidence cannot apply
automatic route actions. Reviewed partial evidence from a failed attempt remains
usable within its fixed scope. Review-ID citations retain the submission,
admission, claim scope and artifact sources in rationale queries.

`unexpected` is a strategy classification, separate from the review finding.
Its outcome must name `review_finding`, such as `supported` or `rejected`. A
supported surprise can promote a rival. `tool_failure` applies no route actions
and consumes no scientific retry. It records only the failed execution.

Repeated reviewed `inconclusive` outcomes count against the funded route's
`retry_limit`, default two. Reaching the limit parks that route and records the
reopening condition. Re-recording the same hypothesis, scope and observation
does not reset its count. Reframe the question or observation before another
allocation; the old inconclusive evidence stays intact.

Use `route_decide` for an explicit practical judgment on saved, imported or
unreviewed reports. It requires citations, exact current route scope, state,
reason and reopening condition. Its basis states that scientific admission is
separate. A manager can retire an exhausted narrow rule while preserving a
credible broader approach, without pretending the reports received a live review.

## Withdrawals, later reviews and human direction

`strategy`, `overview` and `worker_start` reconcile dependencies through the real
graph. Withdrawal or changed applicability queues the affected routes for
review. Later withheld admissions and invalidated submissions do too. A surviving
sufficient derivation retains the route's state while new funding is held for
explicit review. Loss of a required dependency parks a live/promising route.
Unrelated routes retain their state. Cite the change event and surviving evidence
in `route_decide` to record the review and its priority decision. Old evidence and
review history remain queryable.

`steering` keeps the human's text, actor, links and provenance. `intent: "note"`
and `intent: "hunch"` preserve ideas without changing allocations or becoming
empirical facts. An explicit instruction uses `intent: "instruction"` and can
select `next_assignment_id`. Supplying `allocation` creates a replacement ready
assignment with the new contract and retains the cancelled pending predecessor.
Running work continues. Steering a running assignment directly fails; cancel it
explicitly only when the human intends cancellation. Earlier evidence is never
rewritten. Selection of the next pending assignment follows the latest accepted
instruction, then route priority.

## Saved-source exercise and limits

Run `.venv/bin/python research/program-graph/strategy_cases.py` to inspect the
named source hashes and exercise real ProgramManager operations. The
[worked record](../output/research/sessions/2026-10-24-manager-strategy/worked-cases.json)
and [report](../output/research/sessions/2026-10-24-manager-strategy/report.md)
retain scoped CM-E1 retirement, a promising model rival, a funded common-fold
scout, illustrative steering, a counterfactual withdrawal and fresh-manager
reconstruction. No new science, actual human instruction or live review is
claimed by those illustrative cases. Deterministic fixture reviewers separately
test automatic outcomes, futility and admission boundaries. Task 016 still owns
the real coding-client and production-worker release proof.
