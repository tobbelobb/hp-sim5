# 003: Turn a broad question into one economical assignment

Benefit: delegate question selection and briefing before building orchestration.
Budget: zero simulations, fresh fits or collections; no spawned research workers.

Read [autocal.txt](../prompts/autocal.txt), the
[parameter-variation prompt](../prompts/autocal_parameter_variation_focus.txt),
[the topic guide](../catalog.md), and the handoff introduced by task 002.

Add one general prompt under `research/prompts/` for the human's broad question,
notes and existing constraints. Use a supplied program note or the current
conversation; do not require a new program-file hierarchy. Consult a small set
of relevant abstracts, then follow the evidence links that affect the decision.

Return one proposed assignment with: the decision it serves; competing plausible
explanations; existing evidence; the cheapest useful probe; possible outcomes and
what they change; important guardrails; a bounded cost; and evidence to return.
Mention a credible alternative route when it affects the choice. Preserve the
human's question and distinguish a hunch from an established observation.

Use saved metrics or selection replay before fresh fitting, and fitting before
collection when each can answer the question. Do not pretend a scoring-only
replay tests a changed fitting/collection trajectory. If simulation cannot
distinguish explanations, say what measurement is missing. Do not launch the
assignment automatically or promise to resolve all Hangprinter unknowns.

Done when:

- The prompt yields one brief suitable for the existing research agent, and
  explicitly permits retiring a route or reporting inconclusive evidence.
- It carries enough scope/budget/provenance to review the resulting claim without
  requiring independent duplicate collection.
- Reading it against saved ranking results does not merely propose repeating the
  already-tested held-out-scoring idea with no changed hypothesis.

Verify by a short worked example using [Lane 04](../autocal-ranking-lane04-integration.md).
Keep that example in the prompt if it helps, otherwise summarize it in progress.
No new CLI, services, reviewer agents or numerical trial is needed.

## Implemented prompt and worked desk check

Prompt: [next-assignment.txt](../prompts/next-assignment.txt).

Example input: "Can autocal choose more accurate anchors without collecting more
points? My hunch is that held-out scoring could fix ranking." In this example,
new simulations, fits and collections have a zero budget.

Proposed; not tested. Decide whether restricting final consensus to candidates
with finite held-out prediction merits further validation. The human's goal is
better applied anchors from the same observations. The changed hypothesis is
about consensus membership, rather than adding held-out ranking again.

- Explanations: an unvalidated newest fit may degrade the average; alternatively,
  newer fits may contain useful information that a validation-only rule discards.
  A rank-penalty change is another route, but Lane 04's 04-E4a already found the
  same single-fit choice under four sort policies on these histories. Penalties
  that change fitting need a different budget and whole-run replay.
- Evidence: [Lane 04 integration](../autocal-ranking-lane04-integration.md) links
  the source report and 04-E4a policy measurements. Their saved five-points result
  has 6.546 mm summed per-anchor error for the chosen single fit and 10.894 mm for
  applied consensus [4,5]; layered has 8.983 versus 4.670 mm for consensus [3,5].
  Seventh HP3 applies a single fit because the closest pair exceeds the spread
  guard. The mixed consensus-versus-single result does not settle the proposed
  membership rule. The saved history JSONs confirm that iteration 5 in each has
  no held-out prediction. The integration's
  [checks](../../output/research/sessions/2026-10-22-autocal-lane04-cleanup/checks.json)
  record exact extraction equivalence and noise metrics in all 14 candidates,
  correcting the original report's missing-metrics claim.
- Probe: use only the three history paths/hashes in those checks, produced at
  4f175079, and the current
  [history selection functions](../../autocal/history_selection.py). Decode with
  [read_stage_artifact](../../autocal/stage_artifacts.py). Compare existing final
  selection with one variant that supplies only finite-prediction candidates to
  consensus, retaining the same single-fit ranking/fallback, merge threshold and
  spread guard. Reuse stored evaluated tuples; keep candidates, predictions and
  fits frozen. Return applied anchors, member IDs, spread and simulated-truth
  error for each policy and fixture. Do not rescore or refit the candidates.
- Decisions: advance only to a separate validation proposal if summed error falls
  across these fixtures with none worsening. A mixed or worse result retains
  defaults and retires the claim that this restriction helps all three; identical
  outputs retire it here as ineffective. Missing artifacts or a baseline mismatch
  make the comparison inconclusive and stop it. This screen cannot establish a
  better fitting/stopping trajectory, collection savings or hardware accuracy.
- Budget and guardrails: at most three histories and six final-selection
  evaluations, with a 15-minute analysis cap; zero fresh fits, physics steps,
  collections or movements. Preserve the two Slideprinter fixtures and HP3
  fixture's recorded settings. Use truth only for evaluation, never membership.
  Leave production defaults unchanged. A later implementation has its own checks.
- Return: one per-fixture table and saved applied-anchor arrays, commands,
  input/source hashes, baseline comparison, failures and decision handoff in the
  existing research artifacts. Cite producer checks for review; no duplicate
  collection is needed to inspect this frozen-history claim.

Desk-check extent: inspected the linked reports, saved policy JSON, history
metadata and current selection code; all three history hashes match the saved
checks. The numerical results above are saved measurements, not independent
reproduction. The membership variant remains untested. External worktree/session
artifacts are local and may be absent in another checkout.
