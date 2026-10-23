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
