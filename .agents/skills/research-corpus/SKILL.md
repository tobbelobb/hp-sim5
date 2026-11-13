---
name: research-corpus
description: Find prior hp-sim5 findings and publish or revise research reports, evidence references, and topic syntheses. Use when interpreting previous research or updating the research corpus.
---

Research has three useful levels:

- Runs preserve exact inputs, settings, commands, seeds when relevant, code revision,
  environment, measurements, and artifacts. Completing a run does not establish a claim.
- An investigation has one canonical report.md and evidence directory, reused across
  conversations. Keep abstract.md to a title and one result sentence linking to the
  report. Include informative negative results and unresolved contradictions.
- Topic notes under research/topics/ explain current understanding across investigations.
  Keep research/catalog.md as the curated entry point; link original reports rather
  than copying their narratives. Update an existing relevant note before creating one.

Before investigating, search the curated guide, relevant topic notes, and report
abstracts. Read evidence behind claims that affect your next decision. Historical
results have a source revision and scope; they do not automatically describe today's
solver. Do not load the entire corpus into the conversation.

Before publishing, explain the question, comparisons, observed result, strongest
alternative explanation, limitations, and what would change the conclusion. Make
simulation, synthetic, imported, and hardware evidence distinguishable. Separate an
interesting lead from an established improvement, and scientific value from code
readiness. Strengthen performance claims with appropriate baselines, repeated or
held-out comparisons, and uncertainty when the evidence warrants them. Exploratory
work may begin with an unexpected observation rather than a formal hypothesis.

When a finding changes understanding, update the relevant topic synthesis with its
conditions, supporting reports, contradictions or superseded conclusions, and useful
open questions. Update the program's agenda when the finding changes direction.
Fresh review should inspect evidence and reproduce a decisive comparison where feasible.

Keep small reports, synthesis, manifests, and reproducible helpers in Git. Large raw
recordings belong on durable storage available to future cloud runs; record their
portable URI, SHA-256, size, provenance, and access requirements. Moving a recording
is not permission to discard the only copy. Never commit connection credentials or
launcher logs from .research-state/. Retain useful partial and negative evidence;
absence of a final report alone is not grounds to delete an investigation.

Opening a client creates no corpus entry. Do not create placeholder abstracts or
duplicate a session just to resume. Use the assigned artifact path. Regenerate the
local browsing index after publishing with:

```bash
.venv/bin/python scripts/research_index.py
```

The generated index is disposable. Topic syntheses and original evidence are durable.
