# Research Program cleanup, 6 November 2026

The feature was separated from the standalone research tooling in b9f01. This
follow-up removes the mandatory route-planning and allocation machinery that
still drove dispatch after that move.

- MANAGER.md chooses directions and communicates with the human; RESEARCHER.md
  owns hypotheses, experiments, debugging and fixes; REVIEWER.md checks and helps
  deliver useful work. The researcher can launch independently.
- research/AGENTS.md moved into the explicit researcher role so ordinary agents
  reading saved research do not inherit it.
- Dispatch needs a question and settings. Deleted strategy plans, investigation
  contracts, quota accounting, review funding and their obsolete tests/docs.
- Short overviews replace generated strategy boards, JSON dumps and narrative
  ledgers. Manager launch artifacts now stay in ignored storage.
- Researchers retain one report and original evidence. The manager adds one
  short HTML summary linking to them. Catalog entries contain one result sentence.
- Delivery now continues through passing combined checks and a squash on main;
  preparing a candidate alone cannot complete a program.

Separate review, evidence checksums, process ownership, cancellation, timeouts,
worker capacity and combined Git checks remain. Private submission snapshots
protect review integrity; they are not a second published result tree. Historical
records and existing investigations remain available in their original locations.

[Exact context trace](reference/context.md) · [Output layout](../human_docs/research-programs.md)
