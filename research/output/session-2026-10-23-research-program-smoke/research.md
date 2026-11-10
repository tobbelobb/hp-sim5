# Research program smoke tests and question-only entry

Objective: exercise the shipped program workflow and teach question-only use.
User requests a few smoke tests and a prompt an ordinary coding agent can use.

Budget before work: one bounded workflow check; zero fresh fits, physics runs or
steps, prediction/selection experiments, collections or movement. At most four
small CLI fixture scenarios plus one real catalog/evidence lookup and two
manual briefing/review cases. Standard-library harness; pytest is not installed
in this checkout. No world, launcher or model API is needed for these checks.

Hypotheses: RP-S1, catalog finds scoped results and distinguishes pending work;
RP-S2, explicit handoffs survive relocation and diagnose corrupt/missing evidence;
RP-S3, historical job outcomes retain their lifecycle/scientific limits; RP-D1,
a question-only entry can reach a useful scoped answer or bounded proposal.

Initial inspection: catalog search and handoff help work. Existing next-assignment
prompt explicitly ends at a proposal, so added research-program.txt to connect
that briefing step to execution, self-review and an agenda update. Added a short
question-first guide. Default is 30 minutes of saved-evidence work with zero
fresh experiments; the human can supply a different explicit budget. This is
prompt orchestration in the existing conversation, not an autonomous service.

Next: run the CLI smoke cases, perform the two manual desk cases, inspect the
review target and report the actual scope, then regenerate the session catalog.

Completed RP-S1/RP-S2/RP-S3: four unittest smoke checks passed once with fourteen
real CLI calls in 0.435 seconds; invocation outputs preserved. No failures.
Completed RP-D1: two manual question-only applications and same-agent self-reviews.
Saved CM-E1 evidence answers unconditional adoption; the HP4 mass question remains
unresolved and needs a separately budgeted matched test. Read the single-step
follow-up to avoid treating the earlier opt-in four-step result as today's default.
Recorded evidence/source identities in desk-inspection.json. No science run was
repeated, and no new scientific priority was inferred from the smoke fixtures.

Self-review of entry and guide: the briefing-only prompt is a preparation phase;
the entry authorizes one bounded execution, uses the human's constraints and
labels default-budget limits. New topic follows relevant evidence rather than
being forced into the autocal agenda. Next: final documentation/catalog checks.

Final documentation checks passed: 55 local links/anchors, seven saved prompt
references, matching report/abstract, README discovery and regenerated catalog
entry. Historical catalog entries absent from this checkout were preserved.
No runtime implementation changed, so no physics delivery gate was applicable.
Decision: ship the entry/guide and smoke evidence. The user can provide a question
to one existing agent, optionally setting a numerical budget. Live research
runtime remains untested in this smoke session.
