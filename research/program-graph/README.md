# Queryable program evidence

Task 008 extends the [program store](../program-store.md) with scoped claim
versions, typed relations, derivations, reported assessments and dated review
events. No graph service or optional Python package is required. The manager
release still needs tasks 009–016, including reviewed admission in 010.

## Commands

All mutation commands accept `--id ID` and `--expected-revision N`. Existing
artifact, question, approach and decision commands remain usable. JSON is printed
to stdout; invalid requests exit 2 with a diagnostic on stderr.

| Command | Arguments after `PROGRAM` |
| --- | --- |
| `claim` | `TEXT --scope JSON [--prior CLAIM_ID]` |
| `derive` | `REASON --conclusion CLAIM_ID [--premise ID] [--evidence ARTIFACT_ID]`; repeat the input flags |
| `relate` | `REASON --relation TYPE --source ID --target ID` |
| `assess` | `REASON --subject ID --outcome OUTCOME --evidence ARTIFACT_ID [--actor NAME]` |
| `review-event` | `REASON --subject ID --event withdrawal\|applicability\|supersession [--context JSON] [--actor NAME]` |
| `query` | `search [--text TEXT] [--scope JSON]`, `open_questions`, or `support\|upstream\|downstream\|rationale ID` |

Relations are `support`, `challenge`, `scope_limit`, `provenance`, `supersedes`
and `decision`. Evidence relations point from a claim/artifact/assessment to a
claim/question/approach/decision. Provenance points to an artifact. A decision
relation points from a dated decision to its approach/question. Supersession
requires later versions of the same claim key. `claim --prior` creates the
supersession and review event atomically; it preserves the old version and scope.

Each derivation supplies typed `required_premise` edges from its inputs to its
conclusion, labelled by derivation ID in query results. Every premise and cited
artifact in **one** derivation is required (AND). Separate derivations are
alternative sufficient paths (OR). Add a derivation rather than editing its
inputs. Missing/duplicate input IDs and cyclic required-premise paths fail before
writing. Discussion relations can cycle and cannot independently ground a claim.

Artifacts retain paths/URLs, SHA-256 and source revision. Supply actual settings
in `--provenance '{"config":{...},"input_identities":[...]}'`. Config must be an
object; repeated input identities within a record are rejected. Reuse stable
input hashes or measured-dataset identities across reports. Queries expose
identical byte hashes and shared declared inputs as correlated evidence. Missing
lineage never establishes independence. The store validates metadata; callers
remain responsible for inspecting the referenced bytes and their provenance.

Reported assessment outcomes are `supported`, `rejected`, `mixed`, `inconclusive`
and `not_tested`. They always retain `unreviewed`/`unassessed`. Import cannot admit
claims or assessments. Structural support means that a grounded recorded
derivation survives; it does not certify the scientific conclusion.

## Queries and change consequences

`support` includes support, challenges, limits and derivations around a subject.
`upstream` follows required premises; `downstream` follows affected conclusions.
`rationale` returns the route, its dated decisions and cited reasoning.
`open_questions` returns questions still unassessed; a question supported by a
report is not automatically closed. `search` matches saved text and details.
Optional scope filtering matches exact top-level scope key/value pairs and makes
no automatic scientific generalization.

Every result contains primary `matches`, full cited `records`, typed `relations`,
artifact `sources` with hashes/config/revisions, per-record `states` and shared
input warnings. Citation closure can include additional claims outside the
primary matches so that qualifications and sources remain visible. Queries read
one consistent program revision. `assessment_state` remains separate from
`structural_support` and from reported assessment outcomes in the records.

A withdrawal event makes its subject unavailable to required derivations. The
store preserves the old record, dates the event and records the affected claims
and derivations. Queries reevaluate support and review requirements, including
new dependencies added after the event. A valid alternative can preserve
structural support while the conclusion remains queued for review. Unrelated
claims remain unchanged. No automatic route decision follows from withdrawal.

For a new simulator/input configuration use `applicability` with the exact new
revision/settings in `--context`. Affected results expose that new applicability
as `unestablished`, while retaining the recorded historical support and scope.
This is different from withdrawing a faulty measurement. Events are append-only;
restoring/admitting evidence belongs to the reviewed admission path in 010.

The Python API is `store.graph().add_claim`, `add_derivation`, `add_relation`,
`add_assessment`, `review_event` and `query`; argument names follow these commands.
The [curation script](curate.py) and [focused tests](../../tests/python/test_research_program_graph.py)
are working API examples. Transactions and revisions are owned by ProgramStore.

## Inspected real-data exercise

[autocal-seed.json](autocal-seed.json) contains 16 unassessed findings, four open
questions and three approaches. It references nine inspected reports/summaries
and structured sources from CM-E1, two earlier ranking studies, the Lane 02
attribution correction, an unfinished notebook and the current agenda. It copies
no reports or telemetry. [inspection.json](inspection.json) records exact source
hashes, missing artifacts and the 11 CM-E1 compressed/raw input hashes checked.
Historical studies use shared fixture families; their reports are not independent
confirmations of CM-E1. No science replay was performed.

Load into a fresh manager-owned directory and ask the production CLI:

```bash
./hp-sim5-research-program --store /tmp/autocal-graph import research/program-graph/autocal-seed.json
./hp-sim5-research-program --store /tmp/autocal-graph query autocal-evidence rationale finite-membership
./hp-sim5-research-program --store /tmp/autocal-graph query autocal-evidence open_questions
./hp-sim5-research-program --store /tmp/autocal-graph query autocal-evidence search --text hangprinter_4 --scope '{"machine":"hangprinter_4"}'
```

The [saved transcript](queries.json) contains 12 successful fresh-process CLI
calls (`$STORE` denotes the exercise's temporary directory):

- Retirement traces to CM-E1's two anchor regressions and the missing fair
  holdout for changed applied models. The earlier scoped gains remain recorded.
- Open questions include common unseen sweeps, live HP4/hardware accuracy and
  completion of the unfinished selector investigation.
- CM-E1's manifest labels five unchanged histories `hangprinter_4`, including
  historical `hp3` filenames. Six other histories are `slideprinter`; all three
  changed selections are among those. Frozen simulated selection provides no
  live HP4 collection, stopping trajectory or hardware accuracy result.
- The counterfactual withdrawal of `cm-regressions` invalidates `retire-via-cm`
  and `keep-via-cm`. `keep-via-history` survives, preserving conservative support
  for keeping defaults; it does not reproduce CM-E1. The unrelated attribution
  correction stays unaffected. This exercise does not retract real evidence.
- A hypothetical source/config change leaves the historical compatibility
  measurement supported and marks new applicability unestablished.

The old paired-regression JSON, refreshed comparison JSON, Lane 02 summary JSON
and original motivating session are absent here. These limitations remain in
artifact provenance and answers. The pending notebook has no documented final
selector outcome. These imported reports have not passed task 010 admission.

Rebuild the seed and transcript with:

```bash
.venv/bin/python research/program-graph/curate.py
.venv/bin/python tests/python/test_research_program_graph.py -v
.venv/bin/python tests/python/test_research_program.py -v
```

The script checks archived bytes and saved result fields, starts no worker or
physics world, and rewrites only the seed/inspection/transcript beside it. See
the [exercise report](../../output/research/sessions/2026-10-23-program-evidence-graph/report.md)
for the inspection and verification boundary.
