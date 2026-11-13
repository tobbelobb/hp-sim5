# Is Research Program delivering less useful work?

Assessment at `1360d980`, 27 October 2026. This is an inspection of saved results
and current source, with arithmetic checks on two historical timing datasets.
No models, simulations, fits or production services were run for this assessment.

**The available records support the experience of lower delivered value so far.**
The older direct-agent sessions contain measured implementation improvements.
The managed runs mostly demonstrate evidence administration and tool plumbing.
They do not yet demonstrate the intended end-to-end replacement for choosing
research prompts, supervising substantial investigations, reviewing and merging
useful changes. This supports a practical diagnosis, not a causal claim that an
extra manager inherently makes research worse.

**The strongest explanations are changed assignments, much shorter research
budgets, and an incomplete delivery contract.** Cloud runtime failures compound
those differences. Turning a Goal-mode flag on is not sufficient to fix them.

## What the saved runs show

The [data file](data.json) lists every included snapshot, session, classification
and source hash. [audit.py](audit.py) recomputes the counts and timing arithmetic.

| Evidence group | Work actually commissioned | Observed result |
| --- | --- | --- |
| October 24 original live manager gate | One tiny production fit and a saved-evidence investigation | Worker failed on native prerequisites before science; separate client/reviewer startup also failed. No scientific admission. |
| October 24 cloud pilot | 100-step runtime check, saved-evidence review and deliberate cancellation | Two report-only admissions; actual native/MCP/recording use; no calibration improvement. |
| October 26 initial local program | Saved lineage inspection, explicitly no new numerical experiments | One planned assignment, no recorded worker attempt. |
| October 26 completed local program | Eight-minute inspection of four existing CM-E1 sources, explicitly no new numerical experiments | Separate review and report-only admission; no calibration code change. |
| October 26–27 cloud audit | Tool inventory, production-path checks and runtime repairs | Useful source repairs and one ready admission, but final full-auto/browser regression interrupted. No calibration algorithm improvement was claimed. |

Across these **five saved program snapshots**:

- There are **nine launched worker attempts**: five submitted, two failed during
  startup and two cancelled. One cancellation was an intentional lifecycle test;
  the other reached its wall deadline. These counts are not a scientific success
  rate.
- Worker wall allocations are **2, 2, 3, 3, 8, 15, 22, 25 and 25 minutes**.
  The median is **eight minutes**. The longest observed worker lifetime is
  **15.09 minutes**. Together, all nine worker lifetimes total **55.65 minutes**,
  excluding reviews, manager work, parent-run checks and gaps between sessions.
- Six admissions comprise **four report-only, one needs-changes and one ready**.
  The ready result concerns cloud/tool repairs. It is real useful work, rather
  than a calibration accuracy or collection-efficiency result.
- These snapshots contain **zero integration records**. Integration/pool fixture
  tests exist elsewhere, but no included actual run demonstrates the combined
  candidate workflow on a substantive scientific improvement.

This is not evidence for a built-in one-hour manager limit. The controller
accepts larger finite allocations. The cloud audit's creation-to-final-update
span is more than eleven hours, including interruptions and runtime recreation;
that span cannot be treated as active research time. The evidence supports short
worker opportunities, not a reliable direct-versus-program elapsed-time ratio.

## The older agent did produce the kind of outcomes you remember

For a reproducible comparison group, I selected first-level session directories
named `2026-*-improve-encoder*` that retain `launch.json`. There are **16**, of
which **12 have completed summaries and four remain unsummarized**. The completed
summaries describe ten retained implementation changes, one tooling-only repair
and one investigation that justified no source change. All 16 launch records
use the interactive route rather than `codex exec`.

The ten change reports overlap in topic, and some changes were subsequently
revised. They are not ten independent improvements or ten verified merges.
The four unsummarized runs also prevent treating the old workflow as uniformly
successful. Three similarly named directories without launcher provenance and
the many hash-name aliases were excluded, as were unrelated interactive/tooling
sessions. This is a transparent sample, not a complete lifetime census.

Representative results:

| Direct session | Retained result and producer verification | Practical limitation |
| --- | --- | --- |
| [Exact spool-model reuse](../../../research/output/session-2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/report.md) | Three-repeat median replay savings of 2.54% and 10.36%; reports unchanged eleven-fixture results and 5,184 numerical fields | Fitting throughput; no collection or accuracy improvement. I recomputed those percentages from saved timings. |
| [Guarded encoder settling](../../../research/output/session-2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md) | Matched 36-point collection: 429.668 → 320.364 simulated seconds, 25.4% shorter | One simulated collection pair; equal final calibration accuracy on new data was not established. |
| [Stable encoder averaging](../../../research/output/session-2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/report.md) | Small averaging fix; complete automatic simulated collection and more stable repeated fits | The fresh-baseline comparison took more time for lower anchor error; the report explicitly rejects an unconditional speed claim. |
| [Warm-started anchor solve](../../../research/output/session-2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md) | Matched jobs with identical 180-point datasets: 2.706 → 2.420 mm applied-anchor RMSE | Simulation, single matched pairs and limited machine/settings scope. Its notebook explicitly continues the same investigation across four turns. |
| [Constant-objective shortcut](../../../research/output/session-2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/report.md) | Four complete regression suites: 2.81% less mean wall time and 4.31% less child CPU; reports unchanged numerical results | Conditional compute saving; no collection-time or accuracy improvement. I recomputed the timing gains from saved trials. |

Relevant historical implementation commits, including `54e88492`, `854430c6`,
`9422620c`, `294e24c8` and `79e53cbb`, are ancestors of this checkout. That
corroborates actual source delivery; it does not establish that every historical
patch remains unchanged or that its measured benefit persists on today's code.
I inspected reports/artifacts and recomputed saved arithmetic, rather than
rerunning their tests or independently reproducing calibration.

## Why this comparison is confounded

Two of the three October 26 program questions explicitly prohibit new numerical
experiments. Producing an inspected report was correct for those requests.
The cloud pilot and audit target platform operation. They cannot fairly be
graded as failed attempts at the much broader old autocal task.

The completed local audit repeats a conclusion already present in the
[October 23 CM-E1 report](../../../research/output/session-2026-10-23-consensus-membership/report.md):
a small aggregate gain coexists with two per-dataset regressions. Its new value
is fixed independent review and durable admission. If you already knew that
finding, its marginal research value is understandably small. Both statements
can be true without treating proper review as useless.

Task, platform, source revision, model/client and allowed effort differ.
The direct sessions have no reliable complete elapsed-time/token accounting
here. Neither a comparative success percentage nor the claim that Goal mode
caused the difference is established by this archive.

## Source-level explanations

| Difference | Current evidence | Likely consequence — interpretation |
| --- | --- | --- |
| The objective shifted toward reviewed knowledge | [autocal.txt](../../prompts/autocal.txt) explicitly says to implement the best supported change and run regressions. The [manager workflow](../../../src/python/hp_sim5_research/program_instructions/manager.md) emphasizes a small investigation, evidence, review, a decision and result saving. | An accurate answer about existing evidence can satisfy the manager even when the human expected a delivered improvement. |
| Existing topic prompts were not selected automatically | [ProgramManager](../../../src/python/hp_sim5_research/program_manager.py) adds the strategy contract; [WorkerDispatcher](../../../src/python/hp_sim5_research/program_workers.py) adds an assignment and supplied instructions. This audit preceded the new `prompt_portfolio` triage registry. | The new layer enumerates `research/prompts/` candidates with decision metadata for the manager to compare, then records the selected prompt on `route_plan`; live Program behavior still needs validation. |
| Worker continuity changed | Workers always launch with `--batch`, one prompt, a fresh session and a wall deadline. An assignment dispatches once; recovery does not resume its model conversation/world. The direct warm-start notebook records four turns in the same investigation. | Short independent inspections replace opportunities to try a failed idea, repair it, test another and finish a patch within one continuing investigation. This need not require any new resume machinery: a longer, properly scoped worker can already do more. |
| Small budgets dominate the live record | Nine caps above have an eight-minute median; the final live regression expires at fifteen minutes during a parent permission wait. | Setup, queue brokerage and permissions can consume a meaningful fraction of the entire scientific allocation. An early response can mean insufficient opportunity rather than efficient completion. |
| Reviewed acceptance does not require a delivered improvement | [Task 016](../../program-tasks/016-live-manager-release.md) permits a small production-tool check plus saved evidence. `keep/report_only` is valid. | Passing an orchestration demonstration says little about replacing the human's practical improvement workflow. |
| Integration ends before merging | [ProgramIntegrations](../../../src/python/hp_sim5_research/program_integration.py) prepares/checks a combined candidate and reports `automatic_merge_or_push: false`; the manager prompt says not to merge experimental candidates automatically. | The desired review-and-merge chore is not actually automated end to end. This is deliberate current behavior, not a hidden merge failure. |
| Runtime failures are real | Original live gate: headers/client/MCP failures. Cloud audit: worktree output, caches, Viewer/rendering/protocol, invalid status and browser-history responses, plus a permission stall. | Some short runs were diagnosing real blockers. A report about a scientific question should expose those blockers immediately instead of burying them in limitations. |

Current local managers and default workers enable the client's Goal feature.
The five October 26–27 actual attempts record `goal_mode: true`; the earlier
three pilot attempts deliberately record false. In cloud, the
[adapter](../../../.agents/skills/cloud-research-client/scripts/codex_cloud/adapter.py) uses the
parent's actors: CLI Goal/model settings do not configure them. A settings flag
also does not prove an explicit goal was established or continued. The archive
therefore supports a continuity/budget explanation much more directly than a
Goal-mode explanation.

## Changes I recommend, in order

### 1. Commission delivery explicitly and reuse the successful topic prompts

For an improvement program, make the objective a baseline-to-candidate result
with a useful implementation when evidence supports it. Ask the manager to
choose among the existing prompts and record the chosen path and reason. Pass
that prompt's full content to the worker, followed by current scope, known
negative results, constraints and acceptance measurements. Keep the researcher's
permission to explore credible alternatives.

Keep evidence-only audits available when explicitly requested or when a short
check can prevent repeating a known mistake. A negative finding can be valuable;
it should resolve a previously open practical choice. Rechecking a settled
conclusion should not routinely consume an improvement program's only worker.

This can start as a clearer question/assignment contract. It does not need a
new graph schema, scheduler, agent hierarchy or permanent evaluator service.

### 2. Give one researcher a complete improvement mission

Start with one worker in its own worktree and one separate reviewer. Let that
worker handle baseline reproduction, several cheap hypotheses, implementation
and verification in one conversation. Avoid turning each measurement or concern
into another short assignment. Use pools only after this serial path delivers.

For a first trial, an explicit **three-hour overall budget**, with up to
**90 minutes for the research worker**, **20 for review**, **30 for required
delivery checks**, **15 for setup/dispatch** and the remainder for integration
and cleanup, is a reasonable proposed starting allocation. These numbers are an
experiment design, not measured optimal budgets. Increase a necessary allowance
only within the human's agreed total; do not weaken checks or silently extend it.

Keep the manager working through review, any justified correction and the final
delivery decision until the objective is achieved, an actionable blocker is
established or the agreed total expires. Local Goal configuration is already
enabled; the missing piece is that completion contract and sufficient work time.

### 3. Add the missing delivery policy

Agree once on authority to apply/cherry-pick a reviewed, passing candidate to a
designated integration branch, then carry that step through. Use the existing
combined-candidate checks when combining patches, resolve conflicts and review
the actual resulting change. Respect the required physics gate where relevant.
Return the delivered commit and verification evidence, or the exact reason it
could not be integrated. Keep push authority explicit.

Saving a report-only archive remains useful, but its commit must not be presented
as delivery of a verified algorithm improvement. The current no-auto-merge policy
will need an explicit override or small product change to meet this objective.

### 4. In cloud, prove the tool path before spending the mission budget

Use the same controller/launcher permission context that will run the worker.
Check runtime readiness, then preflight the actual attached supervisor and verify
the worker's relay access. Handle any required permission/setup requests before
funding long production work. A later grant to a different command does not fix
the environment of an existing controller.

Use `viewer: none` when the question needs numerical evidence only; enable the
Viewer and richer recording when the investigation requires them. The Viewer
startup failure here is a reason to choose it deliberately, not to avoid useful
visual inspection generally.

Reserve startup and queue-broker time explicitly. A later adapter improvement
could start the scientific allowance after actor binding/readiness, while still
keeping a separate finite startup deadline. Measure queue wait and first useful
tool latency before implementing that change; do not leave startup unbounded.
Keep the parent chat active to broker real actors and completion/cancellation.

Report status as the first sentence: `delivered improvement`, `new negative
finding`, `existing-evidence review`, `blocked before experiment`, or `budget
exhausted`. Name the failed command/tool, whether science ran, what is retained
and the one action that would unblock it. Do this locally too.

### 5. Judge usefulness with a small matched delivery trial

After setup is ready, choose three modest open implementation tasks from the
current backlog. For each, run direct-agent and managed versions from the same
base/input snapshot, with the same topic prompt, client/model where controllable,
permission profile and research/review/check allowances. Keep cloud/local results
separate. Allow the same delivery authority in both arms. Freeze acceptance
criteria before either starts and inspect the resulting changes independently.

Measure verified useful changes actually integrated, new decision-changing
negative findings, human interventions/time, active research time, startup and
review overhead, blocker visibility and required-check completion. Record token
cost only if the platform exposes it. A no-patch outcome can earn credit when it
establishes an unresolved issue; rereading known conclusions should not count as
new research. Three pairs provide an initial engineering signal, not a precise
success probability.

My practical recommendation is to use the direct agent for substantive
implementation work until the manager demonstrates this delivery cycle. Use the
Program layer to select a prompt, give a capable worker enough time, and finish
review/integration. Retain the graph and extra coordination features only where
they reduce your work in that measured cycle.

## Reproduction and source boundaries

Run from the repository root:

```bash
.venv/bin/python research/reports/program-usefulness/audit.py
```

The script needs this machine's ignored `research/output` evidence as well as
committed archives. The saved JSON retains the counts, paths, classifications
and hashes when those raw files are unavailable elsewhere. It deliberately
excludes live connection descriptors, credentials and client configuration.
Local SQLite stores were inspected read-only; they contain the two October 26
programs already included above and add no independent runs.

Primary program evidence:

- [Original live gate](../../../research/output/session-2026-10-24-live-manager-release/report.md)
  and its [snapshot](../../../research/output/session-2026-10-24-live-manager-release/program-export.json).
- [October 24 cloud pilot](../../cloud-client-pilot/2026-10-24/README.md)
  and [snapshot](../../cloud-client-pilot/2026-10-24/program-export.json).
- Initial local program [snapshot](../../../research/output/program-d71e9395e72e43db97d4e21c0d052f51/program.json).
- [Completed local evidence audit](../../../research/output/session-2026-10-26-cm-saved-evidence-review/report.md)
  and [snapshot](../../../research/output/program-e31e5321be5940baacf677409b2579e0/program.json).
- [Cloud audit](../../../research/output/program-7aa80aa18f10499e85cb24ffeae712cb/selected/output/research/sessions/2026-10-26-cloud-platform-audit/report.md)
  and [snapshot](../../../research/output/program-7aa80aa18f10499e85cb24ffeae712cb/program.json).

No external publications or undocumented platform behavior are needed for these
findings. The recommendations above are hypotheses to test, not measured gains.
