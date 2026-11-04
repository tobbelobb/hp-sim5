# Research instruction audit

Audited 2026-11-04. This is a maintenance reference, not an agent startup read.
The scope is the active local/cloud manager, launched worker, reviewer and
single-agent entry paths, including their optional documentation. Historical
reports, frozen submissions and old task cards are evidence, not instructions.

## Graph before cleanup

Solid arrows mean a required read or injected text. Dotted arrows mean optional
navigation. The launcher/code nodes matter: Markdown links alone miss text that
is copied into a session. The root AGENTS instructions may also be loaded by the
coding client independently of these routes.

```mermaid
flowchart TD
    Local[program_entry.client_plan: local manager] --> Shared[research/prompts/research-manager-workflow.txt]
    Skill[.agents/skills/cloud-research-client/SKILL.md] --> Guide[scripts/cloud/RESEARCH.md]
    Cloud[scripts/cloud/prompts/research-manager.txt] --> Shared
    Cloud --> Supplement[scripts/cloud/prompts/manager-workflow.txt]
    Cloud --> Program[research/program.md]
    Cloud --> Guide
    Alias[research/prompts/research-manager.txt: symlink] --> Cloud
    Supplement --> Guide
    Shared --> RA[research/AGENTS.md]
    Shared --> Root[AGENTS.md]
    Shared --> Program
    Shared -.-> Tools[research/program-manager.md]
    Shared -.-> Strategy[research/program-strategy.md]
    Shared -.-> Review[research/program-review.md]
    Program -.-> Guide
    Program -.-> Tools
    Tools --> Program
    Tools -.-> Guide
    RA --> Readme[research/README.md]
    RA --> Investigation[research/investigation.md]
    RA --> Costs[research/autocal-recording-performance.md]
    RA -.-> Architecture[research/program-tasks/MANAGER.md]
    Readme --> Program
    Readme --> Guide
    Readme --> Execution[research/architecture.md]
    Launcher[scripts/research_agent.py: worker] --> RA
    Launcher --> Session[session instructions.md: copied worker rules and bindings]
    Reviewer[program_review.plan: reviewer] --> ReviewPrompt[research/prompts/reviewer.txt]
    Single[research/prompts/research-program.txt] --> RA
    Single --> Program
    Single --> Alias
    Single --> Next[research/prompts/next-assignment.txt]
    Single --> ReviewPrompt
```

The program guide also linked the cloud visual report before introducing the
local route. That report embeds entire prompt snapshots and links back to the
operating guides. It is particularly expensive as a default orientation read.

## Findings and changes

| Finding | Consequence | Resolution |
| --- | --- | --- |
| Machine selection repeated in shared workflow, strategy guide and cloud guide; only cloud rejected omissions | Same assignment had different defaults by transport | Shared worker validation rejects missing choices; capabilities explains both supported fields. Direct launcher requires `--machine`. Removed prose copies; retained executable JSON examples. |
| Cloud entry repeated runtime-check, installation and permission-recovery procedures from guide/supplement | Three copies could diverge | Entry only opens/binds and routes; supplement routes to the operating guide. |
| Cloud skill repeated dispatch, cancellation, review and save procedures | Another full instruction copy; worker attachment advice lagged behind the guide's file-relay route | Skill is a short entry router. Cloud guide owns adapter procedures. |
| Shared workflow required worker AGENTS, broad README, integration essay and program guide | Managers paid for worker experiment instructions and navigational cycles | Shared manager prompt no longer requires those reads. Worker rules still arrive through the launcher. Integration essay is optional background. |
| Shared program-manager guide described cloud env loading, relay mechanics and extra cancellation acknowledgements | Local managers learned adapter procedures they cannot use | Removed those details; retained a conditional link to the cloud operating guide. |
| Worker README repeated cloud startup, saving and pilot history | Workers were routed back into manager/cloud operations | Replaced with optional program-guide navigation. |
| Single-agent entry pointed to a shared-looking symlink whose target was the cloud entry | Local research could accidentally select cloud mode | Points to neutral program guide. Compatibility symlink remains for existing callers. |
| Physics delivery rule repeated in worker AGENTS, README, manager workflow/tools and reviewer prompt | Multiple copies of a rule already enforced by tests/admission | Removed these copies. `npm test`/CI own stability execution; review admission still enforces candidate-specific evidence in code and its detailed reference. |
| Single-agent entry repeated worker recording, reporting, citation and catalog instructions | Same session read the rules twice | Entry now owns only iteration orchestration, briefing authorization and self-review. Worker AGENTS owns experiment/reporting practice. |
| Cloud child could inherit the whole manager conversation in addition to queued instructions | Worker/reviewer received unrelated manager instructions and duplicated context | Cloud dispatch specifies fresh actor contexts (`fork_turns="none"`) with queued prompt/instructions supplied once. |

## Active graph after cloud skill consolidation

```mermaid
flowchart TD
    Local[Local launcher] --> Shared[Shared manager workflow]
    Skill[Cloud Research Program skill] --> Open[Cloud program open]
    Open --> Shared
    Skill -. setup failure .-> Setup[Skill setup reference]
    Skill -. permission failure .-> Recovery[Skill recovery reference]
    Skill -. maintenance only .-> Boundary[Skill boundary reference]
    Shared -. as needed .-> Tools[Manager tool reference]
    Shared -. as needed .-> Strategy[Strategy reference]
    Shared -. as needed .-> Review[Review reference]
    Launcher[Worker launcher] --> Rules[research/AGENTS.md]
    Launcher --> Binding[Session paths and world binding]
    Rules -. as needed .-> Readme[Worker tool README]
    Rules --> Costs[Recording costs before numerical budgets]
    Rules --> Abstract[Abstract template when reporting]
    Rules -. relevant task only .-> Topic[Native / browser / autocal tool guides]
    Reviewer[Review planner] --> ReviewPrompt[Reviewer prompt]
    Reviewer --> Contract[Fixed evidence and JSON response contract]
    Single[Single-agent entry] --> Next[Briefing prompt]
    Single --> ReviewPrompt
```

Cloud open returns the shared manager workflow text as `manager_instructions`;
the skill does not duplicate it or require a separate reading hop. The cloud
adapter, bridges, setup scripts, report assets and tests now live under
`.agents/skills/cloud-research-client/`. `scripts/cloud/` and the old entry prompt
and symlink have been removed. Cloud tests remain in default pytest/CI discovery.

Workers receive their assignment/strategy contract separately from standing
research rules; those bindings are data, not another instruction manual.
Reviewers receive the review prompt plus exact submission/schema/deadline data,
not manager or worker operating guides. Local Codex gets worker rules through
`developer_instructions`; Claude receives them through `instructions.md`.
The cloud broker supplies the same queued files to a fresh actor context.

## Ownership and remaining boundaries

| Information | Owner / discovery point |
| --- | --- |
| Machine choice, accepted settings, conflict diagnostics | `program_workers.py`, controller `capabilities`, launcher `--help` |
| Manager decisions, bounded delegation, review sequence, saving | Shared manager workflow |
| Experiment setup, recording decisions, evidence/report practice | `research/AGENTS.md`; topic guides only when relevant |
| Cloud actor dispatch, credentials, file relay, permission recovery | `.agents/skills/cloud-research-client/SKILL.md` and optional recovery reference |
| Scientific review criteria | `research/prompts/reviewer.txt` |
| Fixed review schema, identity and engineering admission | `program_review.py`; detailed `program-review.md` reference |
| Physics stability | Default test pipeline; failure notes in `tests/benchmark/README.md` |

Cloud-specific procedures in shared mandatory instructions are a layering error.
Conditional links to the optional adapter are navigation, not a reason to load
its instructions locally. Shared code reporting `client_transport` or preserving
a bound mode is a common interface contract and belongs in the shared layer.
Socket restrictions are possible locally too; common preflight/socket diagnostics
are not inherently cloud-specific.

Architecture reports and historical task cards remain optional references,
not startup instructions. Archived evidence is unchanged. The generated visual
report now lives inside the skill and is refreshed from the active source prompts.

An exact once-only context guarantee also depends on the coding client: automatic
AGENTS discovery, previously read files and an existing parent chat are outside
this repository's prompt loader. This cleanup removes explicit redundant read
edges and copied procedures; it does not claim to erase client-provided context.

## Verification

The program/controller/worker/review tests, cloud adapter tests and launcher tests
passed across split runs. The cloud relay and launcher portion was rerun with
local sockets permitted after the sandbox rejected the relay fixture's bind.
New checks cover omitted choices without session creation or budget charges,
commissioned choices and conflicts in both transports, and discoverable help.
The cloud skill passed its frontmatter validator. The offline visual report was
regenerated from source prompts, and `git diff --check` passed. These are code,
CLI and process/relay tests; no new live model research trial was commissioned.
