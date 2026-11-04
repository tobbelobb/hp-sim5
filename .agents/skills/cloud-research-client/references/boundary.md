# Cloud Research Program boundary

The cloud package is `.agents/skills/cloud-research-client/`. Its only mandatory
instruction entry is `SKILL.md`; setup and permission recovery are conditional
references. Local research does not read or import this package.

| Location within this skill | Responsibility |
| --- | --- |
| `SKILL.md` | Natural-language launch/resume entry and cloud actor dispatch/cancellation |
| `scripts/codex_cloud/` | Client policy, authenticated queue argv, runtime readiness and environment reload |
| `scripts/cloud_client_bridge.py` | Bind real actor identity and acknowledge actual final output or cancellation |
| `scripts/cloud_tool_bridge.py` | Launcher-owned private file relay to worker physics and assigned Viewer MCP |
| `scripts/env.sh`, `install.sh`, `start.sh`, `install_playwright_skill.py` | Cloud toolchain setup, environment and optional service startup |
| `assets/playwright-skill.json` | Pinned Playwright skill checksums |
| `tests/` | Adapter, environment, actor bridge and relay regressions; included by default pytest and CI |
| `scripts/build_report.py`, `assets/report_template.html`, `assets/research-program-cloud.html` | Offline architecture report with source prompt panels and historical evidence |

## Shared core

The shared `program*.py` code owns SQLite memory, assignments, budgets, dispatch
ownership, fixed evidence, review admission and saving. The existing worker
launcher, supervisor, native engines, RRF, collection, autocal and Rerun stay in
the core. Their shared correctness fixes remain there too.

`client_transport.py` is the lazy selection seam: local returns no adapter;
cloud adds this skill's scripts directory to Python's import path and imports
`codex_cloud.adapter`. Core callers use its existing optional hooks. There is
no platform autodetection or general plugin registry. The two shell launchers
source this skill's env.sh only for explicit cloud mode. Host descriptors retain
the selected transport through recovery.

A cloud open returns the canonical shared manager prompt as `manager_instructions`.
The cloud skill does not copy it. Local clients receive it through their existing
launcher. Workers and reviewers receive only their own role and assignment files.
Runtime queues/descriptors remain under ignored output directories and are not
portable program memory.

## Removing the adapter

Remove this skill directory and retire `--cloud`/environment startup hooks if
removing public cloud support. Keep the shared role prompts, research services
and correctness fixes. The local-boundary regression opens a real controller
and plans local managers from a checkout copy without this skill.

Run `.venv/bin/python -m pytest tests/python/test_research_program*.py
.agents/skills/cloud-research-client/tests tests/python/test_research_agent.py`
for the program/launcher/adapter checks. These are not evidence of a live model
research trial. Historical cloud evidence remains in research archives.

Regenerate the optional offline report after changing source prompts:

```bash
.venv/bin/python .agents/skills/cloud-research-client/scripts/build_report.py
```
