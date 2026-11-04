# Codex Cloud adapter boundary

Start with the [visual report](../../research/reports/research-program-cloud.html)
for actors, prompt text, tools and the measured October 26–27 run. Use
[RESEARCH.md](RESEARCH.md) to operate this adapter. The
[local program guide](../../research/program.md) describes the core feature.

## What belongs here

| Location | Responsibility |
| --- | --- |
| `src/python/hp_sim5_research/codex_cloud/` | Cloud policy: queue-client argv, inherited authentication, prohibition on nested clients/model overrides, runtime readiness before charging attempts, service-only startup and environment reload. |
| `scripts/cloud/cloud_client_bridge.py` | Wait for a real authenticated subagent, bind its actual identity, verify final-output hash, and acknowledge completion/cancellation. |
| `scripts/cloud/cloud_tool_bridge.py` | Launcher-owned file relay to actual hp_sim5 and assigned Rerun MCP attachments when actor command sockets are denied; Rerun's legacy initialize handshake. |
| `scripts/cloud/prompts/` | Cloud manager entry and workflow supplement. The old `research/prompts/research-manager.txt` is a symlink to the entry, preserving its public path. |
| `.agents/skills/cloud-research-client/SKILL.md` | Platform skill discovery entry and actor dispatch instructions; retained there because that is the skill-discovery location. |
| Other files in `scripts/cloud/` | Environment installation, pinned tools, software Vulkan fallback, setup documentation and this report's generator/template. Some support ordinary cloud development as well as Research Programs. |
| `tests/python/test_cloud*.py` | Cloud environment, actor bridge, tool relay and cloud dispatch guard regressions. These are fixture tests unless explicitly marked live. |

The default runtime queue is `output/cloud-research/client-queue`. The actor
request, private relay descriptors and live connection files are runtime state;
they are not portable program memory.

## What stays in the local core

`program*.py` owns the durable SQLite graph, strategy, human steering, finite
allocations, dispatch, process ownership, fixed submissions, separate review,
admission, optional pools/combined-patch checks and result saving. It does not
reason autonomously: the manager chat chooses the next investigation.
`coding_client.py` builds the normal installed Codex/Claude commands and reuses
their existing authentication. `scripts/research_runtime_check.py` checks common
runtime dependencies and launcher sockets; it is useful locally too.

The existing research-agent launcher, supervisor, RRF, native engines, collection,
autocal and Rerun remain shared services. In particular, these fixes found during
cloud research are general correctness fixes and must stay in the core:

- Worktree experiment output may use the launcher's intentional shared-output
  symlink, with containment checks intact (`experiments.py`).
- Supervisor test jobs get writable isolated caches (`test_jobs.py`).
- Reviewer clients receive an explicit session directory rather than assuming
  reviews live under `output/` (shared call contract; cloud argv lives in adapter).
- Browser intervention history matches its dictionary MCP schema
  (`scripts/hp_sim5_mcp.py`).
- Autocal status has finite JSON diagnostics and clears terminal ownership
  (`autocal_jobs.py`); HTTP encoding errors return valid error responses
  (`scripts/research_runtime.py`).
- Viewer settings/cache belong to the assigned session
  (`scripts/research_attach.py`), for local installations too.

## The remaining attachment points

There is one explicit selection seam: `client_transport.py`. Local mode returns
no adapter and never imports `codex_cloud`. Cloud mode imports its adapter lazily.
No plugin registry, base class or automatic environment detection is involved.

The core calls optional adapter hooks at client argv/environment construction,
entry validation, worker settings, runtime-check execution, pre-dispatch readiness
and launcher startup. The controller retains the shared lease renewal and wall
budget accounting around a potentially slow readiness check. CLI `--cloud` and
`HP_SIM5_RESEARCH_CLIENT` remain stable public selectors, and host descriptors
retain `client_transport` to prevent recovery from changing transport silently.
The two shell entry points have the small `env.sh` sourcing seam needed before
choosing Python. Direct Python entries use `client_transport.activate`.

## Replacing or removing Codex Cloud later

1. Keep the local core, shared workflow/reviewer prompts and shared correctness
   fixes above. Keep historical evidence and frozen submissions unchanged.
2. Replace the lazy selection in `client_transport.py`, or return no adapter for
   local-only use. Remove the cloud selectors/environment-sourcing seams from
   the public entry points when retiring their public interface.
3. Remove `codex_cloud/`, the two cloud bridges, cloud prompts/skill, and cloud
   tests. Remove cloud setup scripts only if ordinary cloud development no longer
   uses them. Remove the old prompt symlink and update its few documentation links.
4. Run the program/agent tests. `test_research_program_local_boundary.py` already
   opens a live local controller and plans Codex/Claude managers from a copy with
   no cloud package, scripts or cloud prompt.

This refactor moves policy and prompts; it adds no new research operations,
actor types, budgets, physics behavior or cloud recovery guarantees.

Regenerate the standalone report after changing source prompts or the explanation:

```bash
.venv/bin/python scripts/cloud/build_report.py
```
