## Tests
Don't change integration test expectations unless explicitly asked to do so.

## Attic
Don't look at files you find in directories called `attic` or `generated_source`.

## Ignore Changes made to Insignificant Files
If you find changes in .log or .md files by the end of your session, just ignore them and leave them alone.
Ignore files called "tags", don't remove them.
Just take care of the actual code files.

## Tool: cass
cass - Search All Your Agent History

Use when you need cross-agent or cross-session knowledge. Find solutions from old Codex and Claude sessions.
Before solving a problem from scratch, check if something similar were already solved in past conversations.
cass indexes past conversations into a unified, searchable index.

Never run bare cass. It launches an interactive TUI. Always use --robot or --json.

### cass Quick Start
Search across all agent histories: `cass search "authentication error" --robot --limit 5`
View a specific result (from search output): `cass view /path/to/session.jsonl -n 42 --json`
Expand context around a line: `cass expand /path/to/session.jsonl -n 42 -C 3 --json`
Search JSON: `cass search "query" --robot`
Paginate: use `_meta.next_cursor` → `cass search "query" --robot --cursor <value>`
Budget tokens: `--max-tokens 200 --robot-meta`
Minimal fields: `--fields minimal` (path,line,agent)
Freshness hint: `--robot-meta` (adds `_meta.index_freshness` + `_warning` when stale)
View source: `cass view <path> -n <line> --json`
Health: `cass state --json`
Get LLM-optimized cass docs: see ai_docs/CASS_ROBOT_MODE.md

### Key Flags of cass

| Flag | Purpose |
|------------------|--------------------------------------------------------|
| --robot / --json | Machine-readable JSON output (required!) |
| --fields minimal | Reduce payload: source_path, line_number, agent only |
| --limit N | Cap result count |
| --agent NAME | Filter to specific agent (claude, codex, cursor, etc.) |
| --days N | Limit to recent N days |

stdout = data only, stderr = diagnostics. Exit 0 = success.
