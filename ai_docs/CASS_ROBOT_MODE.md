# Robot Mode Guide (cass)

Updated: 2025-12-02

## TL;DR (copy/paste)
- First index: `cass index --full`
- Search JSON: `cass search "query" --robot`
- Paginate: use `_meta.next_cursor` → `cass search "query" --robot --cursor <value>`
- Budget tokens: `--max-tokens 200 --robot-meta`
- Minimal fields: `--fields minimal` (path,line,agent)
- Freshness hint: `--robot-meta` (adds `_meta.index_freshness` + `_warning` when stale)
- View source: `cass view <path> -n <line> --json`
- Health: `cass state --json`

## Core commands for agents
| Need | Command |
| --- | --- |
| Search with JSON | `cass search "panic" --robot` |
| Search today | `cass search "auth" --robot --today` |
| Wildcards | `cass search "http*" --robot` |
| Aggregations | `cass search "error" --robot --aggregate agent,workspace` |
| Pagination | pass `_meta.next_cursor` back via `--cursor` |
| Limit output fields | `--fields minimal` or comma list (`source_path,line_number,agent,title`) |
| Truncate content | `--max-content-length 400` or budgeted `--max-tokens 200` |
| Metadata | `--robot-meta` (elapsed_ms, cache stats, index freshness, cursor, warnings) |
| Health snapshot | `cass state --json` (alias `status`) |
| Capabilities | `cass capabilities --json` |
| Introspection | `cass introspect --json` (schemas for responses) |

## Response shapes (robot)
- Search:
  - top-level: `query, limit, offset, count, total_matches, hits, cursor, hits_clamped, request_id`
  - `_meta` (with `--robot-meta`): `elapsed_ms, wildcard_fallback, cache_stats{hits,misses,shortfall}, tokens_estimated, max_tokens, next_cursor, hits_clamped, state{index, database}, index_freshness`
  - `_warning` present when index is stale (age/pending sessions)
  - `aggregations` present when `--aggregate` is used
- State/Status: `healthy, recommended_action, index{exists,fresh,last_indexed_at,age_seconds,stale}, database{exists,conversations,messages,path}, pending{sessions,watch_active}, _meta{timestamp,data_dir,db_path}`
- Capabilities: `crate_version, api_version, contract_version, documentation_url, features[], connectors[], limits{max_limit,max_content_length,max_fields,max_agg_buckets}`

## Flags worth knowing
- `--fields minimal|summary|<list>`: reduce payload size
- `--max-content-length N` / `--max-tokens N`: truncate per-field / by budget
- `--robot-format json|jsonl|compact`: choose encoding
- `--request-id ID`: echoed in results/meta; good for correlation
- Time filters: `--today --yesterday --week --days N --since DATE --until DATE`
- Aggregations: `--aggregate agent,workspace,date,match_type`
- Output display (humans): `--display table|lines|markdown`
- Progress: `--progress bars|plain|none|auto`; Color: `--color auto|always|never`

## Best practices for agents
- Always pass `--robot`/`--json` and `--robot-meta` when you care about freshness or pagination.
- Use `--fields minimal` during wide scans; fetch details with `cass view` if needed.
- Respect `_warning` and `index_freshness.stale`; run `cass index --full` (or ask a human) when stale.
- Store `_meta.next_cursor` for long result sets; avoid re-running the base query.
- Include `--request-id` to correlate retries and logs.
- Clamp limits to published caps (see `cass capabilities --json`).
- Prefer `--max-tokens` to keep outputs small in LLM loops.

## Integration snippets

### Python
```python
import json, subprocess

cmd = ["cass", "search", "error", "--robot", "--robot-meta", "--max-tokens", "200"]
out = subprocess.check_output(cmd, text=True)
data = json.loads(out)
print(data["_meta"]["elapsed_ms"], "ms", "hits:", len(data["hits"]))
```

### Node.js
```js
import { execFileSync } from "node:child_process";

const out = execFileSync("cass", ["search", "timeout", "--robot", "--fields", "minimal"], { encoding: "utf8" });
const result = JSON.parse(out);
console.log(result.hits.map(h => `${h.source_path}:${h.line_number || 0}`).join("\n"));
```

### Bash
```bash
cass search "panic" --robot --fields minimal --robot-meta \
  | jq -r '.hits[] | "\(.source_path):\(.line_number // 0) \(.title // "")"'
```

## Troubleshooting
- “missing index” → run `cass index --full`
- Stale warning → rerun index or enable watch mode
- Empty results but expected matches → try `--aggregate agent,workspace` to confirm ingest; check `watch_state.json` pending
- JSON parsing errors → use `--robot-format compact` to avoid pretty whitespace issues

## Change log (robot-facing)
- 0.1.30: `_meta.index_freshness` + `_warning` in search robot output; capabilities limits enforced; cursor/request-id exposed.

---
For deeper schemas: `cass introspect --json` and `cass capabilities --json`.
