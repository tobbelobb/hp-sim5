# Portable research handoffs and historical autocal status

Use the archive CLI without starting a supervisor, world or Viewer:

```bash
.venv/bin/python scripts/research_handoff.py export SESSION SELECTION.json NEW_DIRECTORY
.venv/bin/python scripts/research_handoff.py show NEW_DIRECTORY
.venv/bin/python scripts/research_handoff.py job SESSION/autocal/JOB_ID/job.json
```

Copy the resulting directory to another machine, then run `show` there. The CLI
uses only Python's standard library. `show` locates the report and abstract,
returns the scoped claim/decision, checks selected file hashes and interprets any
selected saved jobs. It never reads the original source paths during recovery.
Keep the original session path stable while services are running.

## Choose the evidence

Write a small selection file, for example:

```json
{
  "claim": "CM-E1 rejects a blanket finite-prediction consensus restriction",
  "scope": "Eleven frozen simulated histories; final selection only",
  "decision": "Preserve current consensus policy; prediction safety is unresolved",
  "source_revision": "166f0352e032266711887d0abd01d194cbd4acf2",
  "limitations": ["No new fits or hardware measurements; runtime dependencies are external"],
  "files": [
    {"path": "results.json"},
    {"path": "inputs.json"},
    {"path": "inputs/history.json.gz", "source": "/old/worktree/history.json.gz", "role": "input"},
    {"path": "autocal/job.json", "source": "autocal/ended-job/job.json", "role": "job"}
  ]
}
```

Replace the illustrative paths with the files needed for the actual claim.
`abstract.md`, `report.md` and `research.md` are always selected from `SESSION`;
list only additional files. Each `path` is the relative destination/reference
inside the bundle. `source` defaults to that path relative to `SESSION`, or can
name an external worktree's file explicitly. Files are copied byte for byte,
including compressed inputs. Directories are not expanded. Unselected RRDs,
attachment credentials, dependency environments and other runtime files are not
copied. Select evidence files only; do not select credential files or live logs.

An optional `sha256` on each selection entry checks an existing known identity
before accepting the copy. Every copied file also gets a SHA-256 in the generated
`handoff.json`, which records its original source and relative destination. Use
the producer revision for the claim; copy its input/source manifests and relevant
scripts too. Record any differing analysis revision or absent prior evidence in
`limitations` and the report. The revision is supplied provenance, not a claim
that the CLI captured uncommitted source changes.

Keep local document links relative and preserve their target paths in the
selection. Historical absolute links are retained verbatim: the manifest's
`source` → `path` mapping locates the exported counterpart. The CLI does not
rewrite reports, follow arbitrary document links or guess which evidence proves
a claim. Include every input needed for that scope and name omitted context in
`limitations`. Missing sources, a known hash mismatch or malformed selected job
produce a partial manifest alongside the remaining useful files. Existing export
directories are never overwritten.

## Interpret recovery

`show` reports `integrity: verified_selected_files` only when all selected files
match their copied hashes and saved jobs parse. It reports `partial`, with file
diagnostics, when evidence is absent or damaged. Both commands return structured
JSON; exit 0 means the requested integrity/read check passed, 1 means partial
evidence and 2 means an invalid export request. A declared limitation remains
visible even when all selected bytes pass. Hash checks establish byte identity,
not valid input semantics, sufficient evidence or numerical reproduction. A
damaged manifest leaves the copied files available for manual recovery.

Saved `accepted`, `failed`, `cancelled` and `budget_exhausted` job states are
terminal **observations**. Saved `running`/`cancelling` states are
`historical_nonterminal`; current process state remains unknown. `accepted`
records CLI lifecycle acceptance, while `scientific_success` always remains
`not_assessed`. Read the report and measurements to assess the research result.
Corrupt or missing job records get an `unavailable` observation and diagnostics.

Live `autocal_status(job_id)` still addresses only the current supervisor's jobs.
After turnover, use `job PATH/job.json` or select the file with `role: job` for
portable inspection. Recovery does not inspect saved PIDs, signal processes,
adopt jobs or restore a world from a snapshot. To reproduce analysis, use the
recorded source revision and its dependencies, keeping the layout required by
the selected scripts; the bundle itself is an evidence handoff, not an executable
environment.
