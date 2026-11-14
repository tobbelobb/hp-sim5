# Recovering the interrupted autocal session

Salvaged on 2026-11-15, after the original commit `43cbaff0` ("Makes interrupted
work recoverable"). Recovery evidence and documentation were subsequently amended
into that commit. No experiments were rerun and no implementation was changed.

Start with [conversation](evidence/conversation), then inspect `git status` and
the unfinished changes under `autocal/`. The recovery commit does not save the
implementation. The six modified source files and the two
untracked files (`autocal/pose_fit.py`, `autocal/tests/test_pose_fit.py`) were also
snapshotted under `recovery-originals/workspace-wip/`, together with
`tracked.patch`. Compare these copies before restoring anything over live work.
The patch covers tracked changes; the two new files are preserved separately.

## Small evidence to keep in Git

- [Continuation runs](evidence/continuation_runs/) retain the t1–t4 console logs,
  full-auto text/JSONL logs, and distinct replay input. Identical input files are
  referenced by hash in the inventory instead of copied repeatedly.
- [Headless runs](evidence/headless_runs/) now also retain stage fit/history
  snapshots, partial-point streams, force-tuning records, sweep configurations,
  scene snapshots, firmware configurations, recording sidecars, clocks and logs.
  Newly copied files preserve their original layout beneath each h1/h2/h3 folder;
  the previously archived manifest remains at the run folder's root.
- [Task outputs](evidence/task_outputs/) retain the original test completion log
  and two earlier diagnostic command outputs. `b4z8sv5cp.output` records
  **409 passed, 29 deselected**, exit code 0. The run excluded slow/heavy tests.
- [Recovery inventory](evidence/recovery-inventory.json) records original paths,
  sizes, SHA-256 hashes, local recovery paths and matching Git evidence paths.
  It also records the source snapshot hashes and the environment at salvage.
  The captured Git HEAD/environment describe salvage time, not necessarily the
  original experiment. Consult the original run manifests for experiment provenance.

The t1/t2 comparisons preceded the anchor-bounds fix and found mirrored layouts.
The t3/t4 comparisons include shared anchor bounds but precede the final Cauchy
refinement. Their approximately 3 mm fitter agreement is not sub-millimetre anchor
accuracy. Home-offset fitting worsened D Z on this poorly conditioned dataset.
The later unit-test and full-suite results do not establish end-to-end accuracy
for the proposed measurement scheme.

## Local originals outside `/tmp`

`recovery-originals/` is explicitly ignored by the repository's existing
`.gitignore`. It holds 221 original session files, totalling 1,128,591,262 bytes,
plus the unfinished-source snapshot:

- `6be52c02-0bfa-4f95-a23e-a262242026a8/` contains the interrupted research
  session's scratchpad and task outputs. This includes all three Rerun recordings,
  their event streams and the derived `rrd.pkl` caches. No pickle was loaded.
- `0fd224d1-2cec-4c8d-9386-94fb7b9ceff7/` retains an earlier HP3 session,
  including browser b1–b4 and headless r1/r2 data, frame/event streams, a browser
  helper and task logs. This is separately labelled context; its relationship to
  the research comparisons has not been established independently.

All copied originals were checked against their sources with SHA-256. Python
bytecode caches and process-ID files were omitted; original temporary files were
left in place. [Raw recording inventory](evidence/raw-recordings.json) now points
to the local recovery copies as well as the original temporary paths.

These copies survive `/tmp` cleanup on this machine. They are not part of a Git
commit, do not travel with a clone, and have no established off-host backup. Copy
`recovery-originals/` to durable shared storage before replacing this machine or
resuming elsewhere, and retain the inventory alongside it.

For an archived fit input, use `evidence/headless_runs/h2/sweeps.json`. Original
helper scripts and transcript commands contain absolute scratchpad paths. To use
their original directory structure without relying on `/tmp`, the preserved
scratchpad is now at:

```text
research/output/autocal_research_with_Fable5p2/recovery-originals/6be52c02-0bfa-4f95-a23e-a262242026a8/scratchpad/
```

Read historical commands before rerunning them: the transcript records a rejected
removal command, and `--no-collect` runs attempted to send M669 to a local simulator.
The next implementation steps and remaining validation are described in the
conversation. This salvage changes artifact availability, not the scientific claims
in the report.
