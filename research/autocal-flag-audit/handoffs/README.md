# Six parallel autocal investigations

Give one task file to each of six researcher sessions. Each file is a separate handoff and can start immediately. These are prepared tasks; no experiments have been launched.

| Lane | Handoff | Owns | Initial numerical budget beyond baseline capture |
| --- | --- | --- | --- |
| 01 | [Solvers](01-solvers.md) | Optimizer methods, differentiation modes, fallback and anchor budgets | 12 fit executions |
| 02 | [Filtering](02-filtering.md) | Noise, residuals, threshold, filtering and filter schedule | 12 fit executions |
| 03 | [Spool refinement](03-spool.md) | Scale fixes, spool budgets/priors and objective schedule | 12 fit executions |
| 04 | [Ranking and CLI](04-ranking-cli.md) | Rank penalties, ensembles, diagnostics and CLI cleanup | 12 history-policy evaluations, at most 3 reconstruction fits |
| 05 | [Physical model](05-physical-model.md) | Flex, stiffness, origin, plane constraints and tension projection | 12 fit executions; no fresh collection |
| 06 | [Planning and stopping](06-planning-stopping.md) | Candidates, information, stopping, recovery and point count | 12 plan executions, at most 3 recovery fits; no fresh collection |

The original E1–E7 labels remain in the [assessment](../report.md); lane 06 combines E6 and E7. Every lane owns its own flag conclusions. Lane 02 owns the filter schedule; lane 03 owns the objective schedule. Lane 04 owns CLI cleanup, taking numerical recommendations from the other lanes later. Shared physical KEEP settings are controls, not removal targets.

## Common execution contract

Read the root AGENTS.md, research/AGENTS.md, research/README.md and recording-performance guidance. The original [inventory](../inventory.md), [usage excerpts](../usage.json) and [assessment](../report.md) provide source leads and prior research. Read relevant prior negative results before repeating a rejected idea. No lane requires another lane to complete its work.

Use the same starting source snapshot, defaults and development/validation partition. The static audit's revision is in usage.json; do not assume today's checkout still matches it. Before starting, record actual HEAD, dirty diff and hashes of `autocal/*.py`, tool scripts and fixture inputs. If source differs from the audit, note the difference and use one agreed current snapshot for all six lanes. Do not silently compare runs from different code.

Run the six lanes in one read-only source checkout with separate output directories. Put experimental harnesses and copied inputs under the lane's own session directory. Do not edit shared runtime code, defaults, shotgun.conf, references, handoffs, audit evidence or the research catalog during concurrent work. Maintain your own notebook/abstract/report; `research_index.py` can be regenerated once after all six finish to avoid concurrent catalog writes. This is a scheduling instruction, not permission to omit discoverability.

If an investigation needs runtime edits, use a separate worktree at the pinned source snapshot, with its own artifacts, and preserve the patch there for review. Do not create a worktree and assume uncommitted audit/handoff files transferred: copy these instructions explicitly or commit them before checkout creation. Shared `.venv` is for execution only; do not install/update packages while other lanes use it. No lane merges a change into another running lane's source.

## Per-session setup

From the repository root, set the lane ID to the exact value from the assigned handoff. Use a fresh output root for a new investigation; do not reuse an existing run accidentally.

```bash
export FLAG_LANE=01-solvers
export FLAG_RUN_ROOT="$PWD/output/research/sessions/2026-10-22-autocal-flags-$FLAG_LANE"
mkdir -p "$FLAG_RUN_ROOT"
export JAX_COMPILATION_CACHE_DIR="$FLAG_RUN_ROOT/jax-cache"
export PYTHONHASHSEED=0
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
```

These settings reduce contention; they do not guarantee that JAX uses one CPU thread. Use one active numerical subprocess per lane, not nested parallel regressions. Keep caches separate to prevent cross-lane warm-cache interference. Record cache state, host load and peak memory. Preserve baseline optimizer environment deliberately and record it; if environment overrides defaults, resolve them explicitly before comparing.

Six concurrent runs can screen accuracy and feature activation. Their elapsed times are provisional because CPU, compilation, memory bandwidth and cache resources are shared. A speed recommendation requires later quiet-host, sequential, three-repeat paired measurements in rotating order with cold and warm timings separated. If resources are insufficient, narrow the screen or defer timings; do not interpret contention as an algorithm regression.

## Independent baseline acquisition

Use development fixtures `five_points_bigger_deltas`, `layered_manual_some_bad_points`, `seventh_hp3_dataset`. The last is an old HP3 fixture with machine-type alias `hangprinter_4`; do not infer current scene provenance from that alias. Remaining fixtures are validation data, including pressure-heavy cases and the explicit October HP4 fixture. Check `autocal/tools/regress_calibration_logs.py::DATASETS` and reference README for actual truth, bounds and arguments.

Each lane may reuse existing baseline stage artifacts only after verifying source hashes, resolved options, input snapshot and provenance. Otherwise capture its own baselines, at most one full-auto replay per development fixture (three total). Early/middle/final artifacts within those runs provide snapshots. Never use a mutated replay dataset as the untouched holdout.

The following optional command captures **one** baseline with copied data and lane-local logs/artifacts. Run it only after verifying your lane directory and notebook. It is a recipe for the researcher, not an experiment performed during handoff preparation. Select the other fixtures by changing `FLAG_FIXTURE` and run serially. It matches the regression runner's fixture arguments, including extra bounds, and explicitly disables collection.

```bash
export FLAG_FIXTURE=five_points_bigger_deltas
.venv/bin/python - <<'PY'
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from autocal.tools.regress_calibration_logs import DATASETS, _flatten_cli_args

repo = Path.cwd()
root = Path(os.environ['FLAG_RUN_ROOT']).resolve()
spec = next(s for s in DATASETS if s.name == os.environ['FLAG_FIXTURE'])
trial = root / 'baseline' / spec.name
trial.mkdir(parents=True, exist_ok=False)
source = repo / 'autocal/data/references' / f'{spec.name}.json'
dataset = trial / 'sweeps.json'
shutil.copy2(source, dataset)
command = [sys.executable, '-m', 'autocal.autocal', '--sim', '--no-collect',
    '--machine-type', spec.machine_type, '--dataset', str(dataset),
    '--find-radii', spec.find_radii, '--base-radii', spec.base_radii,
    '--buildup-factor', spec.buildup_factor, *_flatten_cli_args(spec.extra_args),
    '--full-auto-log', str(trial / 'full-auto.jsonl'),
    '--stage-artifacts', str(trial / 'stages')]
manifest = {'command': command, 'fixture': spec.name,
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
    'autocal_sources': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted((repo / 'autocal').glob('*.py'))},
    'environment': {k: os.environ[k] for k in (
        'AUTOCAL_OPTIMIZER_MODE', 'AUTOCAL_DISABLE_JAX_OBJECTIVE',
        'AUTOCAL_JAX_LBFGSB_MODE', 'JAX_COMPILATION_CACHE_DIR',
        'JAX_ENABLE_COMPILATION_CACHE', 'XLA_FLAGS', 'OMP_NUM_THREADS',
        'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS',
        'PYTHONHASHSEED') if k in os.environ}}
# Review environment names/values locally before sharing; do not add credentials.
(trial / 'manifest.json').write_text(json.dumps(manifest, indent=2))
with (trial / 'console.log').open('w') as log:
    try:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                timeout=900)
        status = {'returncode': result.returncode}
    except subprocess.TimeoutExpired:
        status = {'timeout_seconds': 900}
(trial / 'status.json').write_text(json.dumps(status, indent=2))
print(trial, status)
PY
```

Add package versions, source dirty diff and the actual start/end timestamps to provenance before comparisons. Inspect emitted artifact kinds and options. Fit/plan replay uses `python -m autocal.tools.replay_stage`; use absolute input/output paths. Plan artifacts retain dataset paths: point any required dataset reference to your lane-local copy and preserve the original hash. Write each trial to a new name; stage writes use temporary files and must never share an output path.

## Measurements and verdicts

For each executed trial record exact command/override, input and source hashes, resolved options, activation, cache state, timeout/crash/invalid fit, evaluations, accepted points/sweeps, information rank/conditioning, physical anchor RMSE/max error, spool error, fixed held-out prediction median/p95/max/failures, selected history iteration, terminal reason, and elapsed/compilation/stage time where applicable. Mark inapplicable metrics explicitly. Keep failures in the denominator.

Use a fixed untouched sweep set and each candidate's own spool model with consistent observation conventions. No held-out refitting. Lock the holdout rule before comparison and keep candidate selection on development data; report any unavoidable model/metadata mismatch. Compare fitted parameters in the same coordinate gauge. Synthetic controls and recorded simulation remain distinct; neither is hardware evidence. Training scores alone cannot identify a winner.

The original proposed screen requires no extra invalid fits/prediction failures; no fixture with an anchor RMSE increase exceeding the larger of 0.1 mm and 5%; no >5% held-out p95 deterioration; and either a clear accuracy gain or >=10% median speed improvement after quiet-host confirmation. Calibrate and freeze tolerances from baseline repeatability before validation. Report all worst cases. If metrics or sample size cannot support a decision, report inconclusive and give the next smallest discriminating experiment.

Each lane produces `research.md`, `abstract.md`, `report.md`, `trials.json` or CSV, commands/overrides, `ideas.md`, and `proposed-changes.md` under its own session root. Provide a per-flag disposition and explain daily/default/explicit use separately. No flag must be removed merely to count as a successful investigation.

## Integration after all six finish

A later integrator regenerates the research index, reads all six reports, and checks common source/data/holdout contracts. Do not combine six individually attractive changes blindly. First validate each finalist alone against the same baseline on untouched fixtures, then test relevant pairs: solver budgets × spool budgets; filtering × scale/objective schedule; physical corrections × sigma/raw-mean; ranking × stopping; candidate density × exclusion/spacing. Run sequential, isolated full regressions for the final candidate set. The runner has `--jobs 1` and dataset selectors, but no generic `--extra-args`; use lane-local command harnesses for variants.

Fresh collection is a subsequent bounded experiment with independent production headless worlds, preflight and recording none/events. Never share a continuing supervisor/world between concurrent lanes. Initial six-lane scope requires no movement, browser or hardware. If a later implementation changes physics, motors, cable transport or scene dynamics, the required `npm run test:physics` gate still applies before delivery.
