# Autocal flag investigation and experiment handoff

Status: static investigation complete; numerical experiments proposed, not run.
Date: 2026-10-22.

The CLI mixes physical assumptions, optimizer budgets, heuristic ranking, collection policy and diagnostics. The audit inventories 92 explicit option spellings from the user's document and searches 13,863 local evidence/source files. Literal occurrences establish that an option has been mentioned, not that a feature ran or helped. No default was established as optimal. The most useful first experiments are scale-polish placement, filtering/noise normalization, solver budgets and ranking penalties. Preserve the user's KEEP decisions. This investigation changes documentation only and provides a bounded sequence of replay experiments before fresh collection.

## Evidence and interpretation

[Inventory](inventory.md) gives every option, KEEP status, parser locations/default expressions, counts of matching log and other files, and an example. [Raw evidence](usage.json) retains up to four excerpts per spelling and the source revision. [Audit script](audit_usage.py) reproduces it from the repository root:

```bash
.venv/bin/python research/autocal-flag-audit/audit_usage.py
```

The search covers `autocal/data`, `output/research`, `research`, `autocal/tests`, `autocal/tools`, and `autocal/shotgun.conf`, including ignored local files. Only `.log`, `.jsonl`, `.md`, `.py`, `.conf` files up to 8 MB are read. It excludes this audit directory. Counts include help text, tests, comments and repeated archived runs. JSON configuration keys and implicit defaults are not counted. Aliases and argparse-generated negative options may lack an independent definition row. Thus zero means no literal evidence in this corpus, never proof of non-use. Examples are search leads; inspect the full command and settings before treating them as executed runs. Large logs, datasets, external hardware invocations and unrecorded daily use remain outside this search.

The original [KEEP annotations](../../autocal-flags.md) remain authoritative. A KEEP option still needs its interactions understood; it is excluded from removal proposals. All other options are covered below. The generated inventory is the exhaustive spelling checklist; the tables group coupled options to avoid pretending they can all be tuned independently.

Read these implementation entry points when running experiments:

- [Main parser and variant parser](../../autocal/_autocal_common.py): defaults, inheritance and validation. `None` in a full-auto variant generally means inherit, rather than disable.
- [Orchestrator](../../autocal/autocal.py): replay/collection decisions, consensus, patience and threshold stopping.
- [Fit stage](../../autocal/fit_stage.py), [filter passes](../../autocal/filter_pass.py), [alternating refinement](../../autocal/alternating_refinement.py), [scale polish](../../autocal/scale_polish.py): fitting and spool estimation.
- [Planning stage](../../autocal/planning_pass.py), [active learning](../../autocal/active_learning.py), [history selection](../../autocal/history_selection.py): information estimates and final candidate selection.
- [Spool model](../../autocal/spool_model.py), [ellipse cost](../../autocal/ellipse_cost.py), [ellipse fitting](../../autocal/ellipse_fitting.py), [flex](../../autocal/flex.py): physical assumptions and residual units.

The checked-in [shotgun configuration](../../autocal/shotgun.conf) actively lists all three sweep metrics with `sigma-source point`; origin/min alternatives are commented. This is concrete configured use, not evidence that all three metrics deserve permanent flags. The regression runner explicitly exercises radius fitting and several physical datasets. Default-enabled filtering is exercised even when no option spelling appears in a command.

## What can be simplified first

No numerical flag has enough evidence here for immediate unconditional deletion. Do not confuse an inherited argument or a conditionally inactive feature with dead code.

The lowest-cost cleanup candidates are positive switches that merely restate defaults (`--pointwise-filtering`, `--pointwise-global-mad`, `--sweep-wise-filtering`, `--use-noise-mean`, `--robust-debug`). They may still override an inherited full-auto setting. Before removing a spelling, test both the main parser and variant parser precedence and search callers; replace paired forms with one consistent boolean convention if needed. This reduces CLI surface without claiming a numerical improvement. Diagnostics belong in a documented output group, not a tuning guide.

`--threshold` is a naming/documentation problem, not a demonstrated no-op. It reaches ellipse validity: `fit_single_sweep` compares RMS residual to the threshold in `ellipse_fitting.py`. `EllipseCostFunction` also stores it, but that assignment alone does not establish its effect there. Main CLI default 250 differs from low-level API defaults 0.01. Trace which residual coordinates and normalization each call uses before attempting to hard-code it.

The most plausible eventual hard-coding candidates are `--scale-fix` placement, `--fit-structure` penalty combinations, `--theta0-mode` when acquisition semantics are known, and optimizer/filter schedules. All need the experiments below. Physical calibration assumptions (`--flex`, `--low-anchor-z`, tension projection) cannot be selected solely by winning on one fixture family.

## Flag-by-flag assessment

In these tables, “test” means not tested in this investigation. Defaults and literal-use evidence for each individual spelling are in the inventory. “Retain” is a reason to keep the capability during investigation, not an added user KEEP annotation.

### Fit model and numerical solver

| Options | Purpose, activation and judgment | Interactions and next test |
| --- | --- | --- |
| `--solve-restarts`, `--solve-iterations` | Defaults 4 and 400. Real convergence/escape budgets; retain until adaptive termination replaces manual budgets. | E1: restarts 1/2/4, iterations 100/200/400, matched starts and cold/warm timing. Coupled to outer/inner spool budgets and robust passes. |
| `--solve-optimizer` | Default lbfgsb; lm/trf are different solve paths, not guaranteed equivalent basins. | E1: compare physical error, bounds, failed/invalid restarts and evaluations; retain alternatives for diagnosis until dominated. |
| `--trf-solver-fallback`, `--no-trf-solver-fallback` | Disabled default. Conditional retry, so a run where it never triggers says nothing about benefit. | E1: use failed lbfgsb starts, log trigger count and final consensus. Could improve recovery or silently change selection. |
| `--optimizer-mode` | Exact JAX Jacobian / JAX finite differences / non-JAX finite differences. Environment can change default. Legacy remains a useful independent numerical control. | E1: record environment and compile counts; verify objective/gradient agreement before timing. Prior native compiler crashes make process failures evidence to preserve. |
| `--threshold` | Ellipse validity gate, default 250; keep pending units/activation audit. | E2: 25/250/2500, first count sweeps accepted/rejected at each call site, then fits. Interacts with point count, residual construction, filtering and underconstraint recovery. |
| `--flex`, `--spring-k-multiplier` | Elastic correction; multiplier default 1 only builds the cost's flex model when flex is enabled. Retain physical capability. A free stiffness knob can absorb geometry/model error. | E5: flex off/on; on with multiplier 0.5/1/2. Need known stiffness/tension records, varied preload and held-out tensions. Do not optimize multiplier on anchor score alone. |
| `--low-anchor-z` | Fixes shared HP4 low-anchor plane; strong dimensional assumption that can improve conditioning or introduce bias. | E5: correct known height, deliberately offset height, unconstrained baseline. Cross with radius fitting. Never infer hardware height from a simulator-only optimum. |
| `--r0-prior-sigma-mm`, `--b-prior-sigma` | Regularize ambiguous spool/geometry estimates. Inactive without corresponding fitted parameter (fit stage emits warnings). Buildup prior auto-default 0.1 is substantive. | E3: radius prior None/1/5 mm; buildup prior 0.03/0.1/0.3. Include truth away from prior mean and bounds. Retain unless identifiability improves enough to dispense with priors. |
| `--spool-outer-iters`, `--spool-inner-iters` | Default 3 and 30. Alternating refinement budgets, conditional on spool fitting. | E3: outer 1/3/6, inner 10/30/60 after E1; report actual iterations/evaluations. Search fewer combinations initially, then the interaction at winning budgets. |
| `--theta0-mode` | Default zero; infer requires raw angles/base lengths and estimates per-axis medians. This is an acquisition-origin assumption. | E5: origin-shifted copies with consistent metadata. Compare zero/infer only where inference inputs exist. Coupled to gearing, zero-tension angles and spool curvature. Prefer automatic metadata-based origin handling if justified. |
| `--scale-fix` | Default 3. Level 1 affects coupled alternating refinement; level 2 final polish on last filter pass; level 3 polish on every pass. | E3: off/1/2/3, then 1,3 if level 1 helps. Coupled to fitted radii, priors, schedules, ranking and scale ambiguity. Candidate for replacing numeric IDs with one validated policy. |
| `--fit-structure` | Default 3. Enables tail-ratio, sweep-bias-directionality and/or tail-factor penalties in layered internal scoring. Selection policy needs separate evaluation from optimizer quality. | E4: off/1/2/3 on frozen candidates; then combinations only for individually useful terms. Count actual score changes and selection regret. Good candidate for hard-coding/removal of losing terms. |
| `--filter-schedule` | Default warmup,warmup,warmup,dynamic. Controls pass sequence; each pass can repeat expensive fitting. | E2/E3: warmup,dynamic vs default vs dynamic. Record weights, discarded sweeps, stage count. Cannot assume a shorter schedule is equivalent to fewer outer iterations. |
| `--objective-schedule` | Default pointwise four times. Prefit, forward-model pointwise, position reconstruction have different modeling/numerical costs. | E3/E5: default vs prefit,pointwise,pointwise,pointwise vs pointwise,pointwise,pointwise,position. Check requested versus resolved pass schedules. Retain position idea for a better implementation if current solve is unstable/slow. |

### Noise and robust filtering

| Options | Purpose and judgment | Interactions and next test |
| --- | --- | --- |
| `--pointwise-residual` | Sampson approximation default; Euclidean alternative may improve geometric fidelity at extra cost. No evidence of global superiority here. | E2: sampson/euclidean on clean and distorted sweeps with matched normalization; inspect residual agreement near high curvature/degeneracy. |
| `--pointwise-filtering`, `--no-pointwise-filtering` | Default enabled. Protects against isolated bad points; can erase informative extremes or model mismatch. | E2: on/off crossed with sweep filtering; inspect point retention by sweep/delta and true residual. |
| `--pointwise-global-mad`, `--pointwise-per-sweep-mad` | Global default pools scale; per-sweep adapts heteroscedasticity but can normalize away an entirely bad sweep. | E2: global/per-sweep with one corrupted sweep and varied noise. Cross sweep rejection and sigma source. |
| `--sweep-wise-filtering`, `--no-sweep-wise-filtering`, `--sweep-metric` | Default enabled/outlier_ratio. Metrics mad/median_abs/outlier_ratio all appear in active shotgun config. Protects against coherent bad sweeps, potentially destroys observability. | E2: three metrics after point filtering baseline; track information rank, retained sweeps and false rejection. Disable sweep filtering as control. |
| `--sigma-source` | Auto/point/origin/min select available normalization evidence; changing the unit of normalized residual changes robust decisions and rankings. | E2: log actual source coverage and fallbacks, then auto/point/origin/min on identical observations. Do not equate auto with point without inspecting each dataset. |
| `--sigma-floor-mm`, `--sigma-used-mm` | Overrides base model floor/final normalization. Final override can overwhelm sensor/model evidence. Retain as diagnostic controls pending calibrated noise model. | E2: unset vs 0.1/1 mm; cross only promising values with sigma source and line width. Inspect weight saturation; do not reward a score merely made smaller by larger sigma. |
| `--use-noise-mean`, `--use-raw-lengths` | Default noise mean; changes observations, not just weights. Noise means may reduce random error while preserving settling bias. | E2/E5: paired observations containing both representations, compare untouched physical predictions and mean/raw discrepancy by tension. |

KEEP `--line-width` participates in this same noise model. KEEP radius/buildup modes, seeds and bounds must be recorded in every fit comparison; a noise-policy win with radii fixed may disappear with scale free.

### Active collection and stopping

| Options | Purpose and judgment | Interactions and next test |
| --- | --- | --- |
| `--sparse-recovery` | Opt-in endpoint-preserving seed when underconstrained. Retain recovery idea; it is not a general permission to thin all data. | E6: paired replay prefixes that actually hit underconstraint; count triggers, recovery and final error. Include endpoints corrupted by outliers. |
| `--sweep-points` | Collector/histogram forwarding, unset by default. Information versus movement/settling cost; retain capability. | E6: fresh matched collection at 10/20/40 points, subject to actual collector constraints. Six observations fail the current min_points=10 guard; older thinning studies already found accuracy regressions. |
| `--max-steps` | Default 20; operational collection budget, not a fit accuracy knob. | E6: 10/20/40 on recorded prefixes first; log terminal reason and unused suffix. Keep a hard budget even if automatic stopping improves. |
| `--stop-cost`, `--stop-std-mm` | Unset by default. OR acceptance thresholds, subject to underconstrained handling. Conditional covariance can be overconfident with model error. | E6: derive thresholds from held-out error calibration on a development set, then freeze for validation. Include early stable but biased fits. |
| `--patience` | Default 3 non-improving full-auto iterations; interacts with consensus/score and noise. | E6: 1/3/5, quantify saved points and false acceptance. Paired fit consensus can still agree on biased geometry. |
| `--candidate-deltas`, `--candidate-count`, `--delta-min`, `--delta-max` | Explicit deltas override generated grid (default count 41); range can derive from travel and observations. Physical exploration policy, not merely speed. | E7: grid 21/41/81 with same range; then range separately. Measure candidate count, ranking time, feasible choices and realized information. Retain range controls for differing machines. |
| `--fd-eps-mm`, `--regularization` | Defaults 1 mm and 1e-6. Used in information/covariance and planning, not the optimizer's finite-difference step flag. | E7: eps 0.1/1/10, ridge 1e-8/1e-6/1e-4. Compare singular values and candidate order; separate numerical stabilization from real information. |
| `--no-exclude-existing`, `--existing-tol-mm`, `--min-fixed-delta-spacing-mm` | Exclusion enabled, tolerance 10 mm and same-sign spacing 20 mm. Avoid repeats, but replication may help noise estimates. | E7: tolerance 1/10/20, spacing 0/20/40; count feasible candidates, repeats and actual prediction improvement. Coupled to grid density/range. |
| `--top-k` | Default 10; shortlist/report setting. Trace whether downstream collection consumes more than first choice before treating as a performance knob. | E7: 1/10 with frozen fit; compare first choice, written config and work. If only reporting changes, document as output option. |
| `--project-zero-tension` | Collector correction requiring a credible tension/elastic model. Retain physical idea, particularly for preload bias. | E5: off/on across varied preload, crossed with flex. Establish whether corrections double-count elasticity; compare inferred zero-load lengths and held-out motion. |

### Run variants, output and recording

| Options | Judgment and bounded check |
| --- | --- |
| `--full-auto-run`, `--shotgun` | Keep experimental comparison capability for now. Shotgun can multiply fit cost and exploit a misaligned score. E4 compares each variant alone versus ensemble on the same history; log winner frequency, physical regret and wall time. Remove losing bundled variants only after validation. |
| `--stage-artifacts` | Retain for this investigation: freezes fit/history payloads for isolated replay. Opt-in artifact serialization costs belong outside solver timing. |
| `--full-auto-log` | Retain provenance output; path control has no numerical optimum. Verify log settings reconstruct actual run and variant inheritance. |
| `--verbose`, `--robust-debug`, `--no-robust-debug`, `--debug-sweep-actions` | Diagnostic verbosity. Move to a consistent logging scheme if simplifying CLI. Verify enabling/disabling logs preserves numerical results; time only when I/O is material. Robust debug currently defaults on. |
| `--residuals-csv`, `--report`, `--plot-residual-histogram` | Diagnostic artifacts, not solver tuning. Retain export capability; document units and which final stage produced residuals. Histogram gamma fit does not demonstrate calibrated errors. |
| `--output-with-explanations`, `--write-sweep-config` | Output behavior/path controls. Retain capability; explain conditional covariance and rank deficiency rather than presenting std as physical accuracy. |
| `--extended-reference-ws`, `--extended-reference` | Recorder transport/lifecycle controls; overlap user KEEP recording controls. Trace callers and owned/external lifecycle before consolidation. Use events/none for numerical sweeps; recording timings in prior research are workload-specific, not fit speedups. |

Aliases `--recording` and `--record` inherit KEEP from `--extended-recording`;
`--simulation` inherits KEEP from `--sim`.

KEEP operational options (`--help`, machine, dataset, replay/collection, firmware/configuration, simulator lifecycle, motors/substeps, collector forwarding, speedup, recording/headless) remain unchanged. Their main experimental role is to freeze provenance and acquisition semantics. No new experiment should tune physics substeps to rescue a calibration-policy comparison.

## Six parallel handoffs

The [parallel launch index](handoffs/README.md) splits this assessment into six
independent researcher tasks, with exclusive flag ownership, lane-local artifacts,
resource limits and an integration plan. Use that execution contract for the
six-way run; it supersedes the sequential scheduling and shared artifact assumptions
below. Each lane can capture its own baseline without waiting for another lane.
The original E1–E7 labels below remain topic references, with E6/E7 in lane 06.

## Experiments in execution order

All E1–E7 below are proposed, not tested. The budget is a suggested ceiling for the next researcher, not authorization to run every cell automatically. Start with static activation checks; stop a family if its feature never activates on the chosen data. Do not launch full Cartesian products across families.

1. Select three development fixtures: `five_points_bigger_deltas` (sparse geometry), `layered_manual_some_bad_points` (outliers), `seventh_hp3_dataset` (HP4/3D fixture as labeled by runner). Confirm machine type from `DATASETS`, not filename. Keep remaining reference fixtures for validation, including pressure-heavy data. Record file SHA-256, revision/dirty diff, Python/NumPy/SciPy/JAX versions, CPU threads, optimizer environment and parameter truth from the runner/reference README.
2. Capture baseline frozen fit/history artifacts at early, middle and selected final iterations. Use identical snapshots for each cell. First spend at most 12 fit replays total on E3 scale-fix off/1/2/3 over three representative snapshots. Then at most 12 on E2 filtering activation and at most 12 on E1 budgets. Expand only a promising or inconclusive family, with a stated extra budget. Ranking-only E4 and planning-only E7 are cheaper and can use all saved histories before refitting.
3. Repeat timing for finalists three times in rotating order on one quiet host; report cold process/compilation and warm execution separately. Match optimizer restarts/seeds/starts and persist any nondeterminism. A crash, timeout or invalid result counts against reliability and must remain in the result table.
4. Run full replay regression only for baseline and one or two finalists, with one worker for interpretable timings. Never regenerate references as part of evaluating a candidate. Reference equality proves compatibility, not optimality; a deliberate accuracy improvement may fail reference tolerance and must be assessed separately.
5. E5 physical-model tests require matching metadata and synthetic truth controls; clean synthetic results do not establish recorded or hardware accuracy. E6/E7 movement gains need later fresh headless production collection since replay only reveals available recorded sweeps. Use no recording or events initially, a bounded collection budget, fresh worlds and matched machine configurations. Follow `research/AGENTS.md` and preflight before live collection. No hardware result is claimed here.

### Commands and stage option mapping

From the repository root, create a new output directory and copy datasets there before replay. `autocal.py` updates datasets/logs; never point an experimental run at the reference file itself.

```bash
mkdir -p output/research/flag-trial-baseline
cp autocal/data/references/five_points_bigger_deltas.json output/research/flag-trial-baseline/sweeps.json
.venv/bin/python autocal/autocal.py --machine-type slideprinter \
  --dataset output/research/flag-trial-baseline/sweeps.json --no-collect \
  --find-radii global --base-radii 30.0 --buildup-factor 0.636619 \
  --stage-artifacts output/research/flag-trial-baseline/stages
```

Use each fixture's `DatasetSpec` arguments for other data; the example is only for this fixture. Read generated artifact `kind` and `payload` before selecting paths. Stage replay expects JSON keyword names, not CLI strings:

```bash
.venv/bin/python -m autocal.tools.replay_stage fit PATH_TO_FIT_ARTIFACT \
  --options PATH_TO_OVERRIDE_JSON --output PATH_TO_NEW_RESULT
.venv/bin/python -m autocal.tools.replay_stage plan PATH_TO_FIT_ARTIFACT \
  --options PATH_TO_PLANNING_JSON --output PATH_TO_NEW_PLAN
.venv/bin/python autocal/tools/regress_calibration_logs.py --jobs 1 \
  --dataset-name five_points_bigger_deltas \
  --dataset-name layered_manual_some_bad_points
```

Example fit override JSON is `{"scale_fix": [2]}`; off is `[]`. Filter/objective schedules should use saved normalized lists. CLI `--threshold` maps to `residual_threshold`, `--flex` to `use_flex`, `--pointwise-residual` to `pointwise_residual_mode`, `--line-width` to `line_width`, and spool budgets to `spool_outer_iters`/`spool_inner_iters`. Override only keys accepted by `fit_ellipse_dataset`; inspect its signature. Optimizer mode and fallback are process settings, not fit-stage keyword arguments: use the CLI for those comparisons or explicitly reproduce `_apply_optimizer_mode_env` and fallback configuration in a separate harness.

History `evaluate` requires original history artifacts; `rank` requires evaluated history and accepts no overrides. Fit-structure affects scoring during fit construction, so recompute those candidate metrics with the alternate policy before reranking; blindly passing `fit_structure` to history replay is invalid. Planning overrides use `candidate_count`, `fd_eps_mm`, etc. The regression runner has no general extra-flags override: use a small separate command harness based on its `DatasetSpec` arguments for variant runs, preserve exact commands and compare its parsed metrics. Do not assume a nonexistent `--extra-args` option.

### Required result table and acceptance rule

Store one JSON/CSV row per fixture/snapshot/value/repetition with: experiment ID; exact command/override; input hash; resolved options; feature activation; wall/compile/stage times; evaluations; validity/failure; accepted points/sweeps; information rank/condition; anchor RMSE and maximum anchor error in the same coordinate gauge; radius/buildup error; held-out prediction median/p95/max and failed predictions; selected candidate/iteration; collection points; terminal reason. Store training cost and score too, but do not use either alone as the winner criterion. Evaluate all alternatives on the same held-out sweeps using their own spool model and observation conventions; never refit to held-out data.

Proposed screening rule: no additional invalid fits or prediction failures, no fixture with >0.1 mm or >5% increase in anchor RMSE (use the larger tolerance), no >5% deterioration in held-out p95, and either >=10% median wall-time improvement or a clear error reduction. These are proposed engineering thresholds, not measured noise limits; set final tolerances before validation after inspecting baseline repeatability. Report all per-fixture regressions and worst cases even when aggregates improve. For hardware without anchor truth, use independently measured geometry or untouched motion predictions and label the missing ground truth.

If a value dominates on development and validation, hard-code it only after caller/compatibility review and remove obsolete branches/tests together. If values trade accuracy for time, retain a documented budget/policy rather than burying another magic number. If nothing wins, record the negative result and keep the baseline.

## Ideas worth investigating beyond their current switches

- Scale handling has a sound target: alternating anchor/radius optimization can stall along a coupled scale direction. Uniform polish searches that direction, but a better training objective can still choose a physically wrong scale. E3 should plot physical error, held-out error, data cost and prior cost against scale. A persistent flat valley calls for independent scale information or a joint well-scaled solve, not more polish iterations. [Current implementation](../../autocal/scale_polish.py).
- Sweep-bias and tail diagnostics detect model mismatch, but hand-weighted multiplicative rank penalties can punish informative data or reward filtering. E4 should measure selection regret on frozen histories before modifying fit dynamics. A successful diagnostic need not be a successful ranking term. [Scoring implementation](../../autocal/_autocal_common.py).
- Position reconstruction may use physical geometry more directly than an ellipse approximation. Its current nested optimization could be costly or poorly conditioned. E5 should separate residual-model accuracy with known poses from latent-pose solver failure, then consider a joint latent-pose formulation. This idea remains untested here.
- Information-based movement planning should account for uncertain radii/buildup/stiffness rather than treating fitted nuisance parameters as certain. E7 first checks derivative/ridge sensitivity; a planner that remains confidently wrong under model mismatch needs nuisance-aware information, not a finely tuned epsilon.
- Uncertainty-based stopping is useful only when estimated uncertainty predicts unseen error. E6 should build coverage curves across prefixes, noise, outliers and model mismatch. Conditional covariance and agreement between two biased fits are insufficient guarantees.

## Earlier findings that constrain these tests

These are prior results, not new experiments in this audit:

- [Ranking, thinning and exact reuse](../../output/research/sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/report.md): score/physical-error disagreements and mixed thinning results. Consult before using fewer points as a default speed optimization.
- [Compilation and numerical controls](../../output/research/sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md): recorded scale changes did not deliver the hoped-for general compilation benefit; padding/fallback changes could alter accuracy. Persistent caching helped measured repeated runs, with stated limits.
- [Settling and point counts](../../output/research/sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): reduced point counts harmed some recorded fits; six-point fits fail the current minimum guard.
- [Research topic guide](../catalog.md): earlier held-out ranking, bounded-update and model-validation investigations, including negative results.
- [Reference provenance](../../autocal/data/references/README.md) and [regression runner](../../autocal/tools/regress_calibration_logs.py): fixture assumptions, simulated truth, current comparisons and runner argument limitations.
- [Recording benchmark](../autocal-recording-performance.md): measured recording costs and why they do not imply full-autocal speedups.

No runtime paths were removed, no new numerical superiority was claimed, and no physics change or physics experiment was made.
