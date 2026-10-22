usage: autocal.py [-h] --machine-type {slideprinter,hangprinter_3,hangprinter_4,hangprinter_5,cubecorners,skycam,hp3,hp4,v3,v4,four_high_anchors,cube_corners} [--no-collect] [--sparse-recovery] [--dataset DATASET] [--sweep-points SWEEP_POINTS]
                  [--plot-residual-histogram] [--max-steps MAX_STEPS] [--stop-cost STOP_COST] [--stop-std-mm STOP_STD_MM] [--full-auto-run FULL_AUTO_RUN] [--shotgun] [--stage-artifacts STAGE_ARTIFACTS] [--full-auto-log FULL_AUTO_LOG] [--verbose]
                  [--patience PATIENCE] [--solve-restarts SOLVE_RESTARTS] [--solve-iterations SOLVE_ITERATIONS] [--solve-optimizer {lbfgsb,lm,trf}] [--trf-solver-fallback | --no-trf-solver-fallback] [--optimizer-mode {fast,fast-fd,legacy}]
                  [--threshold THRESHOLD] [--spring-k-multiplier SPRING_K_MULTIPLIER] [--flex] [--find-radii [{off,global,per-anchor}]] [--find-buildup-factor [{off,global,per-anchor}]] [--base-radii BASE_RADII] [--buildup-factor BUILDUP_FACTOR]
                  [--line-width LINE_WIDTH] [--sigma-floor-mm SIGMA_FLOOR_MM] [--sigma-used-mm SIGMA_USED_MM] [--low-anchor-z LOW_ANCHOR_Z] [--r0-bounds R0_BOUNDS] [--b-bounds B_BOUNDS] [--r0-prior-sigma-mm R0_PRIOR_SIGMA_MM]
                  [--b-prior-sigma B_PRIOR_SIGMA] [--spool-outer-iters SPOOL_OUTER_ITERS] [--spool-inner-iters SPOOL_INNER_ITERS] [--theta0-mode {infer,zero}] [--scale-fix SCALE_FIX] [--fit-structure FIT_STRUCTURE] [--filter-schedule FILTER_SCHEDULE]
                  [--objective-schedule OBJECTIVE_SCHEDULE] [--pointwise-residual {sampson,euclidean}] [--pointwise-filtering] [--no-pointwise-filtering] [--pointwise-global-mad] [--pointwise-per-sweep-mad] [--sweep-wise-filtering]
                  [--no-sweep-wise-filtering] [--sweep-metric {mad,median_abs,outlier_ratio}] [--sigma-source {auto,point,origin,min}] [--use-noise-mean | --use-raw-lengths] [--robust-debug] [--no-robust-debug] [--residuals-csv RESIDUALS_CSV]
                  [--report] [--candidate-deltas CANDIDATE_DELTAS] [--candidate-count CANDIDATE_COUNT] [--delta-min DELTA_MIN] [--delta-max DELTA_MAX] [--fd-eps-mm FD_EPS_MM] [--regularization REGULARIZATION] [--no-exclude-existing]
                  [--existing-tol-mm EXISTING_TOL_MM] [--min-fixed-delta-spacing-mm MIN_FIXED_DELTA_SPACING_MM] [--top-k TOP_K] [--output-with-explanations] [--write-sweep-config WRITE_SWEEP_CONFIG]
                  [--physics-substeps {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16}] [--open-loop-motors] [--collector-args ...] [--firmware {rrf,klipper}] [--config CONFIG] [--rrf-config RRF_CONFIG] [--klipper-config KLIPPER_CONFIG] [--sim]
                  [--keep-sim-alive] [--hp-sim-reset] [--project-zero-tension] [--debug-sweep-actions] [--speedup SPEEDUP] [--extended-recording] [--extended-reference-ws URL] [--extended-reference] [--recording-mode {none,events,sampled,full}]
                  [--headless-sim]

Non-interactive autocal active-learning loop.

options:
KEEP  -h, --help            show this help message and exit
KEEP  --machine-type {slideprinter,hangprinter_3,hangprinter_4,hangprinter_5,cubecorners,skycam,hp3,hp4,v3,v4,four_high_anchors,cube_corners}
                        Machine type (slideprinter | hangprinter_3 | hangprinter_4 | hangprinter_5 | cubecorners | skycam | hp3 | hp4 | v3 | v4 | four_high_anchors | cube_corners)
KEEP  --no-collect          Do not collect new sweeps; stop when replayed data is exhausted.
  --sparse-recovery     Use an endpoint-preserving sparse seed when underconstrained recovery needs it.
KEEP  --dataset DATASET     Dataset file updated each iteration (default: autocal/data/default_dataset.json).
  --sweep-points SWEEP_POINTS
                        Points per sub-sweep (passed to the collector and histogram plot).
  --plot-residual-histogram
                        Write residuals CSV next to the dataset and render a histogram PNG (gamma fit included).
  --max-steps MAX_STEPS
                        Maximum active-learning iterations
  --stop-cost STOP_COST
                        Optional stop condition on ellipse cost
  --stop-std-mm STOP_STD_MM
                        Optional stop condition on max parameter std
  --full-auto-run FULL_AUTO_RUN
                        Full-auto run override flags (repeatable). Example: --full-auto-run "--sweep-metric mad --use-raw-lengths"
  --shotgun             Append full-auto runs from shotgun.conf.
  --stage-artifacts STAGE_ARTIFACTS
                        Save frozen fit/history JSON artifacts for independent stage experiments (opt-in).
  --full-auto-log FULL_AUTO_LOG
                        Write full-auto JSONL logs to this path (default: <dataset>.full_auto_log.jsonl).
  --verbose             Enable verbose iteration output, including anchors, M666, and rank scoring.
  --patience PATIENCE   Number of non-improving full-auto iterations to allow (default: 3).
  --solve-restarts SOLVE_RESTARTS
  --solve-iterations SOLVE_ITERATIONS
  --solve-optimizer {lbfgsb,lm,trf}
                        Anchor solver: lbfgsb (default), trf, or lm.
  --trf-solver-fallback, --no-trf-solver-fallback
                        Retry unsuccessful or invalid lbfgsb restarts with bounded trf (default: disabled; can change the final consensus).
  --optimizer-mode {fast,fast-fd,legacy}
                        Optimization mode: 'fast' (JAX exact Jacobian), 'fast-fd' (JAX value + SciPy finite differences), or 'legacy' (no JAX + SciPy finite differences).
  --threshold THRESHOLD
  --spring-k-multiplier SPRING_K_MULTIPLIER
  --flex
KEEP  --find-radii [{off,global,per-anchor}]
                        Spool-radius fit mode: off | global | per-anchor (default: off).
KEEP  --find-buildup-factor [{off,global,per-anchor}]
                        Buildup-factor fit mode: off | global | per-anchor (default: off).
KEEP  --base-radii BASE_RADII
                        Comma-separated base radii in mm (defaults to config m666_adjusted_by_data_collector/m666_before_data_collection R).
KEEP  --buildup-factor BUILDUP_FACTOR
                        Use this buildup factor k in the model transform (if omitted, falls back to config m666_adjusted_by_data_collector/m666_before_data_collection Q, else 0).
KEEP  --line-width LINE_WIDTH
                        Estimated line width in mm used by the layered noise model (default: 1.0).
  --sigma-floor-mm SIGMA_FLOOR_MM
                        Override the base sigma floor (mm) used in the composite noise model.
  --sigma-used-mm SIGMA_USED_MM
                        Override the final sigma used for pointwise normalization (mm).
  --low-anchor-z LOW_ANCHOR_Z
                        For hangprinter_4, fix the shared low-anchor Z plane to this value in mm.
KEEP  --r0-bounds R0_BOUNDS
                        Bounds for fitted r0 in mm as 'min,max' (default [base_radius, 1.5*base_radius]).
KEEP  --b-bounds B_BOUNDS   Bounds for fitted buildup factor as 'min,max' (default: 0,1).
  --r0-prior-sigma-mm R0_PRIOR_SIGMA_MM
                        Gaussian prior sigma (mm) around base radii during radius fitting.
  --b-prior-sigma B_PRIOR_SIGMA
                        Gaussian prior sigma for buildup-factor fitting (auto default: 0.1 when fitting k).
  --spool-outer-iters SPOOL_OUTER_ITERS
                        Outer iterations for spool parameter refinement.
  --spool-inner-iters SPOOL_INNER_ITERS
                        Inner optimizer iterations per spool refinement step.
  --theta0-mode {infer,zero}
                        Theta offset mode for spool model: infer | zero (default: zero).
  --scale-fix SCALE_FIX
                        Enable scale-coupling fixes by id (comma-separated): 1,2,3 (default: 3). Use 'off' to disable.
  --fit-structure FIT_STRUCTURE
                        Enable fit-structure rank penalties by id (comma-separated): 1=tail_ratio, 2=sweep_bias_directionality, 3=tail_factor (default: 3).
  --filter-schedule FILTER_SCHEDULE
                        Explicit filter-pass schedule as comma-separated words or numbers. Words: warmup,dynamic,constant. Numbers: 0=warmup, 1=dynamic, 2=constant. Default: warmup,warmup,warmup,dynamic.
  --objective-schedule OBJECTIVE_SCHEDULE
                        Explicit spool objective schedule as comma-separated words or numbers. Words: prefit,pointwise,position. Numbers: 0=ellipse_prefit, 1=pointwise_forward_model, 2=position_reconstruction. Default: 1,1,1,1.
  --pointwise-residual {sampson,euclidean}
                        Pointwise residual metric (default: sampson)
  --pointwise-filtering
                        Enable GNC-IRLS style pointwise filtering (default).
  --no-pointwise-filtering
                        Disable pointwise filtering.
  --pointwise-global-mad
                        Use a single global MAD scale for pointwise filtering (default).
  --pointwise-per-sweep-mad
                        Use a per-sweep MAD scale for pointwise filtering.
  --sweep-wise-filtering
                        Enable sweep-wise outlier rejection (default).
  --no-sweep-wise-filtering
                        Disable sweep-wise outlier rejection.
  --sweep-metric {mad,median_abs,outlier_ratio}
                        Per-sweep metric used by sweep-wise filtering (default: outlier_ratio).
  --sigma-source {auto,point,origin,min}
                        Sigma source for normalization: per-point, origin, or fallback min (default: auto).
  --use-noise-mean      Use encoder noise mean lengths when available (default).
  --use-raw-lengths     Use raw l_drive/l_sensor lengths (disable noise-mean datapoints).
  --robust-debug        Print diagnostics for robustness filtering (default).
  --no-robust-debug     Disable robustness filtering diagnostics.
  --residuals-csv RESIDUALS_CSV
                        Write pointwise residuals (approx mm) to CSV after the final GNC stage.
  --report              Write a PNG report (like calibrate.py)
  --candidate-deltas CANDIDATE_DELTAS
                        Comma-separated fixed deltas (mm)
  --candidate-count CANDIDATE_COUNT
                        Grid size when deltas not provided
  --delta-min DELTA_MIN
  --delta-max DELTA_MAX
  --fd-eps-mm FD_EPS_MM
  --regularization REGULARIZATION
  --no-exclude-existing
  --existing-tol-mm EXISTING_TOL_MM
                        Treat sweeps with the same fixed anchors/drive/sensor and fixed delta within this tolerance as duplicates
  --min-fixed-delta-spacing-mm MIN_FIXED_DELTA_SPACING_MM
                        Minimum spacing (mm) from previously collected fixed deltas with the same sign
  --top-k TOP_K
  --output-with-explanations
                        Include explanatory notes for rank deficiency and covariance behavior.
  --write-sweep-config WRITE_SWEEP_CONFIG
                        Write the suggested sweep config to this path (default: <dataset>.active_sweep_cfg.txt)
KEEP  --physics-substeps {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16}
                        Simulator physics steps per firmware tick (default: simulator setting, initially 1)
KEEP  --open-loop-motors    Use open-loop motors in the simulator instead of closed-loop motors.
KEEP  --collector-args ...  Extra args passed to autocal/control/cli/collect_sweep_data.mjs (after --collector-args)
KEEP  --firmware {rrf,klipper}
                        Firmware flavor to target (default: rrf)
KEEP  --config CONFIG       Generic simulator config path. Interpreted as --rrf-config when --firmware rrf, or as --klipper-config when --firmware klipper.
KEEP  --rrf-config RRF_CONFIG
                        Path to RRF simulator config file (env fallback: AUTOCAL_RRF_SIM_CONFIG)
KEEP  --klipper-config KLIPPER_CONFIG
                        Path to Klipper simulator config file (env fallback: AUTOCAL_KLIPPER_SIM_CONFIG)
KEEP  --sim, --simulation   Use simulator + hp-sim WebSocket bridge (default: talk to real firmware)
KEEP  --keep-sim-alive      Leave simulator process running after exit (only when --sim)
KEEP  --hp-sim-reset        Reset hp-sim once before the first sweep (only when --sim)
  --project-zero-tension
                        Project encoder readings to zero tension during each data point.
  --debug-sweep-actions
                        Enable sweep collector debug traces (force modes, waits, moves, data points).
KEEP  --speedup SPEEDUP     Forward this hp-sim speed scale to the collector (shortcut for --collector-args --speedup).
KEEP  --extended-recording, --recording, --record
                        Start and finalize an extended recorder automatically; output is derived from --dataset.
  --extended-reference-ws URL
                        Record timestamped logs and collector events in an --extended-reference flight recorder
  --extended-reference  Record headless physics and all autocal events in one RRD; starts an owned recorder unless a WebSocket URL is supplied
KEEP  --recording-mode {none,events,sampled,full}
                        Headless evidence: none (numerical artifacts/traces), events, sampled (physics stride 100), full (extended RRD, stride 10); geometry stays compact
KEEP  --headless-sim        Run simulated RRF and production JS physics/collector in Node without a browser; implies --sim

## Investigation, 2026-10-22

The KEEP annotations above are preserved. Start with the
[flag assessment and experiment handoff](research/autocal-flag-audit/report.md).
It covers purpose, activation, removal candidates, coupled tuning, limitations
in the underlying ideas, and bounded experiments with acceptance criteria.

The [complete inventory](research/autocal-flag-audit/inventory.md) records parser
defaults and local literal-use evidence for 92 option spellings. The
[raw excerpts](research/autocal-flag-audit/usage.json) and
[reproducible audit](research/autocal-flag-audit/audit_usage.py) distinguish search
evidence from an assertion that an option ran or helped. Default-enabled features
can be used without appearing in commands.

No numerical experiments were run and no default was proved optimal. The proposed
first tests are scale-polish placement, filtering/noise normalization, and solver
budgets. No runtime flags or user KEEP decisions were changed.

Six researchers can start from the [parallel handoff index](research/autocal-flag-audit/handoffs/README.md),
which assigns exclusive topics, isolated outputs, budgets and integration checks.

Lane 01 follow-up: [truth verification and solver integration](research/autocal-solver-lane01-integration.md).
The nominal HP3 scene matches the existing effective-anchor truth; the seventh
fixture remains substantially biased against it. Defaults are retained. Solver
traces now expose methods, budgets, convergence and retry attempts; an enabled
fallback no longer skips a converged primary with zero valid sweeps.
