# Autocal regression subprocesses intermittently segfault or abort in JAX

Reported October 7, 2026. Status: open; root cause unknown. This report hands off the native crashes found while refreshing calibration references. No production fix has been made.

## Impact

The ordinary full regression command sometimes loses a calibration subprocess to SIGSEGV or SIGABRT. Three baseline verification attempts segfaulted; disabling automatic cyclic garbage collection produced a fourth failure with `double free or corruption (out)`. These are native process failures, rather than Python exceptions or reference-score disagreements.

The runner correctly exits 1 when a child fails. However, its aggregate can still say `final_score=0.000` because that calculation includes only surviving datasets. The failed baseline runs reported zero over **10 datasets**, while the battery contains **11**. Require complete dataset coverage and the runner's success status before accepting a verification.

Limiting the outer runner to two concurrent datasets allowed one full baseline verification and one branch comparison to complete. That is useful evidence and a temporary workaround, but those two runs do not establish a reliable fix.

## Source and runtime at the failures

| Item | Captured value |
| --- | --- |
| Failing source | Historical main, `b60856d9d347c3bcacfd9a5bbf34c44aeda5176b` |
| Later successful branch comparison | `research_findings_1`, `4daac40c77276c14d62c86db51abfc2a40292998` |
| Interpreter | `/home/torbjorn/repos/hp-sim5/.venv/bin/python` |
| Python | `3.12.3 (main, Aug 31 2026, 10:18:26) [GCC 13.3.0]` |
| NumPy / SciPy | `2.4.3` / `1.17.1` |
| JAX / jaxlib | `0.9.2` / `0.9.2` |
| Execution | Linux, managed Codex sandbox; offline fixture replay with `--no-collect` |
| Backend | Objective module forces CPU, hides CUDA devices, enables JAX x64 |
| Child thread settings | `OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`, `MKL_NUM_THREADS=1`, `NUMEXPR_NUM_THREADS=1`, supplied by runner defaults |

`XLA_FLAGS` and `JAX_NUM_THREADS` were absent in the parent environment. The four thread variables above were also absent in the parent; `run_autocal()` sets defaults in each child's environment. It additionally defaults `PYTHONHASHSEED` to `0`. These settings do not prove that each JAX child creates only one native thread.

At report-writing time, main is `31e81bed02d41048311355472de8e4b4cc352f05`, which contains the predictive-selection commit and the new references. Do not assume today's main is the failing historical baseline. The runner, JAX objective and ellipse solver have no diff between historical baseline and this main; the history-selection change is elsewhere. The failures were already present before that change.

[Captured provenance](evidence/provenance.json) records the original revisions, versions and reference-source paths. The October 7 reference headers in `autocal/data/references/` also record dataset SHA-256 values. No calibration source or fixture contents were deliberately changed during the failure experiments.

## Observed runs, in order

All runs used `.venv/bin/python autocal/tools/regress_calibration_logs.py --no-fail-score-mismatch --keep-going`, with the adjustments shown. Full scheduling permits 11 concurrent calibration children; the Slideprinter subset contains six.

| Run / captured output | Adjustment | Outcome | Runner exit |
| --- | --- | --- | ---: |
| [generation](evidence/generation.txt) | Default scheduling, old references | All 11 generated summaries; comparison mismatches | 1 |
| [baseline verification](evidence/baseline-verification.txt) | Default scheduling | `flexible_lines_2000`: child `-11`; remaining 10 exact | 1 |
| [baseline retry](evidence/baseline-verification-retry.txt) | Same command | `flexible_lines_2000`: child `-11`; remaining 10 exact | 1 |
| [Slideprinter diagnostic](evidence/slideprinter-crash-diagnostic.txt) | `PYTHONFAULTHANDLER=1`, `--only slideprinter` | All six exact; `ALL DATASETS: PASS` | 0 |
| [full diagnostic](evidence/baseline-verification-diagnostic.txt) | `PYTHONFAULTHANDLER=1` | `sixteenth_hp3_dataset_even_more_pressure`: child `-11`; remaining 10 exact | 1 |
| [GC-disabled diagnostic](evidence/baseline-verification-gc-workaround.txt) | Faulthandler plus startup `gc.disable()` | Same sixteenth dataset: child `-6`, allocator corruption diagnostic; remaining 10 exact | 1 |
| [bounded baseline](evidence/baseline-verification-bounded.txt) | Two outer workers, normal GC, faulthandler | All 11 exact; `final_score=0.000`; `ALL DATASETS: PASS` | 0 |
| [bounded branch](evidence/branch-comparison.txt) | Same two-worker setup, candidate revision | All 11 completed; expected changed-parameter comparisons | 1 |

Linux subprocess return code `-11` means SIGSEGV; `-6` means SIGABRT. The final branch exit 1 is a comparison failure, with no reported subprocess crash. The default generation run and the six-worker subset succeeded, so neither 11-way scheduling nor either affected dataset deterministically causes a crash.

The two partial flexible-dataset child logs are preserved as [ab3de762](evidence/flexible_lines_2000_ab3de762.partial.txt) and [66a88c9d](evidence/flexible_lines_2000_66a88c9d.partial.txt). Their JSONL start records were `2026-10-07T11:54:22.657531` and `2026-10-07T11:56:43.624968`, respectively (timestamps contain no timezone). The first reaches iteration 4, after three recorded selections; the second stops during iteration 1. Neither has a final summary. Thus the failure does not consistently occur at the same replay iteration. The flexible fixture's name also does not mean flex modeling was active: recorded settings have `use_flex=false`.

## Trace evidence and code path

The full diagnostic captured this stack on the sixteenth dataset. Excerpt below omits intervening frames; the linked evidence preserves the entire stack and extension-module list.

```text
ERROR: autocal.py exited with -11
Fatal Python error: Segmentation fault
Current thread ... (most recent call first):
  Garbage-collecting
  jax/_src/mesh.py:305                       __hash__
  jax/_src/interpreters/partial_eval.py:2068 default_process_primitive
  ... JAX pjit/autodiff tracing ...
  jax/_src/numpy/lax_numpy.py:2810           where
  autocal/ellipse_objective_jax.py:338       _safe_positive_sqrt
  autocal/ellipse_objective_jax.py:351       _safe_vector_norm
  autocal/ellipse_objective_jax.py:557       _objective_core
  ... JAX value_and_grad / trace_for_jit ...
  autocal/ellipse_objective_jax.py:882       _wrapped
  autocal/ellipse_solver.py:94              _value_and_grad
  ... SciPy L-BFGS-B ...
  autocal/ellipse_solver.py:154             _run_lbfgsb_minimize
  autocal/ellipse_solver.py:1379            solve_anchors
  autocal/ellipse_solver.py:800             solve_anchors
  autocal/calibrate.py:726                  calibrate_elliptical
  autocal/planning_pass.py:185              plan_next_ellipse_sweep
  autocal/autocal.py:731                    _execute_plan_run
  autocal/autocal.py:1162                   full_auto_loop
```

With automatic cyclic GC disabled, the failure moved to native compilation:

```text
ERROR: autocal.py exited with -6
double free or corruption (out)
Fatal Python error: Aborted
Thread ... (most recent call first):
  jax/_src/compiler.py:362                  backend_compile_and_load
  jax/_src/profiler.py:384                  wrapper
  jax/_src/compiler.py:746                  _compile_and_write_cache
  jax/_src/compiler.py:478                  compile_or_get_cached
  jax/_src/interpreters/pxla.py:2845         _cached_compilation
  ... JAX compilation / pjit ...
  autocal/ellipse_objective_jax.py:882       _wrapped
  autocal/ellipse_solver.py:94              _value_and_grad
  ... same SciPy, solver and planning path ...
```

These Python stacks identify where the process was executing when it died. They do not locate the native memory corruption or prove that `Mesh.__hash__`, the safe-square-root helper, or Python GC caused it. No native backtrace or core was captured.

Relevant source at the failing revision:

- `autocal/tools/regress_calibration_logs.py:1302`: `ThreadPoolExecutor(max_workers=len(jobs))`. These outer threads each launch a separate calibration process via `run_autocal()` and `subprocess.run()` around lines 623–650. They are not eleven solver threads sharing one JAX interpreter.
- `autocal/planning_pass.py:185`: the stack enters the spool-search seed calibration. This call explicitly passes `use_parallel=False`. The inner restart ProcessPool-to-ThreadPool fallback in `ellipse_solver.py` exists, but is not demonstrated as the source of this crash.
- `autocal/ellipse_solver.py:800`: staged point filtering recurses through `solve_anchors`; line 1379 reaches the L-BFGS-B path using the JAX value/gradient wrapper. Recorded optimizer settings are `solve_optimizer=lbfgsb`, `optimizer_mode=fast`.
- Installed JAX `jax/_src/lib/__init__.py:122` registers `_xla_gc_callback`, which calls `xla_client._xla.collect_garbage()`. This is a lead for native lifetime investigation, not evidence the callback caused the failure. Disabling automatic GC also leaves possible explicit collections untested.

Every captured failed child printed a Matplotlib warning that `/home/torbjorn/.config/matplotlib` was unwritable and a temporary `/tmp` cache was created. Its relationship to the crash is unknown. A separate `multiprocessing.Semaphore(1)` probe succeeded, so a semaphore-access failure was not demonstrated. No outside-sandbox comparison was made.

## Reproduction starting point

Use the historical baseline in an isolated checkout if investigating the exact original conditions. Use its local `.venv/bin/python` with the captured dependency versions; do not silently upgrade the environment. The dated references are now on main, so supply their directory with `--ref-dir` when replaying the older revision. The runner's default `--data-dir` is `autocal/data/references`; use those fixture files and check the actual selected references and fixture hashes.

From the repo root containing the October 7 references:

```bash
PYTHONFAULTHANDLER=1 .venv/bin/python autocal/tools/regress_calibration_logs.py \
  --no-fail-score-mismatch --keep-going
```

This is an intermittent reproducer. Repeat only with a recorded experiment budget; retain each runner exit code, complete output and child scratch directory. The runner copies fixtures to `autocal/data/.regress_parallel_runs/<dataset>_<id>/`. Failed children may leave partial `.full_auto.log`, `.full_auto_log.jsonl` and `.replay_tmp.json` files even though the runner does not print their generated-log paths on failure.

For isolating one child, these are reconstructed commands matching the dataset specifications. Neither standalone command has been tested as a crash reproducer. Copy the fixture to a fresh experiment directory first; changing only the dataset to a partially replayed scratch file would change the experiment.

```bash
mkdir -p /tmp/autocal-native-crash
cp autocal/data/references/flexible_lines_2000.json /tmp/autocal-native-crash/flexible_lines_2000.json
PYTHONFAULTHANDLER=1 PYTHONHASHSEED=0 \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
.venv/bin/python autocal/autocal.py --sim --machine-type slideprinter \
  --dataset /tmp/autocal-native-crash/flexible_lines_2000.json \
  --find-radii global --base-radii 30.0 --buildup-factor 0.636619 --no-collect \
  --full-auto-log /tmp/autocal-native-crash/flexible_lines_2000.full_auto_log.jsonl

cp autocal/data/references/sixteenth_hp3_dataset_even_more_pressure.json /tmp/autocal-native-crash/sixteenth.json
PYTHONFAULTHANDLER=1 PYTHONHASHSEED=0 \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
.venv/bin/python autocal/autocal.py --sim --machine-type hangprinter_4 \
  --dataset /tmp/autocal-native-crash/sixteenth.json \
  --find-radii global --base-radii 30 --buildup-factor 0.636619 --no-collect \
  --verbose --r0-bounds 39,40 \
  --full-auto-log /tmp/autocal-native-crash/sixteenth.full_auto_log.jsonl
```

## Tested mitigations

**Disabling automatic cyclic GC failed.** A temporary `sitecustomize.py` containing `import gc; gc.disable()` was loaded through `PYTHONPATH` in the runner and children. Keep this as a failed diagnostic experiment, not a suggested fix.

**Bounding outer scheduling succeeded in two observed batteries.** The preserved [startup hook](evidence/runtime-bounded-workers/sitecustomize.py) patches only processes whose `sys.argv[0]` ends with `regress_calibration_logs.py`. It caps that process's ThreadPoolExecutor workers. Actual `autocal.py` subprocesses keep their ordinary solver behavior and normal GC. From the repo root:

```bash
PYTHONPATH="$PWD/research/bug-reports/autocal-native-crashes-october-7-2026/evidence/runtime-bounded-workers" \
AUTOCAL_REGRESSION_MAX_WORKERS=2 PYTHONFAULTHANDLER=1 \
.venv/bin/python autocal/tools/regress_calibration_logs.py \
  --no-fail-score-mismatch --keep-going
```

`AUTOCAL_REGRESSION_MAX_WORKERS` is interpreted by this experimental hook; the production runner has no such option. Setting the variable without the hook does not cap scheduling. Normal GC was restored for both successful two-worker runs. Baseline returned all 11 reference parameter tuples and intermediate results exactly, which supports numerical parity of this scheduling change on that run.

## Suggested debugging order

1. Capture a native backtrace/core from a failing child with the original versions. If it only fails under the runner, arrange debugging of the subprocess rather than only the parent. Record native thread stacks, loaded library versions, resource limits, CPU affinity, cgroup memory/CPU limits and peak memory. None of those measurements were captured at the original failure.
2. Compare fresh-fixture runs with outer concurrency 1, 2, 6 and 11, plus standalone runs of the two affected children. Use the hook to vary the worker count, and record failures per attempted run. The current evidence suggests concurrency or resource pressure may affect incidence; it does not establish a race between separate interpreters or an out-of-memory cause.
3. Reduce the failing planning seed calibration to a JAX objective/gradient reproducer, preserving sweep shapes, staged filtering, x64 and optimizer settings. Investigate native cache/GC/lifetime behavior alongside compilation. Keep the segmentation fault and allocator abort as potentially related symptoms until a native trace establishes otherwise.
4. Change one runtime factor at a time only after reproducing: JAX/jaxlib version pair, native thread settings, cache behavior, allocator diagnostics, or sandbox environment. If trying a finite-difference/legacy optimizer as a control, record that it changes numerical behavior; its success would not prove the production JAX path fixed.

No debugger, sanitizer, package upgrade/downgrade, allocator variation, XLA-flag experiment or outside-sandbox comparison was performed in this investigation. There is no confirmed upstream defect or known fixed version established here.

## Fix acceptance

Use a declared repeat count for the complete 11-dataset battery with normal GC, and demonstrate no native failures on both the historical baseline behavior and the predictive-selection behavior. Verify all datasets finish and main-baseline numerical results remain consistent with the October 7 references. The candidate branch can legitimately exit 1 for selected-parameter changes, so inspect child completion separately from comparison status. If resource bounding becomes the chosen production mitigation, make it an explicit runner setting and test its effective concurrency; do not leave correctness dependent on `sitecustomize` or treat one successful run as proof the underlying defect disappeared.

## Evidence retention

The `evidence/` directory contains byte-for-byte copies of eight runner outputs, both flexible-dataset partial logs, runtime provenance and the scheduling hook. Text captures use `.txt` so ordinary ignored runtime `.log` patterns do not hide the handoff evidence. `manifest.json` gives source paths and SHA-256 values. Absolute paths inside captures refer to the original workspace.

The original, larger session remains at `output/research/reference-refresh-october-7-2026/`, including its `research.md`, comparison analysis and GC-disabled hook. Child artifacts remain under `autocal/data/.regress_parallel_runs/`. Both locations are ignored by Git and may be absent in another checkout; the report and copied evidence are intended to survive that loss.
