# Autocal

![Autocal's own logo](autocal_logo_shine.jpeg)

Autocal collects circular sweeps and fits anchor positions from their ellipse geometry.
It chooses new sweeps by information gain. It can also fit spool radii and buildup.
At final acceptance, it checks saved estimates against sweeps collected after their fit.

## Python dependencies

The root `requirements.txt` covers the Python dependencies in `autocal/`.

JAX is optional. The default `--optimizer-mode fast` uses it on the CPU.
Autocal falls back to numerical gradients when JAX is unavailable.
Use `--optimizer-mode legacy` to disable the JAX objective.

JAX saves compiled CPU objectives in `output/autocal-jax-cache` for reuse across
launches. Set `JAX_COMPILATION_CACHE_DIR` to choose another directory, or
`JAX_ENABLE_COMPILATION_CACHE=false` to disable disk caching. Explicit user
settings are preserved; in-process JIT reuse also remains active.

JAX 0.9.2 prints a spurious native PJRT compatibility warning on disk-cache hits
([upstream issue](https://github.com/jax-ml/jax/issues/36294)). Autocal defaults
`TF_CPP_MIN_LOG_LEVEL` to `2`, suppressing native C++ INFO and WARNING messages
while retaining errors and Python warnings. Set `TF_CPP_MIN_LOG_LEVEL=1` to
restore native warnings, or `0` for INFO too.

Within each spool/filter pass, repeated exact radius/buildup models reuse up to
16 transformed datasets and their existing residual evaluations. This bounded
cache is discarded after the pass and needs no setting.
Model transformations inside fitting copy length records and configuration,
while borrowing read-only encoder/noise metadata. The public transformation
helper still deep-copies metadata by default.

From the root of the hp-sim5 repo, run:

```bash
.venv/bin/python -m pip install -r autocal/requirements-jax-cpu.txt
```

Check the installed JAX version:

```bash
.venv/bin/python - <<'PYCODE'
import jax
print(jax.__version__)
PYCODE
```


## Quick start (simulation)

Run complete simulated full-auto calibration without a browser:

```bash
.venv/bin/python autocal/autocal.py \
  --headless-sim --machine-type hangprinter_3 \
  --dataset output/headless-trial/sweeps.json \
  --find-radii global --base-radii 30 \
  --buildup-factor 0.636619 --r0-bounds 39,40
```

`--headless-sim` implies `--sim` and requires the built RRF simulator and Node
dependencies. It starts RRF and the production JavaScript physics/collector in
Node. `hangprinter_3`/`hp3` select the HP3 scene and firmware configuration;
`hangprinter_4`/`hp4` select HP4. Slideprinter, Cubecorners and Skycam also have
scene/config pairs. There is currently no headless preset for `hangprinter_5`
or Klipper. The fitting model still calls both HP3 and HP4 `hangprinter_4`.

Bootstrap, force autotuning, default point counts/noise sampling, fitting,
adaptive sweep selection and automatic stopping use the existing full-auto
loop. At acceptance, the headless backend applies both M669 and M666. One world
continues across collections and fitting passes. Collector waits advance fixed
physics steps; settling/noise timestamps use simulation time. `--speedup` retains
its collector timing semantics and is independent of Node's wall throughput.

Dataset/replay, fitting, logging, `--no-collect`, `--hp-sim-reset`, configuration
overrides and raw `--collector-args` work as usual. `--hp-sim-reset` resets once
before the first collection. Owned services stop at exit; `--keep-sim-alive`
retains them after a successful run. Explicit `--server` or
`--no-spawn-rrf-simulator` uses an existing RRF service, which autocal does not
stop. Its reported initial/final firmware parameters are captured separately
from the requested configuration file.

The printed `<dataset-stem>.headless/manifest.json` records backend, scene,
baked scene, configuration, RRF and source hashes, service ownership, simulated
and wall time, stopping reason and applied parameters. Repeated invocations
retain numbered artifact directories. Each collector output has an adjacent
`<output>.partial-points.jsonl` with completed measurements and their sweep
configuration/service identity, including on failure or interruption. Partial
journals are evidence; they are not complete datasets or resumable worlds.

Add `--extended-reference` to record physics, Python logs and stage artifacts,
collector commands/replies and measurements in one `.rrd`, without a browser
or live Viewer. Autocal starts and finalizes its own recorder in the headless
artifact directory. An existing recorder can instead be selected with
`--extended-reference-ws URL`. See the
[extended reference guide](data/references/extended-reference.md#headless-full-auto-collection)
for sampling, clocks and ownership.

Verification includes a fresh HP3 global-radius run through normal automatic
acceptance and independent three-sweep Chromium/Node collection with identical
forces/span and the default 10 points per direction (60 points total):

```bash
.venv/bin/python -m pytest autocal/tests/test_headless_sim.py -q -m slow
```

The matched-input comparison allows three RRF reporting quanta (0.03 degrees)
for raw encoders and noise means/deviations, 0.01 mm for collected
lengths/setpoints, and one 2 ms fixed step for sample durations. Timestamps must
be finite and monotonic; absolute timestamp differences are reported separately
because independent settling can cross a quiet-window boundary later.
Force/span selection is held
fixed for parity; independent adaptive autotuning can choose different spans.
The fresh full-auto test separately exercises default autotuning, later sweep
selection, patience stopping, final firmware application and process cleanup.
Automatic completion does not guarantee calibration accuracy: inspect the
solver's reported fit quality and uncertainty.
See [measured verification results](../research/headless-autocal-verification.md).

For autonomous headless HP3/HP4/RRF research, use
[`hp-sim5-research-agent`](../research/README.md) and its `start_collection` MCP
operation. Poll the returned job with `collection_status`.
The launcher supervises firmware and a continuing physics world.
Collection defaults to the production JavaScript engine in headless Node.
Collector waits advance simulation time. `--doctor` checks collection and
autocal measurement ingestion. See the [native collection guide](../research/native-collection.md)
for encoder references, force settings and calibration limits.

Follow the root README to start Vite. For a Slideprinter, open
<http://localhost:5173/hp-sim5/hp-sim/>. For a 3D machine, use
<http://localhost:5173/hp-sim5/hp-sim-3d/>.

Browser simulation requires a WebSocket connection to the open simulator. Add
`?gcode_ws=ws://localhost:8790` to the selected simulator URL; for example:
<http://localhost:5173/hp-sim5/hp-sim/?gcode_ws=ws://localhost:8790>.

If you see this, you're good to go:
![Image of hp-sim app](doc/hp-sim-startscreen.png)

Initiate simulated full-auto calibration with:

```bash
.venv/bin/python autocal/autocal.py \
  --sim \
  --machine-type slideprinter \
  --speedup 25
```

For a complete recorded browser or headless collection, follow
[extended autocal reference data](data/references/extended-reference.md).
`--extended-reference-ws ws://127.0.0.1:9877` sends timestamped Python and collector
events to the extended flight recorder, alongside physics in one `.rrd`.

`--speedup` is forwarded to the collector.
Use `--collector-args` for other raw collector flags.

Replace `slideprinter` with your machine type: `slideprinter`, `hangprinter_4`,
`hangprinter_5`, `cubecorners`, or `skycam`. The aliases `hp3`, `hp4`, and
`hangprinter_3` currently normalize to `hangprinter_4`.
Keep the hp-sim page visible during calibration.
The browser may pause the simulation when the page is hidden.

If everything went well you should see something like this:
![Image of autocal step1 finished](doc/hp-sim-after-autocal.png)

The default working dataset is `autocal/data/default_dataset.json`.
Missing datasets are bootstrapped. Existing datasets with more than three sweeps
are replayed from their first three sweeps in a temporary file.
After replay, the loop can collect new sweeps into the original dataset.

To fit stored data without starting firmware or collecting new sweeps, use an
existing dataset:

```bash
.venv/bin/python autocal/autocal.py \
  --sim --no-collect \
  --machine-type slideprinter \
  --dataset autocal/data/default_dataset.json
```

Patience or a stop threshold can end replay before all stored sweeps are used.
`--no-collect` does not prevent bootstrap if the dataset is missing.

To inspect residuals from the default anchor-only fit, add:

```bash
--residuals-csv autocal/data/default_dataset.residuals.csv
```

Then render a histogram with:

```bash
.venv/bin/python autocal/plot_residual_hist.py \
  autocal/data/default_dataset.residuals.csv \
  --output autocal/data/default_dataset.residuals.png
```

The default spool-fit schedule currently skips residual CSV and report output.
See [the detailed guide](README_elliptical_feature_calibration.md#residuals-and-logs)
for this limitation.

Here's a demo of the autocal loop on a simulated Slideprinter: https://youtu.be/XLmpuAQYbG4


## Independent stage experiments

The one-click loop composes independently callable stages:

| Stage | API | Runs an optimizer? |
| --- | --- | --- |
| Transform encoder observations into modeled lengths | `spool_model.dataset_with_modeled_lengths` | No |
| Fit and assess anchors/spools | `fit_stage.fit_ellipse_dataset` | Yes |
| Plan the next sweep from a frozen fit | `planning_pass.plan_ellipse_sweep` | No |
| Evaluate frozen history models on future sweeps | `history_selection.evaluate_history_candidates` | No |
| Rank already evaluated candidates | `history_selection.rank_history_candidates` | No |

Existing collection backends and inner initialization, refinement, filter-pass
and scale-polish modules remain separate. `plan_next_ellipse_sweep` composes fit
and planning for existing callers. Treat its fit input as a frozen value;
planning options change candidate generation, not anchors or the measurement model.

Add `--stage-artifacts output/stages` to an ordinary run to save versioned JSON
fit snapshots and, at final acceptance, evaluated history. This is opt-in;
normal runs write no stage artifacts. Existing artifact directories get a numbered
sibling so prior trials remain intact. Each snapshot includes sweep data, model
parameters, settings and source hashes. It is data for offline experiments,
not a physics checkpoint. Keep failed trials alongside successful ones.

For example, after a recorded replay produces `fit-001-default.json` and
`history-006.json`:

```bash
.venv/bin/python -m autocal.tools.replay_stage plan \
  output/stages/fit-001-default.json --output output/trial-plan.json
.venv/bin/python -m autocal.tools.replay_stage evaluate \
  output/stages/history-006.json --output output/trial-evaluation.json
.venv/bin/python -m autocal.tools.replay_stage rank \
  output/trial-evaluation.json --output output/trial-selection.json
.venv/bin/python -m autocal.tools.replay_stage fit \
  output/stages/fit-001-default.json --output output/trial-fit.json
```

`--options options.json` overrides saved keyword options for fit, plan or
evaluate, for example `{"candidate_count": 21}` for planning. Fit replay uses
the embedded sweep snapshot and disables plots/residual CSV output. Planning
writes its suggested config beside the output artifact. None of these commands
starts firmware, collects measurements or applies calibration. Stage times
exclude interpreter startup and artifact I/O; measure wall time separately when
comparing commands. A changed ranking policy can be tested against saved
predictions without rerunning optimization or prediction evaluation.

For an inexpensive fixture check, use:

```bash
.venv/bin/python autocal/tools/regress_calibration_logs.py \
  --jobs 2 --dataset-name ten_points_bigger_deltas
```

Then run the full matrix with `--jobs 2`. This bounds concurrency for reproducible
research; the default concurrency and regression acceptance rules are unchanged.
Compare returned parameters and each intermediate fit, not only the aggregate
score. Lower prediction or history scores can still choose worse physical
parameters. These stage boundaries preserve the existing selection policy.

## Typical workflow (real machine)

Remove `--sim` and `--speedup` to use a real machine:

```bash
.venv/bin/python autocal/autocal.py \
  --machine-type slideprinter
```

The loop stops after three non-improving iterations by default.
Ctrl-C during the loop requests best-so-far acceptance.
Final selection favors successful held-out predictions when available.
These scores measure fit and prediction quality. They do not prove anchor accuracy.

Collection checks the latest 1.5 seconds of encoder readings for quiet settling,
with a range limit scaled to the longer 5-second vibration window's drift allowance.
It retains that longer history for the existing vibration check. This reduces
waiting after transients while keeping point counts and automatic stopping.

On acceptance, RRF receives M669 for the selected anchors.
Fitted M666 spool settings are printed but are not sent automatically.
Klipper skips the M669 send.
Reaching `--max-steps` ends the loop without applying calibration.

Useful options:

- `--dataset` chooses the working dataset.
- `--firmware klipper` selects the Klipper API-mode backend. RRF is the default.
- `--solve-optimizer` accepts `lbfgsb` (default), `lm`, or `trf`.
- `--find-radii` and `--find-buildup-factor` enable spool fitting.
  Each accepts `global` or `per-anchor`. Both default to `off`.
- `--sparse-recovery` adds a sparse seed fit during underconstrained recovery.
- `--shotgun` adds solver variants from `shotgun.conf`.


For full details and log interpretation, see
[`README_elliptical_feature_calibration.md`](README_elliptical_feature_calibration.md).
The [algorithm overview](optimization_algorithm_overview) describes the loops.
The [objective overview](objective_functions_overview) explains optimization,
patience, prediction ranking, and displayed quality.
