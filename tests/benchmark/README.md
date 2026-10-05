# Native HP4 logo benchmark

Run from the repository root with `.venv/bin/python`. The benchmark uses the
default `public/usd_scenes/hp4_rigid_body.usda`, its 2 ms timestep, line layering,
open-loop motors and two cable solver iterations. It consumes the same scheduled
commands as the browser, with no viewer, sleep or recording unless requested.

Generate commands from `public/gcode/Hangprinter_logo6.gcode` using the firmware
implementations, then schedule them with the production browser players:

```bash
mkdir -p /tmp/hp4-logo-vsd/sys /tmp/hp4-logo-vsd/gcodes /tmp/hp4-logo-vsd/logs
cp -r RRF/run/vsd/sys/. /tmp/hp4-logo-vsd/sys/
cp public/gcode/Hangprinter_logo6.gcode /tmp/hp4-logo-vsd/gcodes/logo.gcode
RRF/build/rrf_simulator --vsd /tmp/hp4-logo-vsd \
  --gcode gcodes/logo.gcode --can-log logs/logo.csv \
  -c sys/config_hp4_w_line_layers.g
node scripts/headless_commands.mjs /tmp/hp4-logo-vsd/logs/logo.csv \
  /tmp/hp4-rrf.json .002 RRF/run/vsd/sys/config_hp4_w_line_layers.g

scripts/gcode_to_mcu_commands.sh -m hp4 --line-layers public/gcode/Hangprinter_logo6.gcode
node scripts/headless_commands.mjs public/mcu_commands/hp4/Hangprinter_logo6_buildup.serial \
  /tmp/hp4-klipper.json
```

The RRF simulator must be built first (see the root README). Keep `homeall.g`
in the temporary VSD: the public logo requests homing. Alternatively, schedule
the browser's existing `public/RRF_CAN_commands/Hangprinter_logo6_hp4_w_line_layers.can`
using the same RRF configuration argument. Configuration matters because the
HP4 extruder is CAN driver 44.

```bash
PYTHONPATH=src/python .venv/bin/python tests/benchmark/hp4_logo.py \
  --commands /tmp/hp4-rrf.json --steps 3000 --repeats 3 --output /tmp/rrf-result.json
PYTHONPATH=src/python .venv/bin/python tests/benchmark/hp4_logo.py \
  --commands /tmp/hp4-klipper.json --steps 3000 --repeats 3 --output /tmp/klipper-result.json
```

These measure the first six seconds, including travel and deposition for both
firmwares. Omit `--steps` to run the complete print: each command stream contains
about one million timesteps (34 minutes of simulated motion). Run timings
sequentially without other benchmarks or tests competing for CPU.

JSON results separate command parsing, machine construction and simulation
timings; include the command SHA-256, Python/platform, median throughput and final
frame/cable snapshot; and require exact repeatability within each engine version.
Compare before/after snapshots separately from timing. Differences between RRF
and Klipper trajectories are expected; they have different planners and command
streams.

Add `--profile /tmp/hp4.prof` to profile a separate run after the unprofiled
measurements. Inspect it with:

```bash
.venv/bin/python - <<'PY'
import pstats
pstats.Stats('/tmp/hp4.prof').strip_dirs().sort_stats('cumtime').print_stats(30)
PY
```

Add `--rrd /tmp/logo.rrd` to include every-step native Rerun recording and the
final flush in timings. Each repeat produces its own numbered RRD. Recording
can dominate runtime and file size, especially as the deposited point cloud
grows; measure it separately from physics-only runs.

The optimization results and validation are in [HP4_LOGO_RESULTS.md](HP4_LOGO_RESULTS.md).
